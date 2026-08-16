"""Zenoh bridge for paid fetch-arm-001 ``fetch_object`` actions.

Production payment boundary -- RoboPay Tunnel + x402 facilitator
==============================================================

The RoboPay Tunnel (the shared Go ``tunnel/`` binary) enforces the x402
payment gate with a custom execution-gated settlement middleware: it answers
HTTP 402 for unpaid requests, verifies a paid request synchronously, and then
publishes the action to the ``robot/tunnel/action`` Zenoh topic and returns
202 *accepted*. Settlement -- the actual USDC transfer -- is performed by the
Tunnel's x402 facilitator **only after this bridge publishes a successful
terminal result** on ``robot/tunnel/result``. A failed or timed-out execution
never settles, and a replayed idempotency key / payment payload is rejected
with 409 before anything is published.

This bridge is a fail-closed Zenoh *subscriber*: it only sees already-paid
actions, runs the real MuJoCo physics for ``fetch_object``, and publishes the
terminal result echoing the correlation tuple the Tunnel issued
(action_id, robot_id, skill_id, params_hash, idempotency_key) so the Tunnel
settles strictly on success. It never verifies or settles a payment itself.
"""
from __future__ import annotations

import hashlib
import json
import logging
import os
import time
from dataclasses import dataclass
from typing import Any, Dict

from flow.executor import MuJoCoExecutor


LOGGER = logging.getLogger("robopay.fetch")

ACTION_TOPIC = "robot/tunnel/action"
RESULT_TOPIC = "robot/tunnel/result"
METRICS_TOPIC = "robot/fetch-arm-001/metrics"

ROBOT_ID = "fetch-arm-001"
ALLOWED_ACTIONS = {"fetch_object"}
PROFILE_ID = "fetch.fetch-arm-001.mujoco-sim.v1"

KNOWN_OBJECTS = {
    "fetchable", "unreachable", "collision", "timeout",
    "far_box", "blocked_box", "slow_box",
}
MAX_STEPS_BOUND = (1, 2000)


class ActionContractError(ValueError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


@dataclass
class BridgeSettings:
    robot_id: str
    action_topic: str
    result_topic: str
    metrics_topic: str

    @classmethod
    def from_env(cls) -> "BridgeSettings":
        def configured(name: str, default: str) -> str:
            return os.environ.get(name, default).strip() or default

        return cls(
            robot_id=configured("ROBOT_ID", ROBOT_ID),
            action_topic=configured("ZENOH_ACTION_TOPIC", ACTION_TOPIC),
            result_topic=configured("ZENOH_RESULT_TOPIC", RESULT_TOPIC),
            metrics_topic=configured("ZENOH_METRICS_TOPIC", METRICS_TOPIC),
        )


def _params_hash(params: Dict[str, Any]) -> str:
    return hashlib.sha256(
        json.dumps(params, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()[:16]


def _validate_params(params: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(params, dict):
        raise ActionContractError("INVALID_PARAMS", "params must be an object.")
    unexpected = sorted(set(params) - {"object", "maxSteps"})
    if unexpected:
        raise ActionContractError(
            "INVALID_PARAMS", f"unregistered parameter(s): {', '.join(unexpected)}"
        )
    obj = params.get("object", "fetchable")
    if obj not in KNOWN_OBJECTS:
        raise ActionContractError("INVALID_PARAMS", f"unknown object scene: {obj!r}")
    max_steps = params.get("maxSteps")
    if max_steps is not None:
        if isinstance(max_steps, bool) or not isinstance(max_steps, int):
            raise ActionContractError("INVALID_STEPS", "maxSteps must be an integer.")
        if not MAX_STEPS_BOUND[0] <= max_steps <= MAX_STEPS_BOUND[1]:
            raise ActionContractError(
                "INVALID_STEPS",
                f"maxSteps must be between {MAX_STEPS_BOUND[0]} and {MAX_STEPS_BOUND[1]}.",
            )
    return params


class FetchZenohBridge:
    """Fail-closed Zenoh action bridge with correlated simulator results."""

    def __init__(self, settings: BridgeSettings | None = None):
        try:
            import zenoh
        except ImportError as error:
            raise RuntimeError("Install eclipse-zenoh to run the fetch bridge.") from error

        self._zenoh = zenoh
        self.settings = settings or BridgeSettings.from_env()
        self.robot_id = self.settings.robot_id
        self.action_topic = self.settings.action_topic
        self.result_topic = self.settings.result_topic
        self.metrics_topic = self.settings.metrics_topic

        self._executor = MuJoCoExecutor()
        self._session = zenoh.open(zenoh.Config())
        self._result_publisher = self._session.declare_publisher(self.result_topic)
        self._metrics_publisher = self._session.declare_publisher(self.metrics_topic)
        self._subscriber = self._session.declare_subscriber(
            self.action_topic, self._on_action
        )

    def _publish(self, action_id, robot_id, skill_id, params_hash, idempotency_key,
                 status, result, params) -> None:
        envelope = {
            "action_id": action_id,
            "robot_id": robot_id,
            "skill_id": skill_id,
            "profile_id": PROFILE_ID,
            "params_hash": params_hash,
            "idempotency_key": idempotency_key,
            "status": status,
            "result": result,
        }
        payload = json.dumps(envelope).encode("utf-8")
        self._metrics_publisher.put(payload)
        self._result_publisher.put(payload)

    def _on_action(self, sample) -> None:
        raw = getattr(sample, "payload", sample)
        if hasattr(raw, "to_bytes"):
            raw = raw.to_bytes()
        try:
            event = json.loads(bytes(raw))
        except Exception:
            LOGGER.error("Rejected malformed ActionEvent before simulation.")
            return

        payload = event.get("payload") or {}
        action = (payload.get("action") or "").lower()
        params = payload.get("params") or {}

        action_id = event.get("action_id") or ""
        robot_id = event.get("robot_id") or self.robot_id
        skill_id = event.get("skill_id") or action
        params_hash = event.get("params_hash") or _params_hash(params)
        idempotency_key = event.get("idempotency_key") or action_id

        if action not in ALLOWED_ACTIONS:
            self._publish(action_id, robot_id, skill_id, params_hash, idempotency_key,
                          "failure", {"success": False, "error_code": "UNREGISTERED_ACTION",
                                      "message": f"action {action!r} is not registered"}, params)
            return

        try:
            _validate_params(params)
        except ActionContractError as error:
            self._publish(action_id, robot_id, skill_id, params_hash, idempotency_key,
                          "failure", {"success": False, "error_code": error.code,
                                      "message": str(error)}, params)
            return

        try:
            res = self._executor.execute("fetch_object", params)
        except Exception as error:
            LOGGER.exception("Simulator execution failed")
            self._publish(action_id, robot_id, skill_id, params_hash, idempotency_key,
                          "failure", {"success": False, "error_code": "SIMULATOR_EXECUTION_ERROR",
                                      "message": str(error)}, params)
            return

        self._publish(action_id, robot_id, skill_id, params_hash, idempotency_key,
                      "success" if res.success else "failure",
                      {"success": res.success, "message": res.message, "metrics": res.metrics},
                      params)

    def spin(self) -> None:
        LOGGER.info("fetch-arm-001 bridge %s listening on %s", self.robot_id, self.action_topic)
        try:
            while True:
                time.sleep(0.1)
        finally:
            self.close()

    def close(self) -> None:
        try:
            self._subscriber.undeclare()
            self._result_publisher.undeclare()
            self._metrics_publisher.undeclare()
            self._session.close()
        except Exception:
            pass


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    FetchZenohBridge().spin()


if __name__ == "__main__":
    main()
