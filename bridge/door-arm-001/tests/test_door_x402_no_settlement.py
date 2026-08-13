"""Real Tunnel proof that failed / timed-out / replayed door-arm-001
actions never call the x402 settle endpoint.

Mirrors Spot PR #58's test_x402_no_settlement.py structure: the real Go
Tunnel binary is driven through the local Fabric proxy and the recording
facilitator; the simulator side injects failure, silence (timeout), or a
second dispatch (replay). Settlement must stay at zero on every negative
path.
"""

from __future__ import annotations

import json
import os
import subprocess
import tempfile
import threading
import time
import unittest
import uuid
from pathlib import Path

import zenoh

from x402_harness import (
    FacilitatorHandler,
    LocalFabricProxy,
    NETWORK,
    PAYEE,
    find_tunnel_binary,
    http_post,
    payment_signature_from_402,
    start_facilitator,
)

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ROOT = PACKAGE_ROOT.parents[2]
SKILL_CATALOG = (
    ROOT
    / "registry/vendors/laok/door-arm-001"
    / "bridge/door-arm-001"
    / "skill-catalog.json"
)
ACTION_TOPIC = "robot/tunnel/action"
RESULT_TOPIC = "robot/tunnel/result"
ROBOT_ID = "door_arm_001_nosettle"
ZENOH_TEST_PORT = int(os.environ.get("DOOR_NOSETTLE_ZENOH_PORT", "7447"))
EXECUTION_TIMEOUT_SECONDS = "5"


class InjectedFabricSimulator:
    """Zenoh simulator double that injects a correlated failure or timeout."""

    def __init__(self):
        config = zenoh.Config.from_json5(
            '{"mode":"peer","scouting":{"multicast":{"enabled":false}},'
            f'"listen":{{"endpoints":["tcp/127.0.0.1:{ZENOH_TEST_PORT}"]}}}}'
        )
        self.session = zenoh.open(config)
        self.mode = "fail"
        self.actions: list[dict] = []
        self.publisher = self.session.declare_publisher(RESULT_TOPIC)
        self.subscriber = self.session.declare_subscriber(ACTION_TOPIC, self.on_action)

    def on_action(self, sample) -> None:
        event = json.loads(bytes(sample.payload.to_bytes()))
        self.actions.append(event)
        if self.mode == "silent":
            return  # timeout path: no result ever published
        self.publisher.put(
            json.dumps(
                {
                    "action_id": event.get("action_id", ""),
                    "robot_id": event.get("robot_id", ""),
                    "skill_id": event.get("skill_id", ""),
                    "params_hash": event.get("params_hash", ""),
                    "idempotency_key": event.get("idempotency_key", ""),
                    "status": "failure",
                    "error_code": "SIMULATOR_EXECUTION_FAILED",
                    "result": {"message": "injected failure", "metrics": {}},
                }
            ).encode("utf-8")
        )

    def close(self) -> None:
        try:
            self.subscriber.undeclare()
            self.publisher.undeclare()
            self.session.close()
        except Exception:
            pass


class DoorNoSettlementTests(unittest.TestCase):
    def _setup(self, simulator_mode: str):
        tunnel_binary = find_tunnel_binary(ROOT)
        if not tunnel_binary:
            self.skipTest("Build the real Tunnel first with make build")

        proxy = LocalFabricProxy()
        facilitator, facilitator_thread = start_facilitator()
        simulator = InjectedFabricSimulator()
        simulator.mode = simulator_mode
        proxy.start()
        tunnel = None
        temp_dir_obj = tempfile.TemporaryDirectory(prefix="fabric_nosettle_")
        temp_dir = Path(temp_dir_obj.name)

        config_path = temp_dir / "tunnel.json"
        config_path.write_text(
            json.dumps(
                {
                    "robot_id": ROBOT_ID,
                    "evm_payee_address": PAYEE,
                    "price": "$0.10",
                    "network": NETWORK,
                }
            ),
            encoding="utf-8",
        )
        zenoh_config = temp_dir / "zenoh.json5"
        zenoh_config.write_text(
            json.dumps(
                {
                    "mode": "peer",
                    "scouting": {"multicast": {"enabled": False}},
                    "connect": {"endpoints": [f"tcp/127.0.0.1:{ZENOH_TEST_PORT}"]},
                }
            ),
            encoding="utf-8",
        )
        child_env = os.environ.copy()
        child_env.update(
            {
                "PROXY_WS_URL": f"ws://127.0.0.1:{proxy.port}/ws",
                "FACILITATOR_URL": f"http://127.0.0.1:{facilitator.server_address[1]}",
                "AIP_ENABLED": "false",
                "ZENOH_CONFIG": str(zenoh_config),
                "SKILL_CATALOG_PATH": str(SKILL_CATALOG),
                "ALLOWED_ACTIONS": "open_door,stop",
                "MAX_ACTION_DURATION_SECONDS": "30",
                "EXECUTION_TIMEOUT_SECONDS": EXECUTION_TIMEOUT_SECONDS,
            }
        )
        tunnel = subprocess.Popen(
            [tunnel_binary, "--config", str(config_path)],
            cwd=ROOT,
            env=child_env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        self.assertIsNotNone(
            proxy.wait_for_connection(15),
            "real Tunnel did not connect to the local Fabric proxy",
        )
        return proxy, facilitator, facilitator_thread, simulator, tunnel, temp_dir_obj

    def _teardown(self, proxy, facilitator, facilitator_thread, simulator, tunnel, temp_dir_obj):
        if simulator is not None:
            simulator.close()
        if tunnel is not None and tunnel.poll() is None:
            tunnel.terminate()
            try:
                tunnel.wait(timeout=5)
            except subprocess.TimeoutExpired:
                tunnel.kill()
        proxy.close()
        facilitator.shutdown()
        facilitator.server_close()
        facilitator_thread.join(timeout=5)
        temp_dir_obj.cleanup()

    def _paid_post(self, action_url, unpaid_headers, action_id):
        return http_post(
            action_url,
            {
                "action": "open_door",
                "robot_id": ROBOT_ID,
                "action_id": action_id,
                "idempotency_key": action_id,
                "params": {"door": "open"},
            },
            {"PAYMENT-SIGNATURE": payment_signature_from_402(unpaid_headers)},
        )

    def _unpaid_headers(self, action_url):
        _, headers, _ = http_post(
            action_url, {"action": "open_door", "robot_id": ROBOT_ID}
        )
        return headers

    def _settle_count(self) -> int:
        return len([p for p, _ in FacilitatorHandler.calls if p == "/settle"])

    def test_failed_execution_never_calls_settle(self) -> None:
        proxy = facilitator = facilitator_thread = simulator = tunnel = None
        temp_dir_obj = None
        try:
            proxy, facilitator, facilitator_thread, simulator, tunnel, temp_dir_obj = \
                self._setup("fail")
            action_url = f"http://127.0.0.1:{proxy.port}/robots/{ROBOT_ID}/action"
            headers = self._unpaid_headers(action_url)
            aid = f"fabric-nosettle-fail-{uuid.uuid4().hex}"
            status, _, _ = self._paid_post(action_url, headers, aid)
            self.assertEqual(status, 202)
            # failure result is injected quickly; give the tunnel time to record
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                if self._settle_count() > 0:
                    break
                time.sleep(0.3)
            self.assertEqual(self._settle_count(), 0, "failure must never settle")
        finally:
            self._teardown(proxy, facilitator, facilitator_thread, simulator, tunnel, temp_dir_obj)

    def test_timeout_never_calls_settle(self) -> None:
        proxy = facilitator = facilitator_thread = simulator = tunnel = None
        temp_dir_obj = None
        try:
            proxy, facilitator, facilitator_thread, simulator, tunnel, temp_dir_obj = \
                self._setup("silent")
            action_url = f"http://127.0.0.1:{proxy.port}/robots/{ROBOT_ID}/action"
            headers = self._unpaid_headers(action_url)
            aid = f"fabric-nosettle-timeout-{uuid.uuid4().hex}"
            status, _, _ = self._paid_post(action_url, headers, aid)
            self.assertEqual(status, 202)
            # wait past the tunnel's execution timeout
            time.sleep(float(EXECUTION_TIMEOUT_SECONDS) + 3)
            self.assertEqual(self._settle_count(), 0, "timeout must never settle")
        finally:
            self._teardown(proxy, facilitator, facilitator_thread, simulator, tunnel, temp_dir_obj)

    def test_replay_never_double_settles(self) -> None:
        """A replayed idempotency key is rejected (409) and never re-actuates."""
        proxy = facilitator = facilitator_thread = simulator = tunnel = None
        temp_dir_obj = None
        try:
            proxy, facilitator, facilitator_thread, simulator, tunnel, temp_dir_obj = \
                self._setup("fail")
            action_url = f"http://127.0.0.1:{proxy.port}/robots/{ROBOT_ID}/action"
            headers = self._unpaid_headers(action_url)
            aid = f"fabric-nosettle-replay-{uuid.uuid4().hex}"
            # first dispatch
            status1, _, _ = self._paid_post(action_url, headers, aid)
            self.assertEqual(status1, 202)
            # same key again -> replay rejected
            status2, _, _ = self._paid_post(action_url, headers, aid)
            self.assertEqual(status2, 409, "replayed idempotency key must be rejected")
            self.assertLessEqual(len(simulator.actions), 1,
                                 "replay must not publish a second ActionEvent")
            time.sleep(1)
            self.assertEqual(self._settle_count(), 0, "replay must never settle")
        finally:
            self._teardown(proxy, facilitator, facilitator_thread, simulator, tunnel, temp_dir_obj)


if __name__ == "__main__":
    unittest.main(verbosity=2)
