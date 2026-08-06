"""laok stack-arm-001 RoboPay bridge (Tier 1 simulated manipulator).

Pay-to-Actuate path
-------------------
1. A client requests the skill catalog (GET /v1/robots/{id}/skills).
2. An unpaid action request returns HTTP 402 + `PAYMENT-REQUIRED`.
3. A paid request (PAYMENT-SIGNATURE header + verified x402 envelope) is
   accepted immediately (202) and published to the Zenoh topic
   `robot/tunnel/action`.
4. The bridge validates the envelope (paramsHash, payment gate, idempotency),
   runs the MuJoCo pick_and_stack skill, and publishes the correlated result to
   `robot/tunnel/result`.
5. Settlement is eligible only after a `success` result (failure never settles).

The bridge runs end-to-end without the external Fabric proxy: if the `zenoh`
package is available it uses a real Zenoh session; otherwise it falls back to
an in-process loopback transport so the async action/result flow is still
exercisable and testable offline.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sqlite3
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import yaml

from x402_client import PaymentError, PaymentGate, settle_with_facilitator

try:
    from simulator import MuJoCoSimulator
except Exception:  # pragma: no cover
    MuJoCoSimulator = None  # type: ignore

HERE = os.path.dirname(os.path.abspath(__file__))
PROFILE_DIR = os.path.dirname(HERE)

SKILL_ID = "stack_arm_pick_and_stack"
ACTION_TOPIC = "robot/tunnel/action"
RESULT_TOPIC = "robot/tunnel/result"


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------
def _canonical(params: object) -> str:
    return json.dumps(params, sort_keys=True, separators=(",", ":"))


def params_hash(params: object) -> str:
    return hashlib.sha256(_canonical(params).encode()).hexdigest()


def _now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


# --------------------------------------------------------------------------
# idempotency store
# --------------------------------------------------------------------------
class IdempotencyStore:
    def __init__(self, path: str = ":memory:"):
        self.conn = sqlite3.connect(path, check_same_thread=False)
        self.conn.execute(
            "CREATE TABLE IF NOT EXISTS records ("
            "idempotencyKey TEXT PRIMARY KEY,"
            "actionId TEXT,"
            "authorizationId TEXT,"
            "fingerprint TEXT,"
            "resultStatus TEXT)"
        )
        self.conn.commit()
        self.lock = threading.Lock()

    def seen_key(self, key: str):
        with self.lock:
            row = self.conn.execute(
                "SELECT actionId, authorizationId, resultStatus FROM records WHERE idempotencyKey=?",
                (key,),
            ).fetchone()
            return row

    def seen_auth(self, auth_id: str):
        with self.lock:
            row = self.conn.execute(
                "SELECT idempotencyKey, actionId FROM records WHERE authorizationId=?",
                (auth_id,),
            ).fetchone()
            return row

    def record(self, key, action_id, auth_id, fingerprint, status):
        with self.lock:
            self.conn.execute(
                "INSERT OR REPLACE INTO records VALUES (?,?,?,?,?)",
                (key, action_id, auth_id, fingerprint, status),
            )
            self.conn.commit()


# --------------------------------------------------------------------------
# transport (zenoh or loopback)
# --------------------------------------------------------------------------
class LoopbackTransport:
    """In-process pub/sub used when zenoh is unavailable."""

    def __init__(self, on_action):
        self._on_action = on_action
        self._subscribers = []

    def publish_action(self, envelope: dict):
        # Synchronously dispatch to the local handler to emulate Zenoh delivery.
        self._on_action(envelope)

    def publish_result(self, result: dict):
        for cb in self._subscribers:
            cb(result)

    def on_result(self, cb):
        self._subscribers.append(cb)


class ZenohTransport:
    def __init__(self, on_action):
        import zenoh  # imported lazily so the bridge imports without zenoh

        self._z = zenoh.open(zenoh.Config())
        self._on_action = on_action
        self._sub = self._z.declare_subscriber(ACTION_TOPIC, self._recv)
        self._pub = self._z

    def _recv(self, sample):
        try:
            env = json.loads(sample.payload.decode())
            self._on_action(env)
        except Exception as e:  # pragma: no cover
            print(f"[zenoh] bad action payload: {e}")

    def publish_action(self, envelope: dict):
        self._pub.put(ACTION_TOPIC, json.dumps(envelope).encode())

    def publish_result(self, result: dict):
        self._pub.put(RESULT_TOPIC, json.dumps(result).encode())


# --------------------------------------------------------------------------
# bridge
# --------------------------------------------------------------------------
class Bridge:
    def __init__(self, robot_id: str, payee: str, db: str = ":memory:"):
        self.robot_id = robot_id
        self.payee = payee
        with open(os.path.join(PROFILE_DIR, "payment-policy.yaml")) as f:
            self.policy = yaml.safe_load(f)
        self.gate = PaymentGate(self.policy, payee)
        self.idem = IdempotencyStore(db)
        self.sim = MuJoCoSimulator() if MuJoCoSimulator else None
        self.actions: dict[str, dict] = {}
        self.transport = None  # set by run()

    # --- catalog ---
    def list_skills(self) -> dict:
        pol = next(p for p in self.policy["policies"] if p.get("required"))
        return {
            "robotId": self.robot_id,
            "skills": [
                {
                    "skillId": SKILL_ID,
                    "name": "Physics-executed pick-and-place",
                    "paymentRequired": True,
                    "amount": str(pol["amount"]),
                    "displayAmount": pol.get("displayAmount"),
                }
            ],
        }

    # --- payment header handling ---
    def _payment_from_header(self, header: str | None, envelope: dict) -> dict:
        if header:
            try:
                signed = json.loads(header)
                pay = signed.get("payment", signed)
                if isinstance(pay, dict) and pay.get("provider") == "x402":
                    return pay
            except Exception:
                pass
        return envelope.get("payment", {})

    # --- core request ---
    def request_action(self, envelope: dict, payment_header: str | None = None) -> tuple[int, dict]:
        # structure
        for f in ("actionId", "robotId", "skillId", "params", "paramsHash", "idempotencyKey", "payment"):
            if f not in envelope:
                return 400, {"errorCode": "BAD_ENVELOPE", "error": f"missing {f}"}
        if envelope["skillId"] != SKILL_ID:
            return 400, {"errorCode": "UNKNOWN_SKILL", "error": envelope["skillId"]}
        # params hash
        if envelope["paramsHash"] != params_hash(envelope["params"]):
            return 400, {"errorCode": "PARAMS_HASH_MISMATCH", "error": "paramsHash invalid"}
        # payment gate
        payment = self._payment_from_header(payment_header, envelope)
        if not payment:
            return 402, {"errorCode": "PAYMENT_REQUIRED", "paymentRequired": True}
        try:
            self.gate.check(payment)
        except PaymentError as e:
            code = 402 if e.code in ("PAYMENT_EXPIRED", "PAYMENT_UNAUTHORIZED",
                                      "PAYMENT_ALREADY_SETTLED", "PAYMENT_ISSUED_FUTURE") else 400
            return code, {"errorCode": e.code, "error": e.message}
        # idempotency
        key = envelope["idempotencyKey"]
        auth = payment.get("authorizationId")
        existing = self.idem.seen_key(key)
        if existing:
            if existing[0] != envelope["actionId"]:
                return 409, {"errorCode": "IDEMPOTENCY_CONFLICT",
                              "error": "idempotencyKey reused with different actionId"}
            # same key+action -> duplicate, never re-execute
            return 409, {"errorCode": "DUPLICATE",
                          "error": "already processed",
                          "cachedStatus": existing[2],
                          "secondDelivery": "cached-terminal-reference"}
        if auth:
            seen = self.idem.seen_auth(auth)
            if seen and seen[0] != key:
                return 409, {"errorCode": "PAYMENT_AUTHORIZATION_REPLAY",
                              "error": "authorization already used"}
        # accept -> publish to transport (async execution)
        self.actions[envelope["actionId"]] = {"status": "pending"}
        fp = params_hash([envelope["actionId"], envelope["robotId"], envelope["skillId"],
                          envelope["paramsHash"], auth])
        self.idem.record(key, envelope["actionId"], auth or "", fp, "pending")
        if self.transport:
            self.transport.publish_action(envelope)
        else:
            self._execute(envelope, key, auth, fp)
        return 202, {"status": "accepted", "actionId": envelope["actionId"],
                      "bodyStatus": "accepted", "correlationField": "actionId"}

    # --- execution (invoked by transport) ---
    def _execute(self, envelope: dict, key: str, auth: str | None, fp: str):
        action_id = envelope["actionId"]
        try:
            if self.sim is None:
                raise RuntimeError("MuJoCo simulator unavailable")
            res = self.sim.pick_and_stack(envelope.get("params", {}) or {})
            success = bool(getattr(res, "success", False))
            reason = getattr(res, "reason", "unknown")
            metrics = getattr(res, "metrics", {})
        except Exception as e:
            success = False
            reason = f"ACTION_TIMEOUT:{e}"
            metrics = {}
        status = "success" if success else "error"
        result = {
            "actionId": action_id,
            "robotId": envelope["robotId"],
            "skillId": envelope["skillId"],
            "status": status,
            "reason": reason,
            "metrics": metrics,
            "settlementEligible": success,
            "completedAt": _now_iso(),
        }
        if success:
            # settle only after success (live facilitator if a real signed payment exists)
            payment = envelope.get("payment", {})
            if isinstance(payment.get("payload"), dict):
                try:
                    result["settlementTx"] = settle_with_facilitator(payment["payload"], self.payee)
                except Exception as se:  # pragma: no cover
                    result["settlementError"] = str(se)
        self.actions[action_id] = result
        self.idem.record(key, action_id, auth or "", fp, status)
        if self.transport:
            self.transport.publish_result(result)
        return result

    def handle_action(self, envelope: dict):
        """Transport callback: executes and publishes the result."""
        key = envelope.get("idempotencyKey", "")
        auth = envelope.get("payment", {}).get("authorizationId")
        fp = params_hash([envelope.get("actionId"), envelope.get("robotId"),
                          envelope.get("skillId"), envelope.get("paramsHash"), auth])
        self._execute(envelope, key, auth, fp)

    def get_status(self, action_id: str) -> tuple[int, dict]:
        r = self.actions.get(action_id)
        if not r:
            return 404, {"errorCode": "NOT_FOUND"}
        return 200, r


# --------------------------------------------------------------------------
# HTTP server (functions layer)
# --------------------------------------------------------------------------
def make_handler(bridge: Bridge):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):  # pragma: no cover
            pass

        def _send(self, code, body):
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            if code == 402:
                self.send_header("PAYMENT-REQUIRED",
                                 f"x402; scheme=exact; network={bridge.policy['network']}; "
                                 f"asset={bridge.policy['asset']}; "
                                 f"amount={bridge.policy['policies'][0]['amount']}; "
                                 f"payTo=${{ROBOT_PAYEE_ADDRESS}}; maxTimeoutSeconds=120")
            self.end_headers()
            self.wfile.write(json.dumps(body).encode())

        def do_GET(self):
            p = urlparse(self.path)
            if p.path.endswith("/skills"):
                self._send(200, bridge.list_skills())
            elif p.path.rfind("/actions/") != -1:
                aid = p.path.rsplit("/actions/", 1)[1]
                self._send(*bridge.get_status(aid))
            else:
                self._send(404, {"error": "not found"})

        def do_POST(self):
            p = urlparse(self.path)
            if not p.path.endswith("/actions"):
                self._send(404, {"error": "not found"})
                return
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length) or b"{}")
            header = self.headers.get("PAYMENT-SIGNATURE")
            code, resp = bridge.request_action(body, header)
            self._send(code, resp)

    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-id", default=os.environ.get("ROBOT_ID", "stack-arm-001-demo-001"))
    ap.add_argument("--payee", default=os.environ.get("ROBOT_PAYEE_ADDRESS", ""))
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--db", default="bridge_idempotency.db")
    args = ap.parse_args()

    bridge = Bridge(args.robot_id, args.payee, args.db)
    try:
        bridge.transport = ZenohTransport(bridge.handle_action)
        print(f"[transport] zenoh (topics {ACTION_TOPIC} / {RESULT_TOPIC})")
    except Exception as e:
        bridge.transport = LoopbackTransport(bridge.handle_action)
        print(f"[transport] loopback (zenoh unavailable: {e})")
    bridge.transport.on_result(lambda r: print(f"[result] {r['actionId']} -> {r['status']}"))

    srv = ThreadingHTTPServer((args.host, args.port), make_handler(bridge))
    print(f"[http] listening on http://{args.host}:{args.port}")
    print(f"[robot] {bridge.robot_id}  payee={bridge.payee or '<env ROBOT_PAYEE_ADDRESS>'}")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:  # pragma: no cover
        srv.shutdown()


if __name__ == "__main__":
    main()
