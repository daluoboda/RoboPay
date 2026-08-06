"""Drive the laok fabric-arm-001 bridge to produce live async evidence.

Uses the in-process loopback transport (zenoh not required) so the async
action/result contract is exercised and captured as a terminal log.
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from laok_fabric_arm_zenoh_bridge import Bridge, LoopbackTransport, params_hash

PAYEE = "0x742d35Cc6634C0532925a3b844Bc454e4438f44e"
ROBOT = "fabric-arm-001-demo-001"


def paid(action_id, idem, auth, verified=True, expired=False):
    params = {}
    pay = {
        "provider": "x402", "network": "eip155:84532",
        "asset": "0x036CbD53842c5426634e7929541eC2318f3dCF7e",
        "amount": "100000", "payTo": PAYEE,
        "authorizationId": auth, "verified": verified, "status": "authorized",
        "settled": False, "issuedAt": "2099-01-01T00:00:00Z",
        "expiresAt": "2000-01-01T00:00:00Z" if expired else "2099-01-01T00:05:00Z",
    }
    return {"actionId": action_id, "robotId": ROBOT, "skillId": "fabric_arm_pick_place",
            "params": params, "paramsHash": params_hash(params),
            "idempotencyKey": idem, "payment": pay}


def main():
    bridge = Bridge(ROBOT, PAYEE, ":memory:")
    bridge.transport = LoopbackTransport(bridge.handle_action)

    print("=" * 70)
    print(" laok fabric-arm-001 — ASYNC PAY-TO-ACTUATE EVIDENCE (loopback transport)")
    print("=" * 70)

    print("\n[A] UNPAID REQUEST  ->  HTTP 402 + PAYMENT-REQUIRED")
    code, resp = bridge.request_action(
        {"actionId": "a1", "robotId": ROBOT, "skillId": "fabric_arm_pick_place",
         "params": {}, "paramsHash": params_hash({}), "idempotencyKey": "k1", "payment": {}})
    print(f"    HTTP {code}  {resp}")

    print("\n[B] VALID PAID ACTION  ->  HTTP 202 accepted, async result on result topic")
    code, resp = bridge.request_action(paid("act_001", "idem_001", "auth_A"))
    print(f"    HTTP {code}  {resp}")
    print(f"    [result topic] {json.dumps(bridge.actions['act_001'])}")

    print("\n[C] DUPLICATE idempotencyKey (same envelope)  ->  DUPLICATE, no second execution")
    code, resp = bridge.request_action(paid("act_001", "idem_001", "auth_A"))
    print(f"    HTTP {code}  {resp}")

    print("\n[D] PAYMENT authorizationId REPLAY  ->  PAYMENT_AUTHORIZATION_REPLAY")
    code, resp = bridge.request_action(paid("act_002", "idem_002", "auth_A"))
    print(f"    HTTP {code}  {resp}")

    print("\n[E] EXPIRED authorization  ->  HTTP 402 PAYMENT_EXPIRED")
    code, resp = bridge.request_action(paid("act_003", "idem_003", "auth_B", expired=True))
    print(f"    HTTP {code}  {resp}")

    print("\n[done] async evidence captured")


if __name__ == "__main__":
    main()
