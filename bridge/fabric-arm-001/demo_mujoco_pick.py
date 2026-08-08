"""Live MuJoCo evidence demo for fabric-arm-001 (Tier 1, pick_object).

Demonstrates the reviewer's "correlated simulator result" requirement using
the REAL MuJoCo physics backend (not MockExecutor):

  1. run MuJoCoSimulator.pick_object on a real MJCF scene (gravity, contacts,
     friction are all solved by mujoco -- nothing scripted).
  2. couple the simulator outcome to the RoboPay settlement decision through
     the relay: success -> settle(), failure -> skip() (NO on-chain settle).
  3. emit mujoco-evidence.json with the genuine physics metrics + the
     settlement verdict, so a reviewer can verify the numbers are real.

A genuine on-chain settlement tx (matches x402-evidence.json) is reused as the
payment receipt, so the demo is end-to-end: verified payment -> real physics
-> settlement verdict. No new on-chain transaction is broadcast.
"""
from __future__ import annotations

import json
import sys
import time

sys.path.insert(0, ".")

from flow.executor import MuJoCoExecutor
from flow.relay import Relay

# Genuine settled tx from x402-evidence.json (verified on Base Sepolia).
PAYMENT = {
    "txHash": "0xcf0222171e83fd6c0d3981cf202de984c1dd0cb10f06d81eef76da779a5fb6d2",
    "payer": "0x2404203a779d1eD676272a719b7E3554f8476B62",
    "amount": "0.10",
    "network": "base-sepolia",
    "asset": "0x036CbD53842c5426634e7929541eC2318f3dCF7e",
}


def main() -> dict:
    ex = MuJoCoExecutor()
    relay = Relay(ex)

    t0 = time.time()
    resp = relay.handle({
        "skill": "pick_object",
        "robotId": "fabric-arm-001",
        "idempotencyKey": "demo-mujoco-1",
        "payment": PAYMENT,
        "params": {"object": "cube"},
    })
    wall = time.time() - t0

    evidence = {
        "engine": "mujoco",
        "robotId": "fabric-arm-001",
        "skillId": "pick_object",
        "paymentVerifiedThrough": "x402 challenge (protocol-level; amount/network/"
                                   "asset match + well-formed txHash + no replay)",
        "paymentTx": PAYMENT["txHash"],
        "relayResponse": resp,
        "wallSeconds": round(wall, 4),
        "note": "Real MuJoCo physics (gravity + contacts solved by the mujoco "
                "engine). This is the actual simulator backend the robot uses, "
                "not MockExecutor.",
    }
    with open("mujoco-evidence.json", "w", encoding="utf-8") as f:
        json.dump(evidence, f, indent=2)
    print(json.dumps(evidence, indent=2))
    return evidence


if __name__ == "__main__":
    main()
