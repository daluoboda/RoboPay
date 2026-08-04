"""Payment layer (D1 skeleton).

State machine:
    AUTHORIZED -> EXECUTING -> SUCCESS (settle) / FAILED (no settle)

D1 uses MOCK verification + a local settlement ledger.
D7 replaces verify_payment / SettlementLedger with the real x402 facilitator
on Base Sepolia. The interfaces here are the swap points -- nothing else changes.
"""
from enum import Enum


class PaymentState(str, Enum):
    AUTHORIZED = "AUTHORIZED"
    EXECUTING = "EXECUTING"
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"


class PaymentError(Exception):
    pass


def verify_payment(payment: dict | None) -> dict:
    """Mock verification for D1.

    A payment is considered verified when it carries a txHash.
    D7: call the x402 facilitator here and return the verified receipt.
    """
    if not payment:
        raise PaymentError("no payment attached")
    tx_hash = payment.get("txHash")
    if not tx_hash:
        raise PaymentError("missing txHash")
    # D1 mock: accept any txHash-bearing payment as verified.
    return {"verified": True, "txHash": tx_hash}


class SettlementLedger:
    """Local stand-in for on-chain settlement (D7 swaps for real facilitator)."""

    def __init__(self):
        self.settled = {}  # action_id -> payment

    def settle(self, action_id: str, payment: dict) -> dict:
        self.settled[action_id] = payment
        return {"settled": True, "actionId": action_id}

    def skip(self, action_id: str) -> dict:
        # Failure path: payment MUST NOT be settled.
        return {"settled": False, "actionId": action_id, "reason": "execution_failed"}
