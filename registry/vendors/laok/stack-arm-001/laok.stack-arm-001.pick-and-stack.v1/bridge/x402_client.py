"""x402 payment gate + optional live facilitator verify/settle for stack-arm-001.

Two operating modes:

1. Envelope gating (default, offline-friendly). The action envelope already
   carries a verified x402 payment object (the shape returned by an x402
   client after it calls the facilitator /verify endpoint). We validate the
   envelope fields against the advertised payment policy: provider, network,
   asset, amount, payee, status, verified flag, expiry, and the
   "reject already-settled" rule.

2. Live facilitator verification/settlement (production path). When a raw
   signed x402 PaymentPayload is supplied (the PAYMENT-SIGNATURE header), we
   reconstruct the PaymentRequirements we advertised and ask the x402
   facilitator to verify/settle it on-chain. This is what produced the real
   Base Sepolia settlement evidence for this profile.
"""
from __future__ import annotations

import os
import time
import re
from datetime import datetime, timezone

# Live facilitator is optional: the bridge must still run for local demos /
# tests even if x402.org is unreachable.
try:
    from x402 import PaymentRequirements, PaymentPayload
    from x402.http import FacilitatorConfig, HTTPFacilitatorClientSync
    _X402_AVAILABLE = True
except Exception:  # pragma: no cover
    _X402_AVAILABLE = False

FACILITATOR_URL = os.environ.get("X402_FACILITATOR", "https://x402.org/facilitator")

TXHASH_RE = re.compile(r"^0x[0-9a-fA-F]{64}$")


def _now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _parse(ts: str) -> float:
    try:
        return datetime.fromisoformat(ts.replace("Z", "+00:00")).timestamp()
    except Exception:
        return 0.0


class PaymentError(Exception):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


class PaymentGate:
    """Validates an x402 payment envelope against a payment policy."""

    def __init__(self, policy: dict, payee: str):
        self.policy = policy
        self.payee = payee
        self.required = next(
            (p for p in policy.get("policies", []) if p.get("required")), None
        )

    def check(self, payment: dict) -> None:
        """Raise PaymentError if the payment is not an authorized, live gate."""
        if not isinstance(payment, dict):
            raise PaymentError("PAYMENT_MISSING", "envelope.payment is required")
        if payment.get("provider") != "x402":
            raise PaymentError("PAYMENT_PROVIDER", "provider must be x402")
        req = self.required or {}
        accepted_networks = [self.policy.get("network", "")]
        pi_networks = self.policy.get("piNetworks", [])
        accepted_networks.extend(pi_networks)
        if payment.get("network") not in accepted_networks:
            raise PaymentError("PAYMENT_NETWORK",
                f"network mismatch: {payment.get('network')} not in {accepted_networks}")
        # Asset check: skip for Pi networks (native token)
        if payment.get("network") not in pi_networks:
            if payment.get("asset", "").lower() != str(self.policy.get("asset", "")).lower():
                raise PaymentError("PAYMENT_ASSET", "asset mismatch")
        # Amount: use Pi-specific amount for Pi networks
        if payment.get("network") in pi_networks:
            pi_policy = next((p for p in self.policy.get("piPolicies", [])), {})
            expected = pi_policy.get("amount", req.get("amount", ""))
        else:
            expected = req.get("amount", "")
        if str(payment.get("amount")) != str(expected):
            raise PaymentError("PAYMENT_AMOUNT", "amount mismatch")
        pay_to = payment.get("payTo", "")
        if self.payee and pay_to and pay_to.lower() != self.payee.lower():
            # Allow the ${ENV} placeholder form.
            if pay_to != "${ROBOT_PAYEE_ADDRESS}":
                raise PaymentError("PAYMENT_PAYEE", "payTo mismatch")
        if not payment.get("verified") or payment.get("status") != "authorized":
            raise PaymentError("PAYMENT_UNAUTHORIZED", "payment not authorized")
        if req.get("rejectAlreadySettled") and payment.get("settled"):
            raise PaymentError("PAYMENT_ALREADY_SETTLED", "payment already settled")
        now = time.time()
        if payment.get("expiresAt") and _parse(payment["expiresAt"]) < now:
            raise PaymentError("PAYMENT_EXPIRED", "payment authorization expired")


def build_requirements(payment: dict, payee: str) -> "PaymentRequirements":
    return PaymentRequirements(
        scheme="exact",
        network=payment["network"],
        amount=str(payment["amount"]),
        asset=payment["asset"],
        pay_to=payee,
        max_timeout_seconds=120,
        extra={
            "resource": "robopay://stack-arm-001/pick_and_stack",
            "description": "One physics-executed pick_and_stack on stack-arm-001.",
        },
    )


def verify_with_facilitator(payload_json: dict, payee: str) -> dict:
    """Verify a signed x402 PaymentPayload via the live facilitator.

    Returns the facilitator VerifyResponse dict. Raises PaymentError on failure.
    """
    if not _X402_AVAILABLE:
        raise PaymentError("FACILITATOR_UNAVAILABLE", "x402 lib not installed")
    payload = PaymentPayload.model_validate(payload_json)
    req = build_requirements(payload_json.get("payment", payload_json), payee)
    fc = HTTPFacilitatorClientSync(FacilitatorConfig(url=FACILITATOR_URL, timeout=30))
    resp = fc.verify(payload, req)
    if not getattr(resp, "is_valid", False):
        raise PaymentError("FACILITATOR_REJECT", str(getattr(resp, "invalid_reason", "")))
    return resp


def settle_with_facilitator(payload_json: dict, payee: str) -> str:
    """Settle a signed x402 PaymentPayload via the live facilitator.

    Returns the on-chain transaction hash. Raises PaymentError on failure.
    """
    if not _X402_AVAILABLE:
        raise PaymentError("FACILITATOR_UNAVAILABLE", "x402 lib not installed")
    payload = PaymentPayload.model_validate(payload_json)
    req = build_requirements(payload_json.get("payment", payload_json), payee)
    fc = HTTPFacilitatorClientSync(FacilitatorConfig(url=FACILITATOR_URL, timeout=30))
    resp = fc.settle(payload, req)
    tx = getattr(resp, "transaction", None)
    if not tx:
        raise PaymentError("SETTLE_FAILED", str(getattr(resp, "error_reason", "")))
    if not TXHASH_RE.match(str(tx)):
        raise PaymentError("FACILITATOR_BAD_TX", f"tx hash format invalid: {tx[:20]}...")
    return tx
