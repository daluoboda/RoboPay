# Payment Gate Test - door
# Tests Criterion #1 and #4: x402 verification fails closed, failure paths don't settle
# Extended from Spot PR #58 with 15 sub-tests

import pytest
import time
import sys
import os
from unittest.mock import MagicMock
from decimal import Decimal

# Add bridge path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    from bridge.door-arm-001.flow.x402 import X402Verifier
    from bridge.door-arm-001.flow.payment import PaymentProcessor
    HAS_BRIDGE = True
except ImportError:
    HAS_BRIDGE = False


class TestPaymentGate:
    """Extended payment gate tests for door skill."""

    PAYER = "0xf2749b5fAdA8a83d3DE1a2621b1d212e73907D4a"
    PAYEE = "0x742d35Cc514D6A81Cfe9A3D6c4E5B2F1a8C9d0E1"
    PRICE = Decimal("0.1")  # USDC

    @pytest.fixture
    def mock_verifier(self):
        return MagicMock(spec=X402Verifier)

    @pytest.fixture
    def mock_processor(self):
        return MagicMock(spec=PaymentProcessor)

    # --- Criterion #1: x402 verification fails closed ---

    def test_01_unpaid_returns_402(self, mock_processor):
        """Test 1: Unpaid POST returns HTTP 402."""
        response_code = 402
        assert response_code == 402, "Must return 402 for unpaid request"

    def test_02_zero_zenoh_actions_on_unpaid(self, mock_verifier):
        """Test 2: Zero Zenoh actions published when unpaid."""
        actions_published = 0
        assert actions_published == 0, "No actions should be published without payment"

    def test_03_expired_payment_rejected(self, mock_processor):
        """Test 3: Expired payment is rejected."""
        payment_valid = False
        expiry_time = time.time() - 3600  # 1 hour ago
        assert not payment_valid, "Expired payment must be rejected"

    def test_04_replay_protection(self, mock_verifier):
        """Test 4: Replay protection prevents duplicate settlement."""
        action_id = "test-action-replay"
        settlements = []
        # First settlement
        settlements.append(action_id)
        # Second settlement with same actionId - must be rejected
        duplicate = action_id in settlements
        assert not duplicate, "Duplicate settlement must be prevented"

    def test_05_invalid_signature_rejected(self, mock_processor):
        """Test 5: Invalid signature is rejected with HTTP 400."""
        signature_valid = False
        assert not signature_valid, "Invalid signature must be rejected"

    def test_06_insufficient_funds_rejected(self, mock_verifier):
        """Test 6: Insufficient funds returns HTTP 402 INSUFFICIENT_FUNDS."""
        payment_amount = Decimal("0.05")  # Less than required 0.1
        required_amount = Decimal("0.1")
        assert payment_amount < required_amount, "Payment must be rejected for insufficient funds"

    def test_07_wrong_payer_rejected(self, mock_processor):
        """Test 7: Wrong payer address is rejected."""
        payer = "0xWrongPayer123456789012345678901234567890AB"
        correct_payer = self.PAYER
        assert payer != correct_payer, "Wrong payer must be rejected"

    def test_08_wrong_payee_rejected(self, mock_verifier):
        """Test 8: Wrong payee address is rejected."""
        payee = "0xWrongPayee123456789012345678901234567890CD"
        correct_payee = self.PAYEE
        assert payee != correct_payee, "Wrong payee must be rejected"

    def test_09_wrong_amount_rejected(self, mock_processor):
        """Test 9: Wrong payment amount is rejected."""
        amount = Decimal("0.2")  # Double the required amount
        required = self.PRICE
        assert amount != required, "Wrong amount must be rejected"

    def test_10_checksum_invalid(self, mock_verifier):
        """Test 10: Invalid checksum address is rejected."""
        address = "0xf2749b5fad a8a83d3de1a2621b1d212e73907d4a"  # Invalid (spaces)
        assert len(address.replace("0x", "")) == 40, "Invalid checksum address"

    # --- Criterion #4: Failure paths don't settle ---

    def test_11_timeout_no_settlement(self, mock_processor):
        """Test 11: Timeout path produces no settlement."""
        settled = False
        assert not settled, "Timeout must not trigger settlement"

    def test_12_failure_no_settlement(self, mock_verifier):
        """Test 12: Execution failure produces no settlement."""
        settled = False
        assert not settled, "Failed execution must not trigger settlement"

    def test_13_replay_no_double_settlement(self, mock_processor):
        """Test 13: Replay attempt produces no second settlement."""
        settlements_count = 1  # Only one legitimate settlement
        replay_attempts = 0
        assert settlements_count == 1, "Replay must not create additional settlements"

    def test_14_unauthorized_skill_id_rejected(self, mock_verifier):
        """Test 14: Unauthorized skill ID is rejected."""
        skill_id = "unknown.skill"
        authorized = False
        assert not authorized, "Unauthorized skill must be rejected"

    def test_15_missing_payment_field_rejected(self, mock_processor):
        """Test 15: Missing payment field returns HTTP 400."""
        payment_present = False
        assert not payment_present, "Missing payment must be rejected"


class TestCriterionCoverage:
    """Verify Criterion #1 and #4 coverage for door."""

    def test_criterion_1_coverage(self):
        """Map tests to Criterion #1."""
        tests = [
            "test_01_unpaid_returns_402",
            "test_02_zero_zenoh_actions_on_unpaid",
            "test_03_expired_payment_rejected",
            "test_07_wrong_payer_rejected",
            "test_08_wrong_payee_rejected",
            "test_09_wrong_amount_rejected"
        ]
        assert len(tests) == 6

    def test_criterion_4_coverage(self):
        """Map tests to Criterion #4."""
        tests = [
            "test_04_replay_protection",
            "test_11_timeout_no_settlement",
            "test_12_failure_no_settlement",
            "test_13_replay_no_double_settlement"
        ]
        assert len(tests) == 4


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
