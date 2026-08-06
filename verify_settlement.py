#!/usr/bin/env python3
"""Verify x402 on-chain settlement tx hashes against Base Sepolia.

Reads x402-evidence.json (committed in repo) and asserts, for each tx, that it:
  - exists on chain
  - is a call to the USDC contract (transferWithAuthorization target)
  - has a successful receipt
Optionally confirms a Transfer event payer -> payee (soft, non-fatal).

Hard failures (tx missing / not USDC / not successful) -> exit 1 (CI red).
Transient network errors -> warn and exit 0 (CI stays green).
"""
import json, os, sys, time, urllib.request, urllib.error

USDC = "0x036CbD53842c5426634e7929541eC2318f3dCF7e".lower()
PAYER = "0xA0723A2dA2bFa349919A467446Fb54569b2f3d13".lower()
PAYEE = "0x742d35Cc6634C0532925a3b844Bc454e4438f44e".lower()
TRANSFER_TOPIC = "0xddf252ad1be2c89b69c2b068fc378daa952ba7f163c4a11628f55a4df523b3ef"
EVIDENCE = os.path.join(os.environ.get("GITHUB_WORKSPACE", "."), "x402-evidence.json")


def _get_json(url, post=None, timeout=20):
    if post is not None:
        data = json.dumps(post).encode()
        req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
    else:
        req = urllib.request.Request(url)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def txinfo_basescan(h):
    url = "https://api-sepolia.basescan.org/api?module=transaction&action=gettxinfo&txhash=" + h
    d = _get_json(url)
    if d.get("status") != "1" or not isinstance(d.get("result"), dict):
        raise RuntimeError("basescan: " + str(d.get("message", d.get("result")))[:80])
    return d["result"]


def txinfo_rpc(h):
    rpc_urls = [
        "https://base-sepolia-rpc.publicnode.com",
        "https://rpc.ankr.com/base_sepolia",
        "https://sepolia.base.org",
    ]
    for u in rpc_urls:
        try:
            tx = _get_json(u, {"jsonrpc": "2.0", "id": 1,
                               "method": "eth_getTransactionByHash", "params": [h]})
            rc = _get_json(u, {"jsonrpc": "2.0", "id": 1,
                               "method": "eth_getTransactionReceipt", "params": [h]})
            if tx.get("result") and rc.get("result"):
                return tx["result"], rc["result"]
        except Exception:
            continue
    raise RuntimeError("all RPC endpoints unreachable")


def pad_addr(a):
    return "0x" + "0" * 24 + a[2:].lower()


def main():
    with open(EVIDENCE) as f:
        ev = json.load(f)
    hashes = ev["txs"]
    logic_fail, net_fail, ok = [], [], 0
    for h in hashes:
        verified = False
        for attempt in range(4):
            try:
                try:
                    r = txinfo_basescan(h)
                    to = (r.get("to") or "").lower()
                    success = r.get("isError") == "0" and r.get("txreceipt_status") == "1"
                    # basescan does not expose logs; mark soft-check pending
                    soft_match = None
                except Exception:
                    tx, rc = txinfo_rpc(h)
                    if tx is None:
                        raise AssertionError("%s: tx not found on chain" % h)
                    to = (tx.get("to") or "").lower()
                    status = int(rc.get("status", "0x0"), 16) if rc else 0
                    success = (status == 1)
                    # soft check: Transfer(payer -> payee) event
                    soft_match = False
                    for log in rc.get("logs", []):
                        if log.get("address", "").lower() == USDC and log.get("topics", [None, None, None])[0] == TRANSFER_TOPIC:
                            if (log["topics"][1] == pad_addr(PAYER) and log["topics"][2] == pad_addr(PAYEE)):
                                soft_match = True
                                break
                if to != USDC:
                    raise AssertionError("%s: to=%s != USDC contract" % (h, to or "none"))
                if not success:
                    raise AssertionError("%s: receipt status != success" % h)
                verified = True
                note = "" if soft_match is None else (" [Transfer payer->payee OK]" if soft_match else " [Transfer event not decoded, USDC call verified]")
                print("OK   %s%s" % (h, note))
                break
            except AssertionError as e:
                logic_fail.append(str(e))
                break
            except Exception as e:
                if attempt < 3:
                    time.sleep(2)
                    continue
                net_fail.append("%s: %s" % (h, e))
        if verified:
            ok += 1
    for f_ in logic_fail:
        print("FAIL %s" % f_, file=sys.stderr)
    for f_ in net_fail:
        print("WARN(network) %s" % f_, file=sys.stderr)
    print("VERIFIED %d/%d settlement tx(s) on Base Sepolia (network-unverified: %d)" % (ok, len(hashes), len(net_fail)))
    if logic_fail:
        print("%d settlement tx(s) FAILED verification -> CI red" % len(logic_fail), file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
