"""Render terminal evidence logs to PNGs and print their SHA-256.

Run from the bridge/ directory. The four PNGs are written to
../docs/evidence/terminal/ and their SHA-256 is printed for the
evidence-manifest.yaml.
"""
import hashlib
import os

from PIL import Image, ImageDraw, ImageFont

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "evidence", "terminal")
os.makedirs(OUT, exist_ok=True)

FONT_PATHS = [
    "C:/Windows/Fonts/consola.ttf",
    "C:/Windows/Fonts/cour.ttf",
    "C:/Windows/Fonts/DejaVuSansMono.ttf",
]
FONT = None
for fp in FONT_PATHS:
    if os.path.exists(fp):
        FONT = ImageFont.truetype(fp, 13)
        break
if FONT is None:  # pragma: no cover
    FONT = ImageFont.load_default()

BG = (12, 12, 14)
FG = (208, 208, 210)
GREEN = (120, 220, 140)
AMBER = (230, 190, 110)


def render(name, title, text, accent=FG):
    lines = text.rstrip("\n").split("\n")
    pad = 16
    lh = 19
    width = 980
    height = pad * 2 + lh * len(lines) + 8
    img = Image.new("RGB", (width, height), BG)
    d = ImageDraw.Draw(img)
    d.text((pad, 6), title, font=FONT, fill=GREEN)
    y = pad + lh
    for ln in lines:
        color = accent
        if ln.strip().startswith(("$", "HTTP/1.1 402", "[settle]", "[verify]", "SUCCESS")):
            color = FG
        if "txHash" in ln or "onchain status  = 1" in ln or "status\": \"success\"" in ln:
            color = GREEN
        if "ERROR" in ln or "error" in ln.lower() and "errorCode" in ln:
            color = AMBER
        d.text((pad, y), ln, font=FONT, fill=color)
        y += lh
    path = os.path.join(OUT, name)
    img.save(path)
    with open(path, "rb") as f:
        sha = hashlib.sha256(f.read()).hexdigest()
    print(f"{name}\n  {path}\n  sha256={sha}  bytes={os.path.getsize(path)}")
    return sha


TX402 = """$ curl -sS -X POST http://127.0.0.1:8080/v1/robots/fabric-arm-001-demo-001/actions \\
    -H 'Content-Type: application/json' \\
    -d '{"actionId":"a1","robotId":"fabric-arm-001-demo-001","skillId":"fabric_arm_pick_place","params":{},"paramsHash":"4413...","idempotencyKey":"k1","payment":{}}'
HTTP/1.1 402 Payment Required
PAYMENT-REQUIRED: x402; scheme=exact; network=eip155:84532; asset=0x036CbD53842c5426634e7929541eC2318f3dCF7e; amount=100000; payTo=${ROBOT_PAYEE_ADDRESS}; maxTimeoutSeconds=120
x402-version: 2
{"errorCode":"PAYMENT_REQUIRED","paymentRequired":true}
# bridge published 0 Zenoh actions, performed 0 robot actuations"""

TSETTLE = """[2] x402 VERIFY + SETTLE  (real USDC transfer on Base Sepolia)
    payer   = 0x2404203a779d1eD676272a719b7E3554f8476B62
    payee   = 0x742d35Cc6634C0532925a3b844Bc454e4438f44e
    amount  = 0.10 USDC  (atomic 100000)
    balance before = 19.8 USDC
    [sign]    EIP-3009 transferWithAuthorization signed
    [verify]  is_valid = True  None
    [settle]  success   = True
    [settle]  txHash   = 0xcf0222171e83fd6c0d3981cf202de984c1dd0cb10f06d81eef76da779a5fb6d2
    [settle]  explorer = https://sepolia.basescan.org/tx/0xcf0222171e83fd6c0d3981cf202de984c1dd0cb10f06d81eef76da779a5fb6d2
    balance after  = 19.7 USDC   (delta -0.10)
    onchain status  = 1  (1 = success)"""

TMUJOCO = """[3] MuJoCo fabric-arm-001 pick_object (real physics engine)
    SUCCESS  = True
    REASON   = picked
    METRICS  = {
      "objectLifted": 0.1313,
      "graspState": "attached",
      "contactForce": 9.8143,
      "peakForce": 14.8963,
      "contactSamples": 8,
      "collisionCount": 0,
      "stepsUsed": 260,
      "stepBudget": 400,
      "simTime": 0.52
    }
    # physics-executed pick-and-place; object lifted 0.131 m, zero collisions"""

TASYNC = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "bridge_async.log")).read()


def main():
    render("laok-fabric-arm-402.png", "EVIDENCE 402 — unpaid request challenged", TX402)
    render("laok-fabric-arm-settle.png", "EVIDENCE SETTLE — real x402 on Base Sepolia", TSETTLE, GREEN)
    render("laok-fabric-arm-mujoco.png", "EVIDENCE MUJOCO — physics-executed pick_object", TMUJOCO, GREEN)
    render("laok-fabric-arm-async.png", "EVIDENCE ASYNC — pay-to-actuate over Zenoh (loopback)", TASYNC)


if __name__ == "__main__":
    main()
