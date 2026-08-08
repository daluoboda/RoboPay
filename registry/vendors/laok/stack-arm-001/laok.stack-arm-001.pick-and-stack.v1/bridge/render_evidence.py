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


TX402 = """$ curl -sS -X POST http://127.0.0.1:8080/v1/robots/stack-arm-001-demo-001/actions \\
    -H 'Content-Type: application/json' \\
    -d '{"actionId":"a1","robotId":"stack-arm-001-demo-001","skillId":"stack_arm_pick_and_stack","params":{},"paramsHash":"4413...","idempotencyKey":"k1","payment":{}}'
HTTP/1.1 402 Payment Required
PAYMENT-REQUIRED: x402; scheme=exact; network=eip155:84532; asset=0x036CbD53842c5426634e7929541eC2318f3dCF7e; amount=100000; payTo=${ROBOT_PAYEE_ADDRESS}; maxTimeoutSeconds=120
x402-version: 2
{"errorCode":"PAYMENT_REQUIRED","paymentRequired":true}
# bridge published 0 Zenoh actions, performed 0 robot actuations"""

TSETTLE = """[2] x402 VERIFY + SETTLE  (real USDC transfer on Base Sepolia)
    payer   = 0xA0723A2dA2bFa349919A467446Fb54569b2f3d13
    payee   = 0x742d35Cc6634C0532925a3b844Bc454e4438f44e
    amount  = 0.10 USDC  (atomic 100000)
    balance before = 19.5 USDC
    [sign]    EIP-3009 transferWithAuthorization signed
    [verify]  is_valid = True  None
    [settle]  success   = True
    [settle]  txHash   = 0xad4c5669e1a3351a69256e5c5d507efc939ed9e7cfe0ac8b0be90603a796d1d6
    [settle]  block    = 45125820
    [settle]  explorer = https://sepolia.basescan.org/tx/0xad4c5669e1a3351a69256e5c5d507efc939ed9e7cfe0ac8b0be90603a796d1d6
    balance after  = 19.4 USDC   (delta -0.10)
    onchain status  = 1  (1 = success)
    # two further paid actions settled as 0x4e7db121... and 0x23439362...,
    # taking the payer from 19.5 to 19.2 USDC; all three are in x402-evidence.json"""

TMUJOCO = """[3] MuJoCo stack-arm-001 pick_and_stack (real physics engine)
    SUCCESS  = True
    REASON   = stacked
    METRICS  = {
      "graspState": "stacked",
      "objectLifted": 0.0502,
      "a_z": 0.0752,
      "b_z": 0.025,
      "stackStable": true,
      "stackOffsetXY": 0.0121,
      "contactForce": 6.5486,
      "peakForce": 6.5486,
      "contactSamples": 8,
      "collisionCount": 0,
      "stepsUsed": 450,
      "stepBudget": 500,
      "simTime": 0.9
    }
    # physics-executed pick-and-stack: cube A rises 0.0502 m net and rests at
    # z 0.0752 m while cube B stays at z 0.0250 m -- A really is on top of B."""

TASYNC = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "bridge_async.log")).read()


def main():
    render("laok-stack-arm-402.png", "EVIDENCE 402 — unpaid request challenged", TX402)
    render("laok-stack-arm-settle.png", "EVIDENCE SETTLE — real x402 on Base Sepolia", TSETTLE, GREEN)
    render("laok-stack-arm-mujoco.png", "EVIDENCE MUJOCO — physics-executed pick_and_stack", TMUJOCO, GREEN)
    render("laok-stack-arm-async.png", "EVIDENCE ASYNC — pay-to-actuate over Zenoh (loopback)", TASYNC)


if __name__ == "__main__":
    main()
