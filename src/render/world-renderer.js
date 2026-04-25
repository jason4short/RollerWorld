// Draws the bot and ground. Wraps the bot's x position around the view
// (Asteroids-style) — physics state keeps true x; only the drawn position
// is wrapped so controllers and plots see continuous motion.

export class WorldRenderer {
  constructor(canvas, pxPerMeter = 180) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.pxPerM = pxPerMeter;
  }

  draw(state, params, navTargetX = null) {
    const { L, R } = params;
    const ctx = this.ctx;
    const W = this.canvas.width, H = this.canvas.height;
    ctx.clearRect(0, 0, W, H);

    const R_px    = R * this.pxPerM;
    const groundY = H - 40;
    const axleY   = groundY - R_px;

    ctx.strokeStyle = '#333'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(0, groundY); ctx.lineTo(W, groundY); ctx.stroke();

    ctx.strokeStyle = '#1a1a1a';
    for (let i = -10; i <= 10; i++) {
      const gx = W / 2 + i * this.pxPerM * 0.5;
      ctx.beginPath(); ctx.moveTo(gx, 0); ctx.lineTo(gx, groundY); ctx.stroke();
    }

    const viewM  = W / this.pxPerM;
    const halfM  = viewM / 2;
    const wrappedX = ((state.x + halfM) % viewM + viewM) % viewM - halfM;
    const cx_px = W / 2 + wrappedX * this.pxPerM;
    const cy_px = axleY;

    // Nav target flag (wraps the same way the bot does).
    if (navTargetX !== null) {
      const wt = ((navTargetX + halfM) % viewM + viewM) % viewM - halfM;
      const tx = W / 2 + wt * this.pxPerM;
      ctx.strokeStyle = '#6cf'; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(tx, groundY); ctx.lineTo(tx, groundY - 40); ctx.stroke();
      ctx.fillStyle = '#6cf';
      ctx.beginPath(); ctx.moveTo(tx, groundY - 40); ctx.lineTo(tx + 12, groundY - 34); ctx.lineTo(tx, groundY - 28); ctx.closePath(); ctx.fill();
    }

    // body (tilted)
    ctx.save();
    ctx.translate(cx_px, cy_px);
    ctx.rotate(state.pitch);
    const L_px = L * this.pxPerM;
    ctx.strokeStyle = '#bbb'; ctx.lineWidth = 3;
    for (const dx of [-16, 16]) {
      ctx.beginPath(); ctx.moveTo(dx, 0); ctx.lineTo(dx, -L_px * 1.4); ctx.stroke();
    }
    ctx.fillStyle = '#a33';
    ctx.fillRect(-18, -L_px - 18, 36, 36);
    ctx.fillStyle = '#2a2a2a';
    ctx.fillRect(-22, -L_px * 1.4 - 6, 44, 8);
    ctx.fillStyle = '#ec6';
    ctx.beginPath(); ctx.arc(0, -L_px, 5, 0, Math.PI * 2); ctx.fill();
    ctx.restore();

    // wheel
    ctx.fillStyle = '#222';
    ctx.strokeStyle = '#888'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(cx_px, axleY, R_px, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    const spoke = state.x / R;
    ctx.beginPath();
    ctx.moveTo(cx_px + Math.cos(spoke) * R_px * 0.85, axleY + Math.sin(spoke) * R_px * 0.85);
    ctx.lineTo(cx_px - Math.cos(spoke) * R_px * 0.85, axleY - Math.sin(spoke) * R_px * 0.85);
    ctx.stroke();
    ctx.fillStyle = '#888';
    ctx.beginPath(); ctx.arc(cx_px, axleY, 4, 0, Math.PI * 2); ctx.fill();
  }
}
