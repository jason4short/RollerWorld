// 2D analog stick. Drag the knob; release auto-centers (RC pitch/roll feel).
// Outputs `{x, y}` each in [-1, 1]. +x = screen-right, +y = screen-up.

export class Joystick {
	constructor(canvas, { selfCenter = true } = {}) {
		this.canvas		= canvas;
		this.ctx		= canvas.getContext('2d');
		this.selfCenter = selfCenter;
		this.x = 0; this.y = 0;
		this.dragging	= false;
		this.pointerId	= null;
		this._bindPointer();
		this._scheduleDraw();
		window.addEventListener('resize', () => this._scheduleDraw());
	}

	value() { return { x: this.x, y: this.y }; }

	_scheduleDraw() { requestAnimationFrame(() => this._draw()); }

	_bindPointer() {
		const c = this.canvas;
		c.style.touchAction = 'none';
		c.addEventListener('pointerdown', e => {
			this.dragging	= true;
			this.pointerId	= e.pointerId;
			c.setPointerCapture(e.pointerId);
			this._updateFromEvent(e);
		});
		c.addEventListener('pointermove', e => {
			if (!this.dragging || e.pointerId !== this.pointerId) return;
			this._updateFromEvent(e);
		});
		const end = e => {
			if (e.pointerId !== this.pointerId) return;
			this.dragging	= false;
			this.pointerId	= null;
			if (this.selfCenter) { this.x = 0; this.y = 0; }
			this._draw();
		};
		c.addEventListener('pointerup', end);
		c.addEventListener('pointercancel', end);
	}

	_updateFromEvent(e) {
		const rect	= this.canvas.getBoundingClientRect();
		const cx	= rect.width  / 2;
		const cy	= rect.height / 2;
		const r		= Math.min(cx, cy) - this._knobR();
		const px	= e.clientX - rect.left - cx;
		const py	= e.clientY - rect.top  - cy;
		this.x = Math.max(-1, Math.min(1,  px / r));
		this.y = Math.max(-1, Math.min(1, -py / r));
		this._draw();
	}

	_knobR() {
		const w = this.canvas.clientWidth || this.canvas.width;
		return Math.max(10, w * 0.11);
	}

	_draw() {
		const c		= this.canvas;
		const dpr	= window.devicePixelRatio || 1;
		const w		= c.clientWidth  || 240;
		const h		= c.clientHeight || 240;
		if (c.width !== Math.round(w * dpr) || c.height !== Math.round(h * dpr)) {
			c.width  = Math.round(w * dpr);
			c.height = Math.round(h * dpr);
		}
		const ctx = this.ctx;
		ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
		ctx.clearRect(0, 0, w, h);

		const pad	 = 4;
		const radius = 18;
		ctx.fillStyle	= '#2a2a2a';
		ctx.strokeStyle	= '#000';
		ctx.lineWidth	= 3;
		this._roundRect(ctx, pad + 1.5, pad + 1.5, w - (pad + 1.5) * 2, h - (pad + 1.5) * 2, radius);
		ctx.fill(); ctx.stroke();

		ctx.strokeStyle	= '#1d1d1d';
		ctx.lineWidth	= 1;
		ctx.beginPath();
		ctx.moveTo(w / 2, pad + 10); ctx.lineTo(w / 2, h - pad - 10);
		ctx.moveTo(pad + 10, h / 2); ctx.lineTo(w - pad - 10, h / 2);
		ctx.stroke();

		const cx	= w / 2, cy = h / 2;
		const r		= Math.min(cx, cy) - this._knobR();
		const kx	= cx + this.x * r;
		const ky	= cy - this.y * r;
		const kr	= this._knobR();
		ctx.fillStyle	= '#a8a8a8';
		ctx.strokeStyle	= '#000';
		ctx.lineWidth	= 2;
		ctx.beginPath();
		ctx.arc(kx, ky, kr, 0, Math.PI * 2);
		ctx.fill(); ctx.stroke();
	}

	_roundRect(ctx, x, y, w, h, r) {
		ctx.beginPath();
		ctx.moveTo(x + r, y);
		ctx.lineTo(x + w - r, y);
		ctx.quadraticCurveTo(x + w, y, x + w, y + r);
		ctx.lineTo(x + w, y + h - r);
		ctx.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
		ctx.lineTo(x + r, y + h);
		ctx.quadraticCurveTo(x, y + h, x, y + h - r);
		ctx.lineTo(x, y + r);
		ctx.quadraticCurveTo(x, y, x + r, y);
		ctx.closePath();
	}
}
