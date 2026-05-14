// Wheels — chassis (force_fwd, torque_yaw) → per-wheel torque.
//
// Just the differential mix:
//
//   torque_left  = (force_fwd − torque_yaw / wheelbase) / 2
//   torque_right = (force_fwd + torque_yaw / wheelbase) / 2
//
// Pure forward command sends equal torque to both wheels; pure yaw
// torque sends opposite. The motor module owns the PWM/deadband/back-EMF
// math now — see physics/motor.js applyTorque().
//
// Why this layer still exists: cascade's pedagogy is "four explicit
// layers, each with its own rate." Wheels keeping its slot keeps the
// rate-hierarchy diagram intact even though the math is trivial. The
// per-wheel inverse-model PI lives in Motor where any controller can
// share it.

export class Wheels {
	constructor() {
		this.lastTorqueLeft  = 0;
		this.lastTorqueRight = 0;
	}

	reset() {
		this.lastTorqueLeft  = 0;
		this.lastTorqueRight = 0;
	}

	// attOut:  { force_fwd, torque_yaw }
	// gains:   { wheelbase }
	update(attOut, sensors, gains /*, dt, motor */) {
		const wb = gains.wheelbase;
		const torque_left  = (attOut.force_fwd - attOut.torque_yaw / wb) / 2;
		const torque_right = (attOut.force_fwd + attOut.torque_yaw / wb) / 2;
		this.lastTorqueLeft  = torque_left;
		this.lastTorqueRight = torque_right;
		return { torque_left, torque_right };
	}
}
