// Yaw controller — decoupled from the pitch (balance) loop.
//
// Pilot or nav sets a target yaw rate (rad/s). This loop produces the yaw
// torque that drives the bot's measured yaw rate to match. The plant
// applies this torque alongside whatever forward force the pitch
// controller produced.
//
// On a real two-wheel bot, yaw torque is realized by a differential PWM
// between left and right wheels. We model it directly as a torque here for
// simplicity; if you want the per-wheel detail, the differential is
// (yaw_torque / wheelbase) of force per side.

export class YawController {
	constructor() {
		this.target_yaw_rate = 0;
		this.lastTorque      = 0;
	}

	reset() { this.lastTorque = 0; }

	// gains: { Kyaw, MaxTauYaw }
	update(sensors, gains) {
		const { Kyaw, MaxTauYaw } = gains;
		let yaw_torque = Kyaw * (this.target_yaw_rate - sensors.yaw_rate);
		if (yaw_torque >  MaxTauYaw) yaw_torque =  MaxTauYaw;
		if (yaw_torque < -MaxTauYaw) yaw_torque = -MaxTauYaw;
		this.lastTorque = yaw_torque;
		return yaw_torque;
	}
}
