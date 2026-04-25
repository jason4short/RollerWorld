// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

extern RC_Channel* rc_ch[8];

static void default_dead_zones()
{
    g.rc_1.set_dead_zone(60);
    g.rc_2.set_dead_zone(60);
    g.rc_3.set_dead_zone(60);
    g.rc_4.set_dead_zone(80);
    g.rc_6.set_dead_zone(0);
}

static void init_rc_in()
{
	// set rc channel ranges
	g.rc_1.set_angle(MAX_INPUT_YAW_ANGLE);
	g.rc_2.set_angle(MAX_INPUT_PITCH_ANGLE);
	g.rc_3.set_range(0,1000);
	g.rc_4.set_angle(4500);

	g.rc_1.set_type(RC_CHANNEL_ANGLE_RAW);
	g.rc_2.set_type(RC_CHANNEL_ANGLE_RAW);
	g.rc_4.set_type(RC_CHANNEL_ANGLE_RAW);

	//set auxiliary ranges
	g.rc_5.set_range(0,1000);
	g.rc_6.set_angle(300);
	g.rc_7.set_range(0,1000);
	g.rc_8.set_range(0,1000);

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8, &g.rc_9, &g.rc_10, &g.rc_11, &g.rc_12);
#elif MOUNT == ENABLED
    update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8, &g.rc_10, &g.rc_11);
#endif
}

static void init_rc_out()
{
	APM_RC.Init( &isr_registry );		// APM Radio initialization
	init_motors_out();
	motors_output_enable();
}

//#define FAILSAFE_RADIO_TIMEOUT_MS 2000       // 2 seconds
static void read_radio()
{
    if (hal.rcin->valid() > 0) {
        last_update = millis();
		ap_system.new_radio_frame = true;
        uint16_t periods[8];
        hal.rcin->read(periods,8);
        g.rc_1.set_pwm(periods[0]);
        g.rc_2.set_pwm(periods[1]);
        g.rc_3.set_pwm(periods[2]);
        g.rc_4.set_pwm(periods[3]);
        g.rc_5.set_pwm(periods[4]);
        g.rc_6.set_pwm(periods[5]);
        g.rc_7.set_pwm(periods[6]);
        g.rc_8.set_pwm(periods[7]);
    }
}

static void trim_radio()
{
    for (uint8_t i = 0; i < 30; i++) {
        read_radio();
    }

    g.rc_1.trim();      // roll
    g.rc_2.trim();      // pitch
    g.rc_4.trim();      // yaw

    g.rc_1.save_eeprom();
    g.rc_2.save_eeprom();
    g.rc_4.save_eeprom();
}

