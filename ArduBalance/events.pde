// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
 */
static void failsafe_radio_on_event()
{
}

// failsafe_off_event - respond to radio contact being regained
// we must be in AUTO, LAND or RTL modes
// or Stabilize or ACRO mode but with motors disarmed
static void failsafe_radio_off_event()
{}

static void low_battery_event(void)
{}

// failsafe_gps_check - check for gps failsafe
static void failsafe_gps_check()
{}

// failsafe_gps_off_event - actions to take when GPS contact is restored
static void failsafe_gps_off_event(void)
{}

static void update_events()     // Used for MAV_CMD_DO_REPEAT_SERVO and MAV_CMD_DO_REPEAT_RELAY
{
    if(event_repeat == 0 || (millis() - event_timer) < event_delay)
        return;

    if(event_repeat != 0) {             // event_repeat = -1 means repeat forever
        event_timer = millis();

        if (event_id >= CH_5 && event_id <= CH_8) {
            if(event_repeat%2) {
                hal.rcout->write(event_id, event_value);                 // send to Servos
            } else {
                hal.rcout->write(event_id, event_undo_value);
            }
        }

        if  (event_id == RELAY_TOGGLE) {
            relay.toggle();
        }
        if (event_repeat > 0) {
            event_repeat--;
        }
    }
}

