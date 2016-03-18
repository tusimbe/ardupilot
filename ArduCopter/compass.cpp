
#include "Copter.h"

static int8_t m_last_switch_position = -1;
static uint32_t m_last_edge_time_ms;
static uint8_t m_compass_switch_cnt = 0;

void Copter::check_switchs_compass_cal(){

    bool is_failsafe = failsafe.radio || failsafe.radio_counter != 0;

    // to fail fast
    if (hal.util->get_soft_armed() || is_failsafe) {
        return;
    }

    uint32_t tnow_ms = millis();

    uint8_t pos = read_3pos_switch(g.rc_5.radio_in);
    
    if (m_last_switch_position == -1) { // wait to start check
    
        if (pos == AUX_SWITCH_LOW){
            m_last_switch_position = pos;
            m_last_edge_time_ms = tnow_ms;
        }

        return;
    }


    if (m_last_switch_position == AUX_SWITCH_LOW) { // wait for HIGH
        if (pos == AUX_SWITCH_HIGH) {
            if (tnow_ms - m_last_edge_time_ms < 200) {
                m_last_switch_position = pos;
                m_last_edge_time_ms = tnow_ms;
            } else {
                m_last_switch_position = -1; // reset to wait start
                m_compass_switch_cnt = 0;
            }
        }

        return;
    }

    if (m_last_switch_position == AUX_SWITCH_HIGH) { // wait for LOW
        if (pos == AUX_SWITCH_LOW) {
            if (tnow_ms - m_last_edge_time_ms < 200) {
                m_last_switch_position = pos;
                m_last_edge_time_ms = tnow_ms;
                
                m_compass_switch_cnt++;
            } else {
                m_last_switch_position = -1; // reset to wait start
                m_compass_switch_cnt = 0;
            }

            if (m_compass_switch_cnt >= 3) {

                if (!compass.start_calibration_all(false, true, 0.2, false)) {
                    AP_Notify::flags.compass_cal_failed = 1;
                }
                    
                m_last_switch_position = -1; // reset to wait start
                m_compass_switch_cnt = 0;
            }
        }

         return;
    }
}

