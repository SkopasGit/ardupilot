#include "mode.h"
#include "Plane.h"

bool ModeFBWB::_enter()
{
#if HAL_SOARING_ENABLED
    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();
#endif
    
    if (plane.previous_mode == &plane.mode_qtakeoff){
        plane.target_altitude.amsl_cm=plane.mode_takeoff.get_target_dist()*100+plane.ahrs.get_home().alt;
    }
    else{
    plane.set_target_altitude_current();
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Target Alt AMSL: %.1d m",plane.target_altitude.amsl_cm);
    return true;
}

void ModeFBWB::update()
{
    // Thanks to Yury MonZon for the altitude limit code!

    if (!gps_disabled)
    {
        // Перевіряємо, чи ми прийшли з Q_TAKEOFF
        if (plane.previous_mode == &plane.mode_qtakeoff)
        {
            // перевіряємо чи виконався transition
            if (!quadplane.in_transition() && plane.get_mode() == Mode::FLY_BY_WIRE_B)
            {
                if (plane.ahrs.EKF3.healthy() && 
                    plane.ahrs.healthy() &&
                    AP::gps().status() != AP_GPS::NO_FIX)
                {
                    // EKF стабільний
                    if (ekf_stable_start_ms == 0)
                    {
                        ekf_stable_start_ms = AP_HAL::millis();
                    }
                    else if ((AP_HAL::millis() - ekf_stable_start_ms) > 5000)
                    {
                        gcs().send_text(MAV_SEVERITY_INFO, "FBWB: EKF3 stable, disabling GPS");
                    
                        AP::gps().force_disable(true);
                        gps_disabled = true; // щоб знову не виконувалося
                    }
                }
                else
                {
                    // Якщо EKF нестабільний, обнуляємо таймер
                    ekf_stable_start_ms = 0;
                }
            }
        }
    }

    plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_load_factor();
    plane.update_fbwb_speed_height();
}
