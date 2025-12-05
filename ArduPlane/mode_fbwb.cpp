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
   // FBWB_init = false; // Ініціалізація FBWB
}

void ModeFBWB::update()
{
    if (FBWB_init == false)
    {
    // Перевірка переходу з Q_TAKEOFF
        if (plane.previous_mode == &plane.mode_qtakeoff)
        {
            if (!quadplane.in_transition() && plane.get_mode() == Mode::FLY_BY_WIRE_B)
            {
                // Отримуємо статуси
                bool ekf3_healthy = plane.ahrs.EKF3.healthy();
                bool ahrs_healthy = plane.ahrs.healthy();
                bool has_inertial_nav = plane.ahrs.have_inertial_nav();
                Location loc;
                bool has_location = plane.ahrs.get_location(loc);
                Vector3f vel;
                bool has_velocity = plane.ahrs.get_velocity_NED(vel);
                bool not_vibration_affected = !plane.ahrs.is_vibration_affected();
                bool gps_ok = (AP::gps().status() != AP_GPS::NO_FIX);

                if (ekf3_healthy && ahrs_healthy &&
                    has_inertial_nav && has_location && has_velocity &&
                    not_vibration_affected && gps_ok)
                {
                    // EKF стабільний
                    if (ekf_stable_start_ms == 0)
                    {
                        ekf_stable_start_ms = AP_HAL::millis();
                    }
                    else if ((AP_HAL::millis() - ekf_stable_start_ms) > 5000)
                    {
                        gcs().send_text(MAV_SEVERITY_INFO, "FBWB: EKF stable, disabling GPS");
                        FBWB_init= true; // Ініціалізація FBWB завершена
                        //AP::gps().force_disable(true);
                        
                    }
                }
                else
                {
                    ekf_stable_start_ms = 0;
   
                }
            }
        }
    
    }   
    // Управління каналами
    plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_load_factor();
    plane.update_fbwb_speed_height();
}
