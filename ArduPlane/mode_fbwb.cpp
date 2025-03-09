#include "mode.h"
#include "Plane.h"

bool ModeFBWB::_enter()
{
#if HAL_SOARING_ENABLED
    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();
#endif

    plane.set_target_altitude_current();

    return true;
    
}

void ModeFBWB::update()
{
    // Thanks to Yury MonZon for the altitude limit code!
  
    // Перевіряємо, чи ми прийшли з Q_TAKEOFF
        if (plane.previous_mode == &plane.mode_qtakeoff) {
      //  gcs().send_text(MAV_SEVERITY_INFO, "FBWB: Entered from Q_TAKEOFF");
       // Отримуємо активний EKF
    uint8_t ekf_index = plane.ahrs.get_ekf_type();

    // Переконуємося, що EKF3 активний і стабільний
    if (ekf_index == 3 && plane.ahrs.healthy() && millis() - plane.started_flying_ms > 5000 && AP::gps().status()!=AP_GPS::NO_FIX)  {
        gcs().send_text(MAV_SEVERITY_INFO, "FBWB: EKF3 stable, disabling GPS");
        AP::gps().force_disable(true);
    
    }
   // plane.target_altitude.amsl_cm=2500;
}


    plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_load_factor();
    plane.update_fbwb_speed_height();

}

