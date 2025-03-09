#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>
#include "quadplane.h"
//#include <AP_Motors/AP_Motors.h>



#if HAL_QUADPLANE_ENABLED

// Опис параметрів для Q_TAKEOFF залишив параметри для літакового takeoff
//ALT висота на якой буде переход на fbwb
//LVL_ALT швидкість набору висоти (ще плануеться)





// Вхід у режим Q_TAKEOFF
bool ModeQTakeOff::_enter()
{
    
    target_alt_with_sea = plane.mode_takeoff.target_alt.get();
   
    gcs().send_text(MAV_SEVERITY_INFO, "Q_TAKEOFF: ALT:  %hd%%", target_alt_with_sea);
    
    // Встановлюємо режим QHOVER для вертикального зльоту
  if (!(plane.current_loc.initialised() && AP::ahrs().home_is_set() && target_alt_with_sea > 50)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Q_TAKEOFF: No GPS lock! or Target ALT is low");
        return false ;
    }
  //  quadplane.throttle_wait = false;
   // SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.g2);
  // SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, quadplane.motors->get_throttle_hover());
   plane.mode_qhover._enter();
    level_alt_cm=plane.mode_takeoff.level_alt.get()* 100;
    //target_alt_with_sea=qtarget_alt.get()*100;
    //  float hover_throttle = quadplane.motors->get_throttle_hover();
    // attitude_control->set_throttle_out(hover_throttle, true, 0);
        return true;
}

// Оновлення режиму Q_TAKEOFF
void ModeQTakeOff::update()
{
   
    plane.mode_qstabilize.update();
  
}

// Основна логіка роботи Q_TAKEOFF
void ModeQTakeOff::run()
{
    // Отримуємо поточну висоту
    
       if (plane.barometer.get_altitude()< target_alt_with_sea) {
     //  if (plane.current_loc.alt< 1){  
        quadplane.pos_control->set_pos_target_z_from_climb_rate_cm(level_alt_cm);
      //(plane.current_loc.alt< 1);
        

        plane.mode_qhover.run();
    }
    else {
        // Коли висота досягнута - переходимо в FBWB
        gcs().send_text(MAV_SEVERITY_INFO, "Q_TAKEOFF: Altitude reached, switching to FBWB");
       // plane.mode_fbwb.run();
        plane.set_mode(Mode::FLY_BY_WIRE_B, ModeReason::MISSION_CMD);
     

}
}

#endif
