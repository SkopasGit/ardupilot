#include "Tracker.h"

/*
 * Code to move pitch and yaw servos to attain a target heading or pitch
 */

// init_servos - initialises the servos
void Tracker::init_servos()
{
    // update assigned functions and enable auxiliary servos
    AP::srv().enable_aux_servos();

    SRV_Channels::set_default_function(CH_YAW, SRV_Channel::k_tracker_yaw);
    SRV_Channels::set_default_function(CH_PITCH, SRV_Channel::k_tracker_pitch);

    // yaw range is +/- (YAW_RANGE parameter/2) converted to centi-degrees
    SRV_Channels::set_angle(SRV_Channel::k_tracker_yaw, g.yaw_range * 100/2);

    // pitch range is +/- (PITCH_MIN/MAX parameters/2) converted to centi-degrees
    SRV_Channels::set_angle(SRV_Channel::k_tracker_pitch, (-g.pitch_min+g.pitch_max) * 100/2);

    SRV_Channels::calc_pwm();
    SRV_Channels::output_ch_all();

    yaw_servo_out_filt.set_cutoff_frequency(SERVO_OUT_FILT_HZ);
    pitch_servo_out_filt.set_cutoff_frequency(SERVO_OUT_FILT_HZ);
}

/**
   update the pitch (elevation) servo. The aim is to drive the boards ahrs pitch to the
   requested pitch, so the board (and therefore the antenna) will be pointing at the target
 */
void Tracker::update_pitch_servo(float pitch)
{
    switch ((enum ServoType)g.servo_pitch_type.get()) {
    case SERVO_TYPE_ONOFF:
        update_pitch_onoff_servo(pitch);
        break;

    case SERVO_TYPE_CR:
        update_pitch_cr_servo(pitch);
        break;

    case SERVO_TYPE_POSITION:
    default:
        update_pitch_position_servo();
        break;
    }
}

/**
   update the pitch (elevation) servo. The aim is to drive the boards ahrs pitch to the
   requested pitch, so the board (and therefore the antenna) will be pointing at the target
 */
void Tracker::update_pitch_position_servo()
{
    int32_t pitch_min_cd = g.pitch_min*100;
    int32_t pitch_max_cd = g.pitch_max*100;
    // Need to configure your servo so that increasing servo_out causes increase in pitch/elevation (ie pointing higher into the sky,
    // above the horizon. On my antenna tracker this requires the pitch/elevation servo to be reversed
    // param set RC2_REV -1
    //
    // The pitch servo (RC channel 2) is configured for servo_out of -9000-0-9000 servo_out,
    // which will drive the servo from RC2_MIN to RC2_MAX usec pulse width.
    // Therefore, you must set RC2_MIN and RC2_MAX so that your servo drives the antenna altitude between -90 to 90 exactly
    // To drive my HS-645MG servos through their full 180 degrees of rotational range, I have to set:
    // param set RC2_MAX 2540
    // param set RC2_MIN 640
    //
    // You will also need to tune the pitch PID to suit your antenna and servos. I use:
    // PITCH2SRV_P      0.100000
    // PITCH2SRV_I      0.020000
    // PITCH2SRV_D      0.000000
    // PITCH2SRV_IMAX   4000.000000

    // calculate new servo position
    float new_servo_out = SRV_Channels::get_output_scaled(SRV_Channel::k_tracker_pitch) + g.pidPitch2Srv.update_error(nav_status.angle_error_pitch, G_Dt);

    // position limit pitch servo
    if (new_servo_out <= pitch_min_cd) {
        new_servo_out = pitch_min_cd;
        g.pidPitch2Srv.reset_I();
    }
    if (new_servo_out >= pitch_max_cd) {
        new_servo_out = pitch_max_cd;
        g.pidPitch2Srv.reset_I();
    }
    // rate limit pitch servo
    SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_pitch, new_servo_out);

    if (pitch_servo_out_filt_init) {
        pitch_servo_out_filt.apply(new_servo_out, G_Dt);
    } else {
        pitch_servo_out_filt.reset(new_servo_out);
        pitch_servo_out_filt_init = true;
    }
}


/**
   update the pitch (elevation) servo. The aim is to drive the boards ahrs pitch to the
   requested pitch, so the board (and therefore the antenna) will be pointing at the target
 */
void Tracker::update_pitch_onoff_servo(float pitch) const
{
    int32_t pitch_min_cd = g.pitch_min*100;
    int32_t pitch_max_cd = g.pitch_max*100;

    float acceptable_error = g.onoff_pitch_rate * g.onoff_pitch_mintime;
    if (fabsf(nav_status.angle_error_pitch) < acceptable_error) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_pitch, 0);
    } else if ((nav_status.angle_error_pitch > 0) && (pitch*100>pitch_min_cd)) {
        // positive error means we are pointing too low, so push the
        // servo up
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_pitch, -9000);
    } else if (pitch*100<pitch_max_cd) {
        // negative error means we are pointing too high, so push the
        // servo down
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_pitch, 9000);
    }
}

/**
   update the pitch for continuous rotation servo
*/
void Tracker::update_pitch_cr_servo(float pitch)
{
    const float pitch_out = constrain_float(g.pidPitch2Srv.update_error(nav_status.angle_error_pitch, G_Dt), -(-g.pitch_min+g.pitch_max) * 100/2, (-g.pitch_min+g.pitch_max) * 100/2);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_pitch, pitch_out);
}

/**
   update the yaw (azimuth) servo.
 */
void Tracker::update_yaw_servo(float yaw)
{
	switch ((enum ServoType)g.servo_yaw_type.get()) {
    case SERVO_TYPE_ONOFF:
        update_yaw_onoff_servo(yaw);
        break;

    case SERVO_TYPE_CR:
        update_yaw_cr_servo(yaw);
        break;

    case SERVO_TYPE_POSITION:
    default:
        update_yaw_position_servo();
        break;
    }
}

/**
   update the yaw (azimuth) servo. The aim is to drive the boards ahrs
   yaw to the requested yaw, so the board (and therefore the antenna)
   will be pointing at the target
 */
void Tracker::update_yaw_position_servo()
{
    const int32_t yaw_limit_cd = g.yaw_range * 100 / 2;

    const float current_servo_out =
        SRV_Channels::get_output_scaled(SRV_Channel::k_tracker_yaw);

    // ------------------------------------------------------------
    // 1. Якщо зараз у режимі реверсу - примусово женемо серву
    //    до ПРОТИЛЕЖНОГО краю
    // ------------------------------------------------------------
    if (this->yaw_reversing) {
    const float slew_time = MAX(g.yaw_slew_time, 0.1f);
    const float reverse_rate_cd_per_sec = (g.yaw_range * 100.0f) / slew_time;
    const float step_cd = reverse_rate_cd_per_sec * G_Dt;

    // цільовий край, куди треба прийти
    const float target_servo_out = (this->yaw_reverse_dir < 0) ? -yaw_limit_cd : yaw_limit_cd;

    float new_servo_out = current_servo_out;

    // рух до цілі без перелітання
    if (target_servo_out > current_servo_out) {
        new_servo_out = MIN(current_servo_out + step_cd, target_servo_out);
    } else if (target_servo_out < current_servo_out) {
        new_servo_out = MAX(current_servo_out - step_cd, target_servo_out);
    }

    new_servo_out = constrain_float(new_servo_out, -yaw_limit_cd, yaw_limit_cd);

    SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, new_servo_out);

    if (yaw_servo_out_filt_init) {
        yaw_servo_out_filt.apply(new_servo_out, G_Dt);
    } else {
        yaw_servo_out_filt.reset(new_servo_out);
        yaw_servo_out_filt_init = true;
    }

    const uint32_t reverse_time_ms =
        (uint32_t)(MAX(g.min_reverse_time, 0.0f) * 1000.0f);

    const bool time_ok =
        (AP_HAL::millis() - this->yaw_reverse_start_ms) >= reverse_time_ms;

    const bool reached_opposite_limit =
        is_equal(new_servo_out, target_servo_out);

    if (time_ok && reached_opposite_limit) {
        this->yaw_reversing = false;
        this->yaw_reverse_dir = 0;
        g.pidYaw2Srv.reset_I();
    }

    return;
}

    // ------------------------------------------------------------
    // 2. Звичайний PID-режим
    // ------------------------------------------------------------
    float err = nav_status.angle_error_yaw;

// DEAD BAND (в градусах → переводимо в centideg)
    const float deadband_cd = g.yaw_deadband * 100.0f;

    if (fabsf(err) < deadband_cd) {
       g.pidYaw2Srv.reset_I();   // обов'язково!
       return;
    }
    float servo_change = g.pidYaw2Srv.update_error(nav_status.angle_error_yaw, G_Dt);
    servo_change = constrain_float(servo_change, -18000, 18000);

    float new_servo_out = current_servo_out + servo_change;
    new_servo_out = constrain_float(new_servo_out, -18000, 18000);

    // ------------------------------------------------------------
    // 3. Якщо вперлись у правий ліміт і PID далі штовхає вправо,
    //    запускаємо повний обхід уліво
    // ------------------------------------------------------------
    if (new_servo_out >= yaw_limit_cd && nav_status.angle_error_yaw > 0) {
        this->yaw_reversing = true;
        this->yaw_reverse_dir = -1;
        this->yaw_reverse_start_ms = AP_HAL::millis();
        g.pidYaw2Srv.reset_I();
        return;
    }

    // ------------------------------------------------------------
    // 4. Якщо вперлись у лівий ліміт і PID далі штовхає вліво,
    //    запускаємо повний обхід управо
    // ------------------------------------------------------------
    if (new_servo_out <= -yaw_limit_cd && nav_status.angle_error_yaw < 0) {
        this->yaw_reversing = true;
        this->yaw_reverse_dir = +1;
        this->yaw_reverse_start_ms = AP_HAL::millis();
        g.pidYaw2Srv.reset_I();
        return;
    }

    // ------------------------------------------------------------
    // 5. Нормальний запис у вихід
    // ------------------------------------------------------------
    new_servo_out = constrain_float(new_servo_out, -yaw_limit_cd, yaw_limit_cd);

    SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, new_servo_out);

    if (yaw_servo_out_filt_init) {
        yaw_servo_out_filt.apply(new_servo_out, G_Dt);
    } else {
        yaw_servo_out_filt.reset(new_servo_out);
        yaw_servo_out_filt_init = true;
    }
}

/**
   update the yaw (azimuth) servo. The aim is to drive the boards ahrs
   yaw to the requested yaw, so the board (and therefore the antenna)
   will be pointing at the target
 */
void Tracker::update_yaw_onoff_servo(float yaw) const
{
    float acceptable_error = g.onoff_yaw_rate * g.onoff_yaw_mintime;
    if (fabsf(nav_status.angle_error_yaw * 0.01f) < acceptable_error) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, 0);
    } else if (nav_status.angle_error_yaw * 0.01f > 0) {
        // positive error means we are counter-clockwise of the target, so
        // move clockwise
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, 18000);
    } else {
        // negative error means we are clockwise of the target, so
        // move counter-clockwise
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, -18000);
    }
}

/**
   update the yaw continuous rotation servo
 */
void Tracker::update_yaw_cr_servo(float yaw)
{
    const float yaw_out = constrain_float(-g.pidYaw2Srv.update_error(nav_status.angle_error_yaw, G_Dt), -g.yaw_range * 100/2, g.yaw_range * 100/2);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, yaw_out);
}
