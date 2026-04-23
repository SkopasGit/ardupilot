#include "mode.h"
#include "Tracker.h"

static constexpr float DT = 0.02f; // 50 Hz

// швидкість зміни кута (centideg/sec)
static constexpr float YAW_RATE = 6000.0f;   // 60 deg/sec
static constexpr float PITCH_RATE = 3000.0f; // 30 deg/sec

static constexpr float STICK_DEADBAND = 0.05f;

void ModeAssist::init_targets_from_current_attitude()
{
    const AP_AHRS &ahrs = AP::ahrs();

    _target_yaw_cd = wrap_180_cd(ahrs.yaw_sensor);
    _target_pitch_cd = constrain_float(ahrs.pitch_sensor,
                                       tracker.g.pitch_min * 100,
                                       tracker.g.pitch_max * 100);

    _target_initialized = true;
}

void ModeAssist::update()
{
    if (!_target_initialized) {
        init_targets_from_current_attitude();
    }

    const RC_Channel *yaw_ch = RC_Channels::rc_channel(CH_YAW);
    const RC_Channel *pitch_ch = RC_Channels::rc_channel(CH_PITCH);

    float yaw_in = 0.0f;
    float pitch_in = 0.0f;

    if (yaw_ch != nullptr) {
        yaw_in = yaw_ch->norm_input_dz(); // -1..1
    }

    if (pitch_ch != nullptr) {
        pitch_in = pitch_ch->norm_input_dz();
    }

    // інтегруємо input → цільовий кут
    if (fabsf(yaw_in) > STICK_DEADBAND) {
        _target_yaw_cd = wrap_180_cd(_target_yaw_cd + yaw_in * YAW_RATE * DT);
    }

    if (fabsf(pitch_in) > STICK_DEADBAND) {
        _target_pitch_cd += pitch_in * PITCH_RATE * DT;

        _target_pitch_cd = constrain_float(_target_pitch_cd,
                                           tracker.g.pitch_min * 100,
                                           tracker.g.pitch_max * 100);
    }

    // ===== ГОЛОВНЕ =====
    // підставляємо в nav_status як в AUTO

    tracker.nav_status.bearing = _target_yaw_cd * 0.01f;
    tracker.nav_status.pitch   = _target_pitch_cd * 0.01f;

    // і використовуємо штатний AUTO пайплайн
    update_auto();
}