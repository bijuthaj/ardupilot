#include "Rover.h"

#define AR_CIRCLE_ACCEL_DEFAULT         1.0 // default acceleration in m/s/s if not specified by user
#define AR_CIRCLE_RADIUS_MIN            0.5 // minimum radius in meters
#define AR_CIRCLE_REACHED_EDGE_DIST     0.2 // vehicle has reached edge if within 0.2m

const AP_Param::GroupInfo ModeAutoTune::var_info[] = {

  // @Param: RTUN_ENABLE
  // @DisplayName: Rover Quicktune enable
  // @Description: Enable quicktune system
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
    AP_GROUPINFO("_ENABLE", 1, ModeAutoTune, enable, 1),   

  // @Param: RTUN_AXES
  // @DisplayName: Rover Quicktune axes
  // @Description: axes to tune
  // @Bitmask: 0:Steering,1:Speed
  // @User: Standard
AP_GROUPINFO("_AXES", 2, ModeAutoTune, axes, 3), 


  // @Param: RTUN_STR_FFRATIO
  // @DisplayName: Rover Quicktune Steering Rate FeedForward ratio
  // @Description: Ratio between measured response and FF gain. Raise this to get a higher FF gain
  // @Range: 0 1.0
  // @User: Standard
    AP_GROUPINFO("_STR_FFRATIO", 3, ModeAutoTune, strFFRatio, 0.9), 

  // @Param: RTUN_STR_P_RATIO
  // @DisplayName: Rover Quicktune Steering FF to P ratio
  // @Description: Ratio between steering FF and P gains. Raise this to get a higher P gain, 0 to leave P unchanged
  // @Range: 0 2.0
  // @User: Standard
    AP_GROUPINFO("_STR_P_RATIO", 4, ModeAutoTune, strPRatio, 0.45), 

 
  // @Param: RTUN_STR_I_RATIO
  // @DisplayName: Rover Quicktune Steering FF to I ratio
  // @Description: Ratio between steering FF and I gains. Raise this to get a higher I gain, 0 to leave I unchanged
  // @Range: 0 2.0
  // @User: Standard
    AP_GROUPINFO("_STR_I_RATIO", 5, ModeAutoTune, strIRatio, 0.5), 


  // @Param: RTUN_SPD_FFRATIO
  // @DisplayName: Rover Quicktune Speed FeedForward (equivalent) ratio
  // @Description: Ratio between measured response and CRUISE_THROTTLE value. Raise this to get a higher CRUISE_THROTTLE value
  // @Range: 0 1.0
  // @User: Standard
    AP_GROUPINFO("_SPD_FFRATIO", 6, ModeAutoTune, spdFFRatio, 1.0), 

  // @Param: RTUN_SPD_P_RATIO
  // @DisplayName: Rover Quicktune Speed FF to P ratio
  // @Description: Ratio between speed FF and P gain. Raise this to get a higher P gain, 0 to leave P unchanged
  // @Range: 0 2.0
  // @User: Standard
    AP_GROUPINFO("_SPD_P_RATIO", 7, ModeAutoTune, spdPRatio, 1.0), 


  // @Param: RTUN_SPD_I_RATIO
  // @DisplayName: Rover Quicktune Speed FF to I ratio
  // @Description: Ratio between speed FF and I gain. Raise this to get a higher I gain, 0 to leave I unchanged
  // @Range: 0 2.0
  // @User: Standard
    AP_GROUPINFO("_SPD_I_RATIO", 8, ModeAutoTune, spdIRatio, 1.0), 


  // @Param: RTUN_AUTO_FILTER
  // @DisplayName: Rover Quicktune auto filter enable
  // @Description: When enabled the PID filter settings are automatically set based on INS_GYRO_FILTER
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
    AP_GROUPINFO("_AUTO_FILTER", 9, ModeAutoTune, autoFilter, 1),

  // @Param: RTUN_AUTO_SAVE
  // @DisplayName: Rover Quicktune auto save
  // @Description: Number of seconds after completion of tune to auto-save. This is useful when using a 2 position switch for quicktune
  // @Units: s
  // @User: Standard
    AP_GROUPINFO("_AUTO_FILTER", 10, ModeAutoTune, autoFilter, 5),

    AP_GROUPEND
};

ModeAutoTune::ModeAutoTune() : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeAutoTune::_enter() {
  return true;
}

void ModeAutoTune::update()
{
    printf("in ModeAutoTune:strFFRatio at %f\n", strFFRatio.get());
   // Exit immediately if not enabled
    if (enable <= 0) {
        return;
    }

    if(have_pilot_input()) {
      last_pilot_input = AP_HAL::millis();
    }

    float steering_out, throttle_out;
    rover.get_steering_and_throttle(steering_out, throttle_out);

    // Check switch position (0: low, 1: middle, 2: high)
    if (sw_pos == 1 && (!rover.arming.is_armed() || throttle_out <= 0) && AP_HAL::millis() > last_warning + 5) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "RTun: must be armed and moving to tune");
        last_warning = AP_HAL::millis();
        return;
    }
    if (sw_pos == 0 || !rover.arming.is_armed()) {
        // Abort and revert parameters
        if (need_restore) {
            need_restore = false;
            //restore_all_params();
            //restore_gcs_pid_mask();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "RTun: gains reverted");
        }
        //reset_axes_done();
        return;
    }

}

//check for pilot input to pause tune
bool ModeAutoTune::have_pilot_input() {
  if(fabsf(channel_roll->norm_input_dz()) > 0 || fabsf(channel_throttle->norm_input_dz()) > 0 ) {
      return true;
    }
    return false;
}