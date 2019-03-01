#include "Copter.h"

bool Copter::ModePlanckTracking::init(bool ignore_checks){
    //If the copter is currently in-flight, entry into this mode indicates
    //that the user would like to return to the tag tracking, so run RTB
    if(!copter.ap.land_complete)
    {
        copter.planck_interface.request_rtb(
          (float)copter.g.rtl_altitude/100.,
          copter.pos_control->get_speed_up()/100.,
          copter.pos_control->get_speed_down()/100.);
    }
    
    //Otherwise do nothing, as we are on the ground waiting for a takeoff
    //command
    
    return Copter::ModeGuided::init(ignore_checks);
}

void Copter::ModePlanckTracking::run() {
  
    //If there is new command data, send it to Guided
    if(copter.planck_interface.new_command_available()) {
        switch(copter.planck_interface.get_cmd_type()) {
          //Set guided mode attitude/velocity commands
          case copter.planck_interface.ACCEL:
          {
              Vector3f accel_cmss;
              float yaw_cd;
              float vz_cms;
              bool is_yaw_rate;
              
              bool good_cmd = copter.planck_interface.get_accel_yaw_zrate_cmd(
                  accel_cmss, yaw_cd, vz_cms, is_yaw_rate
              );
              
              if(!good_cmd) {
                  accel_cmss.x = accel_cmss.y = yaw_cd = vz_cms = 0;
                  is_yaw_rate = true;
              }
              
              //Turn accel into lean angles
              float roll_cd, pitch_cd;
              copter.pos_control->accel_to_lean_angles(
                accel_cmss.x,
                accel_cmss.y,
                roll_cd,
                pitch_cd);

              //Convert this to quaternions, yaw rates
              Quaternion q;
              q.from_euler(ToRad(roll_cd/100.), ToRad(pitch_cd/100.), ToRad(yaw_cd/100.));
              float yaw_rate_rads = ToRad(yaw_cd / 100.);
          
              //Update the GUIDED mode controller
              Copter::ModeGuided::set_angle(q,vz_cms,is_yaw_rate,yaw_rate_rads);
              break;
          }

          case copter.planck_interface.ATTITUDE:
          {
              Vector3f att_cd;
              float vz_cms;
              bool is_yaw_rate;
              
              bool good_cmd = copter.planck_interface.get_attitude_zrate_cmd(
                  att_cd, vz_cms, is_yaw_rate
              );
              
              if(!good_cmd) {
                  att_cd.x = att_cd.y = att_cd.z = vz_cms = 0;
                  is_yaw_rate = true;
              }

              //Convert this to quaternions, yaw rates
              Quaternion q;
              q.from_euler(ToRad(att_cd.x/100.), ToRad(att_cd.y/100.), ToRad(att_cd.z/100.));
              float yaw_rate_rads = ToRad(att_cd.z / 100.);
          
              //Update the GUIDED mode controller
              Copter::ModeGuided::set_angle(q,vz_cms,is_yaw_rate,yaw_rate_rads);
              break;
          }

          case copter.planck_interface.VELOCITY:
          {
              Vector3f vel_cmd;
              float yaw_cmd_cd;
              bool yaw_rate = true;
              
              bool good_cmd = copter.planck_interface.get_velocity_yaw_cmd(vel_cmd, yaw_cmd_cd);
              
              if(!good_cmd) {
                  vel_cmd.x = vel_cmd.y = vel_cmd.z = 0;
                  yaw_cmd_cd = 0;
                  yaw_rate = true;
              }
              
              Copter::ModeGuided::set_velocity(vel_cmd, yaw_rate, yaw_cmd_cd);
              break;
          }

          case copter.planck_interface.POSITION:
          {
              Location loc_cmd;
              copter.planck_interface.get_position_cmd(loc_cmd);
              Copter::ModeGuided::set_destination(loc_cmd);
              break;
          }

          case copter.planck_interface.POSVEL:
          {
              Location loc_cmd;
              Vector3f vel_cmd;
              float yaw_cmd_cd;
              bool is_yaw_rate = true;
              
              bool good_cmd = copter.planck_interface.get_posvel_cmd(
                loc_cmd,
                vel_cmd,
                yaw_cmd_cd,
                is_yaw_rate);
            
              //Set a zero velocity if this is a bad command
              if(!good_cmd) {
                  vel_cmd.x = vel_cmd.y = vel_cmd.z = 0;
                  yaw_cmd_cd = 0;
                  Copter::ModeGuided::set_velocity(vel_cmd, yaw_cmd_cd, yaw_cmd_cd);
              } else {
                Copter::ModeGuided::set_destination_posvel(
                  copter.pv_location_to_vector(loc_cmd),
                  vel_cmd,
                  !is_yaw_rate,
                  yaw_cmd_cd,
                  is_yaw_rate,
                  yaw_cmd_cd);
              }
              break;
          }
  
          default:
            break;
      }
    }
    
    //Run the guided mode controller
    Copter::ModeGuided::run();
}

bool Copter::ModePlanckTracking::do_user_takeoff_start(float final_alt_above_home)
{
    // Check if planck is ready
    if(!copter.planck_interface.ready_for_takeoff())
      return false;

    // Tell planck to start commanding
    copter.planck_interface.request_takeoff(final_alt_above_home/100.);

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();

    return true;
}
