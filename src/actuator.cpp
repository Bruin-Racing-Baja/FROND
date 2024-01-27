#include <actuator.h>
#include <constants.h>

/**
 * Constructor for the actuator
 * @param odrive Pointer to odrive object
 * @param axis_num Axis number of actuator motor
 */
Actuator::Actuator(ODrive* odrive, int axis_num, float velocity_lim) :odrive(odrive), axis_num(axis_num), velocity_lim(velocity_lim){}

/**
 * Initializes connection to physical odrive
 * @return true if successful
 */
bool Actuator::init() {
  // TODO: Why do we have this ?
  // Due to CAN interrupt handler weirdness
  commanded_axis_state = odrive->get_axis_state(axis_num);
  return 1;
}

/**
 * Instructs odrive to attempt encoder homing
 * @return true if successful
 */
bool Actuator::encoder_index_search() {
  int state =
      odrive->set_state(axis_num, ODRIVE_ENCODER_INDEX_SEARCH_STATE);
  commanded_axis_state = ODRIVE_ENCODER_INDEX_SEARCH_STATE;
  delayMicroseconds(5e6);
  return state == 0;
}

/**
 * If the targeted actuator velocity is different than the current velocity, set it to the updated velocity
 * @param velocity The velocity to update to
 * @return The current set velocity of the actuator
 */
float Actuator::update_velocity(float velocity, float brake_offset) {
  if (commanded_axis_state == ODRIVE_VELOCITY_CONTROL_STATE &&
      velocity == cur_velocity) {
    return velocity;
  }
  return Actuator::set_velocity(velocity, brake_offset);
}

/**
 * Instructs the odrive object to set given velocity
 * @param velocity The velocity to set
 * @return The velocity that is actually set
 */
float Actuator::set_velocity(float velocity, float brake_offset) {
  // TODO: Return on error (and reset variables). Properly implement brake offset.
  if(odrive->set_state(axis_num, ODRIVE_VELOCITY_CONTROL_STATE) != 0){
    Serial.printf("Error setting ODrive to velocity control state (CAN Error)\n");
  }
  commanded_axis_state = ODRIVE_VELOCITY_CONTROL_STATE;

  float target_velocity = constrain(velocity, -velocity_lim, velocity_lim);
  target_velocity += brake_offset;
  if (odrive->set_input_vel(axis_num, target_velocity, 0) != 0) {
    Serial.printf("Error setting ODrive velocity (CAN Error)\n");
  }

  cur_velocity = target_velocity;
  return cur_velocity;
}