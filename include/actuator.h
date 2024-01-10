#ifndef actuator_h
#define actuator_h

#include <Arduino.h>
#include <constants.h>
#include <odrive.h>

class Actuator {
 public:
  Actuator(ODrive* odrive);
  bool init();
  bool encoder_index_search();
  bool actuator_homing();
  float update_speed(float target_speed, float brake_offset);

  // Getters
  int get_readout(float readout[5]);

 private:
  int actuator_error = 0;
  int homing_error = 0;
  int homing_timer = 0;
  float current_speed = 0.0;
  int commanded_axis_state = -1;
  float commanded_axis_velocity = 0.0;
  // Constants
  int axis_number = ACTUATOR_AXIS;
  // Functions
  float set_speed(float set_speed, float brake_offset);

  ODrive* odrive;
};

#endif