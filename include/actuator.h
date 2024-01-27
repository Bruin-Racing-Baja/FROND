#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <Arduino.h>
#include <constants.h>
#include <odrive.h>

class Actuator {
 public:
  Actuator(ODrive* odrive, int axis_num, float velocity_lim);
  bool init();
  bool encoder_index_search();
  float update_velocity(float velocity, float brake_offset);

 private:
  ODrive* odrive;
  int axis_num;
  float velocity_lim;

  float cur_velocity = 0.0;
  int commanded_axis_state = -1;

  float set_velocity(float velocity, float brake_offset);

};

#endif