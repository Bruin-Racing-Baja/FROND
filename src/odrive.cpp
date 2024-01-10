#include <FlexCAN_T4.h>
#include <odrive.h>

FlexCAN_T4<FLEXCAN_BUS_NUM, FLEXCAN_RX_SIZE, FLEXCAN_TX_SIZE> flexcan_bus;

/**
 * @brief Initialize the CAN bus for communication with the ODrive
 * @param parse Pointer to function that parses received messages 
 */
bool ODrive::init(void (*parse)(const CAN_message_t& msg)) {
  flexcan_bus.begin();
  flexcan_bus.setBaudRate(250000);
  flexcan_bus.setMaxMB(16);
  flexcan_bus.enableFIFO();
  flexcan_bus.enableFIFOInterrupt();
  flexcan_bus.onReceive(parse);
  NVIC_SET_PRIORITY(IRQ_CAN2, 1);
  return true;
}

/**
 * @brief Send a command to a specific axis
 * @param axis ODrive axis number
 * @param cmd_id ODrive CAN command ID number
 * @param remote RTR bit for CAN message (typically 0 for setters and 1 for requesters)
 * @param buf 8-wide array of command data bytes
 * @return Status code of command send
 */
int ODrive::send_command(int axis, int cmd_id, bool remote, uint8_t buf[8]) {
  CAN_message_t msg;
  if (axis != 0 && axis != 1) {
    return ODRIVE_CMD_ERROR_INVALID_AXIS;
  }

  if (cmd_id < 0x1 || 0x1B < cmd_id) {
    return ODRIVE_CMD_ERROR_INVALID_COMMAND;
  }

  msg.id = (axis << 5) | cmd_id;
  msg.len = 8;
  memcpy(&msg.buf, buf, 8);
  msg.flags.remote = remote;

  int write_code = flexcan_bus.write(msg);
  if (write_code == -1) {
    return ODRIVE_CMD_ERROR_WRITE_FAILED;
  }
  return ODRIVE_CMD_SUCCESS;
}

/**
 * @brief Send a global command (defaults to axis 0)
 * @param cmd_id Command ID number
 * @param remote RTR bit for CAN message (typically 0 for setters and 1 for requesters)
 * @param buf 8-wide array of command data bytes
 * @return Status code of command send
 */
int ODrive::send_command(int cmd_id, bool remote, uint8_t buf[8]) {

  return ODrive::send_command(0, cmd_id, remote, buf);
}

/**
 * @brief Send a command with no particular data (typically used for requesters)
 * @param axis Axis number
 * @param cmd_id Command ID number
 * @param remote RTR bit for CAN message (typically 0 for setters and 1 for requesters)
 * @return Status code of command send
 */
int ODrive::send_empty_command(int axis, int cmd_id, bool remote) {
  uint8_t buf[8] = {0};
  return ODrive::send_command(axis, cmd_id, remote, buf);
}

/**
 * @brief Send a global command with no particular data (typically used for requesters; defaults to axis 0)
 * @param cmd_id Command ID number
 * @param remote RTR bit for CAN message (typically 0 for setters and 1 for requesters)
 * @return Status code of command send
 */
int ODrive::send_empty_command(int cmd_id, bool remote) {
  return ODrive::send_empty_command(0, cmd_id, remote);
}

/**
 * @brief Parse a CAN message and update ODrive members
 * @param msg CAN message to parse
 */
void ODrive::parse_message(const CAN_message_t& msg) {
  uint32_t axis = msg.id >> 5;
  uint32_t cmd_id = msg.id & 0x1F;

  switch (cmd_id) {
    case CAN_HEARTBEAT_MSG:
      // Cyclic message; sent every 100ms
      last_heartbeat_ms = millis();
      memcpy(&axis_error[axis], msg.buf, 4);
      memcpy(&axis_state[axis], msg.buf + 4, 1);
      memcpy(&motor_flags[axis], msg.buf + 5, 1);
      memcpy(&encoder_flags[axis], msg.buf + 6, 1);
      memcpy(&controller_flags[axis], msg.buf + 7, 1);
      break;
    case CAN_GET_MOTOR_ERROR:
      memcpy(&motor_error[axis], msg.buf, 4);
      break;
    case CAN_GET_ENCODER_ERROR:
      memcpy(&encoder_error[axis], msg.buf, 4);
      break;
    case CAN_GET_SENSORLESS_ERROR:
      memcpy(&sensorless_error[axis], msg.buf, 4);
      break;
    case CAN_GET_ENCODER_ESTIMATES:
      // Cyclic message; sent every 10ms
      memcpy(&pos_estimate[axis], msg.buf, 4);
      memcpy(&vel_estimate[axis], msg.buf + 4, 4);
      break;
    case CAN_GET_ENCODER_COUNT:
      memcpy(&shadow_count[axis], msg.buf, 4);
      memcpy(&count_in_cpr[axis], msg.buf + 4, 4);
      break;
    case CAN_GET_IQ:
      memcpy(&gpio_states, msg.buf, 4);
      memcpy(&iq_measured[axis], msg.buf + 4, 4);
      break;
    case CAN_GET_SENSORLESS_ESTIMATES:
      memcpy(&sensorless_pos_estimate[axis], msg.buf, 4);
      memcpy(&sensorless_vel_estimate[axis], msg.buf + 4, 4);
      break;
    case CAN_GET_VBUS_VOLTAGE:
      memcpy(&vbus_voltage, msg.buf, 4);
      memcpy(&vbus_current, msg.buf + 4, 4);
      break;
      /*
    case CAN_GET_GPIO_STATES:
      memcpy(&gpio_states, msg.buf, 4);
      break;
    */
  }
}

/**
 * Requests ODrive data over the CAN bus. This occurs asynchronously through an interrupt that calls ODrive.parse_message
 * when the message is recieved, meaning that class members won't be updated immediately.
 */
#pragma region REQUESTERS

int ODrive::request_motor_error(int axis) {
  return send_empty_command(axis, CAN_GET_MOTOR_ERROR, 1);
}

int ODrive::request_encoder_error(int axis) {
  return send_empty_command(axis, CAN_GET_ENCODER_ERROR, 1);
}

int ODrive::request_sensorless_error(int axis) {
  return send_empty_command(axis, CAN_GET_SENSORLESS_ERROR, 1);
}

int ODrive::request_encoder_count(int axis) {
  return send_empty_command(axis, CAN_GET_ENCODER_COUNT, 1);
}

int ODrive::request_iq(int axis) {
  return send_empty_command(axis, CAN_GET_IQ, 1);
}

int ODrive::request_sensorless_estimates(int axis) {
  return send_empty_command(axis, CAN_GET_SENSORLESS_ESTIMATES, 1);
}

int ODrive::request_vbus_voltage() {
  return send_empty_command(CAN_GET_VBUS_VOLTAGE, 1);
}

int ODrive::request_gpio_states() {
  // TODO: Implement GPIO reading
  // return send_empty_command(CAN_GET_GPIO_STATES, 1);
  return request_iq(0);  
}
#pragma endregion REQUESTERS

/**
 * Getters for the various class ODrive class members that are updated by CAN messages.
 */
#pragma region GETTERS
uint32_t ODrive::get_time_since_heartbeat_ms() {
  return millis() - last_heartbeat_ms;
}

uint32_t ODrive::get_axis_error(int axis) {
  return axis_error[axis];
}

uint8_t ODrive::get_axis_state(int axis) {
  return axis_state[axis];
}

uint8_t ODrive::get_motor_flags(int axis) {
  return motor_flags[axis];
}

uint8_t ODrive::get_encoder_flags(int axis) {
  return encoder_flags[axis];
}

uint8_t ODrive::get_controller_flags(int axis) {
  return controller_flags[axis];
}

uint32_t ODrive::get_motor_error(int axis) {
  return motor_error[axis];
}

uint32_t ODrive::get_encoder_error(int axis) {
  return encoder_error[axis];
}

uint32_t ODrive::get_sensorless_error(int axis) {
  return sensorless_error[axis];
}

float ODrive::get_vel_estimate(int axis) {
  return vel_estimate[axis];
}

float ODrive::get_pos_estimate(int axis) {
  return pos_estimate[axis];
}

int32_t ODrive::get_shadow_count(int axis) {
  return shadow_count[axis];
}

int32_t ODrive::get_count_in_cpr(int axis) {
  return count_in_cpr[axis];
}

float ODrive::get_iq_setpoint(int axis) {
  return iq_setpoint[axis];
}

float ODrive::get_iq_measured(int axis) {
  return iq_measured[axis];
}

float ODrive::get_sensorless_vel_estimate(int axis) {
  return sensorless_vel_estimate[axis];
}

float ODrive::get_sensorless_pos_estimate(int axis) {
  return sensorless_pos_estimate[axis];
}

float ODrive::get_voltage() {
  return vbus_voltage;
}

float ODrive::get_current() {
  return vbus_current;
}

uint32_t ODrive::get_gpio_states() {
  return gpio_states;
}

uint8_t ODrive::get_gpio(uint8_t pin) {
  return (gpio_states >> pin) & 0x1;
}
#pragma endregion GETTERS

/**
 * Commands for the ODrive.
 */
#pragma region COMMANDS
int ODrive::start_anticogging(int axis) {
  return send_command(axis, CAN_START_ANTICOGGING, 0);
}

int ODrive::reboot() {
  return send_empty_command(CAN_REBOOT_ODRIVE, 0);
}

int ODrive::clear_errors() {
  return send_empty_command(CAN_CLEAR_ERRORS, 0);
}
#pragma endregion COMMANDS

/**
 * Commands for the ODrive.
 */
#pragma region SETTERS
int ODrive::set_state(int axis, int state) {
  if (get_axis_state(axis) == state) {
    return 0;
  }
  uint8_t buf[8] = {0};
  memcpy(buf, &state, 4);
  return send_command(axis, CAN_SET_AXIS_REQUESTED_STATE, 0, buf);
}

int ODrive::set_axis_node_id(int axis, int axis_node_id) {
  uint8_t buf[8] = {0};
  memcpy(buf, &axis_node_id, 4);
  return send_command(axis, CAN_SET_AXIS_NODE_ID, 0);
}

int ODrive::set_controller_modes(int axis, int control_mode, int input_mode) {
  uint8_t buf[8] = {0};
  memcpy(buf, &control_mode, 4);
  memcpy(buf + 4, &input_mode, 4);
  return send_command(axis, CAN_SET_CONTROLLER_MODES, 0);
}

int ODrive::set_input_pos(int axis, float input_pos, int16_t vel_ff,
                          int16_t torque_ff) {
  uint8_t buf[8] = {0};
  memcpy(buf, &input_pos, 4);
  memcpy(buf + 4, &vel_ff, 2);
  memcpy(buf + 6, &torque_ff, 2);
  return send_command(axis, CAN_SET_INPUT_POS, 0);
}

int ODrive::set_input_vel(int axis, float input_vel, float torque_ff) {
  uint8_t buf[8] = {0};
  memcpy(buf, &input_vel, 4);
  memcpy(buf + 4, &torque_ff, 4);
  return send_command(axis, CAN_SET_INPUT_VEL, 0, buf);
}

int ODrive::set_input_torque(int axis, float input_torque) {
  uint8_t buf[8] = {0};
  memcpy(buf, &input_torque, 4);

  return send_command(axis, CAN_SET_INPUT_TORQUE, 0);
}

int ODrive::set_limits(int axis, float current_limit, float vel_limit) {
  uint8_t buf[8] = {0};
  memcpy(buf, &current_limit, 4);
  memcpy(buf + 4, &vel_limit, 4);
  return send_command(axis, CAN_SET_LIMITS, 0);
}

int ODrive::set_traj_vel_limit(int axis, float traj_vel_limit) {
  uint8_t buf[8] = {0};
  memcpy(buf, &traj_vel_limit, 4);
  return send_command(axis, CAN_SET_TRAJ_VEL_LIMIT, 0);
}

int ODrive::set_traj_accel_limits(int axis, float traj_decel_limit,
                                  float traj_accel_limit) {
  uint8_t buf[8] = {0};
  memcpy(buf + 4, &traj_decel_limit, 4);
  memcpy(buf, &traj_accel_limit, 4);
  return send_command(axis, CAN_SET_TRAJ_ACCEL_LIMITS, 0);
}

int ODrive::set_traj_intertia(int axis, float traj_inertia) {
  uint8_t buf[8] = {0};
  memcpy(buf, &traj_inertia, 4);
  return send_command(axis, CAN_SET_TRAJ_INERTIA, 0);
}

int ODrive::set_linear_count(int axis, float position) {
  uint8_t buf[8] = {0};
  memcpy(buf, &position, 4);

  return send_command(axis, CAN_SET_LINEAR_COUNT, 0);
}

int ODrive::set_pos_gain(int axis, float pos_gain) {
  uint8_t buf[8] = {0};
  memcpy(buf, &pos_gain, 4);
  return send_command(axis, CAN_SET_POSITION_GAIN, 0);
}

int ODrive::set_vel_gains(int axis, float vel_gain, float vel_integrator_gain) {
  uint8_t buf[8] = {0};
  memcpy(buf, &vel_gain, 4);
  memcpy(buf + 4, &vel_integrator_gain, 4);
  return send_command(axis, CAN_SET_VEL_GAINS, 0);
}
#pragma endregion SETTERS