#ifndef DATA_TYPES_HPP
#define DATA_TYPES_HPP

#include <stdint.h>

struct MapPoint {
  int32_t encoderMilimeters{};
  int32_t baseMotorPWM{};
  int32_t baseVacuumPWM{};
  enum MarkType {
    LEFT_MARK,
    RIGHT_MARK,
    HANDMADE_MARK,
    STOP_COMMAND_MARK,
    UNKNOWN_MARK
  } markType;
};

struct ParametersConfig {
  bool    runOnMappingMode{};
  int32_t vacuumPWM{};
  /// When true, line IR calibration min/max are forced to fixed values after
  /// \c calibrate() (see MainTask). Set via CLI \c
  /// Calibration.hardcodedCalibration.
  bool hardcodedCalibration{};
  /// Line-follow PID gains (PathController). Set via CLI \c PID.kP, \c PID.kI,
  /// \c PID.kD.
  float pidKp{};
  float pidKi{};
  float pidKd{};
  /// Base motor PWM magnitude while mapping (see MainTask MAPPING state).
  /// BLE: \c Mapping.mappingMotorPWM (clamped to \c RobotEnv::MAX_MOTOR_PWM).
  int32_t mappingMotorPWM{};
};

#endif // DATA_TYPES_HPP
