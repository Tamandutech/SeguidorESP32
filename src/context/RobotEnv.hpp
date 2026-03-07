#ifndef ROBOT_ENV_HPP
#define ROBOT_ENV_HPP

namespace RobotEnv {
const int32_t ROBOT_WIDTH                 = 4;
const int32_t WHEEL_RADIUS                = 11;
const int32_t WHEEL_CIRCUMFERENCE         = 70;
const int32_t ENCODER_PULSES_PER_ROTATION = 4095;

const int32_t MAX_SPEED          = 4;
const int32_t MAX_DECELERATION   = 4;
const int32_t MAX_ROTATION_SPEED = 4;

const int32_t MAPPING_MOTOR_PWM = 15;
const int32_t BASE_VACUUM_PWM   = 100;

const int32_t MAX_MOTOR_PWM = 66;

const int32_t SIDE_SENSOR_READ_AVERAGE_COUNT = 5;

constexpr float EPSILON_TOLERANCE =
    1e-6F; // Tolerância para comparações de ponto flutuante

// Constantes para prevenção de integral windup no PID
constexpr float INTEGRAL_MAX = 1000.0F;  // Valor máximo para o termo integral
constexpr float INTEGRAL_MIN = -1000.0F; // Valor mínimo para o termo integral

const uint8_t GPIO_LED_DEBUG = 47;

const uint8_t GPIO_BATTERY_VOLTAGE = 18;

const uint8_t GPIO_DIRECTION_A = 9;
const uint8_t GPIO_DIRECTION_B = 37;
const uint8_t GPIO_PWM_A       = 3;
const uint8_t GPIO_PWM_B       = 38;

const uint8_t GPIO_PWM_VACUUM = 11;

const uint8_t GPIO_ENCODER_LEFT_A  = 7;
const uint8_t GPIO_ENCODER_LEFT_B  = 6;
const uint8_t GPIO_ENCODER_RIGHT_A = 12;
const uint8_t GPIO_ENCODER_RIGHT_B = 13;

const uint8_t GPIO_MULTIPLEXER_DIGITAL_ADDRESS[]    = {39, 40, 41, 42};
const uint8_t GPIO_MULTIPLEXER_ANALOG_INPUT         = 10;
const uint8_t GPIO_MULTIPLEXER_LINE_SENSORS_INDEX[] = {13, 12, 11, 10, 9, 8,
                                                       5,  4,  3,  2,  1, 0};
const uint8_t GPIO_MULTIPLEXER_SIDE_SENSORS_INDEX[] = {15, 14, 6, 7};

} // namespace RobotEnv

#endif // ROBOT_ENV_HPP
