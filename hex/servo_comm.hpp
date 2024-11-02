#ifndef __SERVO_COMM_HPP
#define __SERVO_COMM_HPP

#include "stm32f4xx.h" // IWYU pragma: keep

#include <cstdint>

#include "hexapod.hpp"
#include "legs.hpp"

/** ServoComm is a class that communicates with the Pololu Maestro servo
 * controller over USART.
 *
 * As the communication parameters are not likely to change much, they are
 * hardcoded in the class.
 */
class ServoComm {
public:
    struct ServoConfig {
        uint8_t pin;
        uint16_t center;
        int16_t dir;
    };
    typedef Legs<Leg<ServoConfig>> LegsServoConfig;

    static constexpr uint8_t SERVO_FREQ = 250;
    static constexpr uint8_t HEADER_SIZE = 3;
    static constexpr uint8_t JOINT_CMD_SIZE = 2;
    static constexpr uint8_t SINGLE_CMD_SIZE =
        (HEADER_SIZE + NUM_JOINTS * JOINT_CMD_SIZE);

    ServoComm(const LegsServoConfig &config) : config(config){};
    void init() volatile;
    void prepare_buffer(const Legsf &angles) volatile;
    void trigger_transmit() volatile;

private:
    constexpr static uint8_t MULTIPLE_TARGETS_CMD = 0x9F;
    constexpr static uint16_t CMD_MASK = (1 << 7) - 1;

    void UART_init() volatile;
    void maestro_reset(bool state) volatile;
    void encode_single(volatile uint8_t *ptr, volatile uint16_t duty) volatile;
    uint16_t angle_to_duty(float angle, const ServoConfig &cfg) volatile;

    const LegsServoConfig &config;
    volatile uint8_t buffers[2][SINGLE_CMD_SIZE];
    volatile uint8_t current_buffer = 0;
    uint8_t lowest_pin = 0;
};

#endif // __SERVO_COMM_HPP
