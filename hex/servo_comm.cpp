#include "servo_comm.hpp"

#include <array>
#include <cstdint>

#include "stm32f4xx.h" // IWYU pragma: keep

#include "hexapod.hpp"
#include "legs.hpp"
extern "C" {
// #include "stm32f4xx.h" // IWYU pragma: keep

// #include "Board_Buttons.h" // ::Board Support:Buttons
#include "Board_LED.h" // ::Board Support:LED
}
volatile uint8_t MAESTRO_INIT = 0xAA;
DMA_HandleTypeDef dma_uart6_tx;
UART_HandleTypeDef uart6;

void ServoComm::init() volatile {
    UART_init();

    // Reset the Maestro
    maestro_reset(true);
    HAL_Delay(100);
    maestro_reset(false);
    HAL_Delay(100);
    HAL_UART_Transmit_DMA(&uart6, (uint8_t *)&MAESTRO_INIT, 1);
    HAL_Delay(100);

    // Initialize the buffers
    for (uint8_t i = 0; i < 2; i++) {
        for (uint8_t j = 0; j < SINGLE_CMD_SIZE; j++) {
            buffers[i][j] = 0;
        }
    }

    // Find the lowest pin
    lowest_pin = config.fl.coxa.pin;
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        if (config[i].coxa.pin < lowest_pin) {
            lowest_pin = config[i].coxa.pin;
        }
        if (config[i].femur.pin < lowest_pin) {
            lowest_pin = config[i].femur.pin;
        }
        if (config[i].tibia.pin < lowest_pin) {
            lowest_pin = config[i].tibia.pin;
        }
    }
}

void ServoComm::prepare_buffer(const Legsf &angles) volatile {
    volatile uint8_t *cmd = buffers[current_buffer];
    // Setting multitarget, servos MUST be on subsequent pins
    // Protocol: 0x9F, num of target, first channel number, cmds...
    cmd[0] = MULTIPLE_TARGETS_CMD;
    cmd[1] = NUM_JOINTS;
    // cmd[1] = 3;
    cmd[2] = lowest_pin;

    auto duties_sorted = std::array<uint16_t, NUM_JOINTS>();
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        auto coxa = angle_to_duty(angles[i].coxa, config[i].coxa);
        auto femur = angle_to_duty(angles[i].femur, config[i].femur);
        auto tibia = angle_to_duty(angles[i].tibia, config[i].tibia);

        duties_sorted[config[i].coxa.pin - lowest_pin] = coxa;
        duties_sorted[config[i].femur.pin - lowest_pin] = femur;
        duties_sorted[config[i].tibia.pin - lowest_pin] = tibia;
    }

    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        volatile uint8_t *ptr = cmd + HEADER_SIZE + i * JOINT_CMD_SIZE;
        encode_single(ptr, duties_sorted[i]);
    }
}

void ServoComm::trigger_transmit() volatile {
    volatile uint8_t *cmd = buffers[current_buffer];
    // Send the buffer
    if (cmd[0] == MULTIPLE_TARGETS_CMD) {
        auto resp =
            HAL_UART_Transmit_DMA(&uart6, (uint8_t *)cmd, SINGLE_CMD_SIZE);
    }

    // Switch the buffer
    current_buffer = (current_buffer + 1) % 2;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    while (1) {
        LED_On(0);
    }
}

void ServoComm::encode_single(volatile uint8_t *ptr,
                              volatile uint16_t duty) volatile {
    duty *= 4;
    ptr[0] = duty & CMD_MASK;
    ptr[1] = (duty >> 7) & CMD_MASK;
}

uint16_t ServoComm::angle_to_duty(float angle,
                                  const ServoConfig &cfg) volatile {
    int16_t displacement = int16_t(angle * 1000.0f / (M_PI / 2.0f)) * cfg.dir;
    int16_t duty = cfg.center + displacement;
    if (duty < 0 && duty + 4000 <= 2550) {
        duty += 4000;
    }
    if (duty < 460) {
        duty = 460;
    }
    if (duty > 2550) {
        duty = 2550;
    }

    return duty;
}

void ServoComm::UART_init() volatile {
    __HAL_RCC_USART6_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // USART6 GPIO Configuration
    // PC8   ------>   MAESTRO_RST
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // PC7   ------>   USART6_RX
    // PC6   ------>   USART6_TX
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    uart6.Instance = USART6;
    uart6.Init.BaudRate = 115200;
    uart6.Init.WordLength = UART_WORDLENGTH_8B;
    uart6.Init.StopBits = UART_STOPBITS_1;
    uart6.Init.Parity = UART_PARITY_NONE;
    // uart6.Init.Mode = UART_MODE_TX_RX;
    uart6.Init.Mode = UART_MODE_TX;
    uart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart6.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&uart6);

    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
    NVIC_EnableIRQ(USART6_IRQn);
}

void ServoComm::maestro_reset(bool state) volatile {
    if (state) { // Reset with a low signal
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    }
}

// *****************************************************************************
// * HAL Callbacks and IRQ Handlers
// *****************************************************************************

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
    __HAL_RCC_DMA2_CLK_ENABLE();

    dma_uart6_tx.Instance = DMA2_Stream6;
    dma_uart6_tx.Init.Channel = DMA_CHANNEL_5;
    dma_uart6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    dma_uart6_tx.Init.MemInc = DMA_MINC_ENABLE;
    dma_uart6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    dma_uart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dma_uart6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dma_uart6_tx.Init.Mode = DMA_NORMAL;
    dma_uart6_tx.Init.Priority = DMA_PRIORITY_HIGH;
    dma_uart6_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&dma_uart6_tx);

    __HAL_LINKDMA(huart, hdmatx, dma_uart6_tx);

    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}

extern "C" {
void USART6_IRQHandler() { HAL_UART_IRQHandler(&uart6); }

void DMA2_Stream6_IRQHandler(void) { HAL_DMA_IRQHandler(&dma_uart6_tx); }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {}
