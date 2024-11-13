#include <atomic>
#include <cstdint>
extern "C" {
#include "stm32f4xx.h" // IWYU pragma: keep
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"

#include "Board_Buttons.h" // ::Board Support:Buttons
#include "Board_LED.h"     // ::Board Support:LED
}

#define RF24_SPI_SPEED (100000000 / 128)
#include "rf24/gpio.hpp"
#include "rf24/rf24.hpp"
#include "rf24/spi.hpp"

#include <Eigen/Dense>

#include "hexapod.hpp"
#include "legs.hpp"
#include "servo_comm.hpp"
#include "tripod.hpp"

std::atomic_bool initialized;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef dma_spi3_rx;

// PC1      ------> RF24_CS
// PC0      ------> RF24_CE
HalPin ce{GPIOC, GPIO_PIN_0};
HalPin csn{GPIOC, GPIO_PIN_1};

Rf24Config rf24_config = {
    ce, csn, 4, 32, false, 1,
};

Stm32SpiDma spi(&hspi3, &dma_spi3_rx, &dma_spi3_rx, csn);

SimpleRf24 radio(rf24_config, &spi);

uint8_t data[32];

Hexapod hexapod{
    .angles_offsets =
        Legsf{
            Legf{0.0f, 0.0f, 0.0f},
            Legf{0.0f, 0.0f, 0.0f},
            Legf{0.0f, 0.0f, 0.0f},
            Legf{0.0f, 0.0f, 0.0f},
            Legf{0.0f, 0.0f, 0.0f},
            Legf{0.0f, 0.0f, 0.0f},
        },
    .hip_offsets =
        LegsVec3f{
            Eigen::Vector3f(0.088f, 0.051f, 0.0f),
            Eigen::Vector3f(0.0f, 0.071f, 0.0f),
            Eigen::Vector3f(-0.088f, 0.051f, 0.0f),
            Eigen::Vector3f(0.088f, -0.051f, 0.0f),
            Eigen::Vector3f(0.0f, -0.071f, 0.0f),
            Eigen::Vector3f(-0.088f, -0.051f, 0.0f),
        },
    .leg_dims = Legf{0.0294f, 0.078f, 0.134f},
};
// clang-format off
constexpr float TRIPOD_F_X = 0.15f;
constexpr float TRIPOD_R_X = -TRIPOD_F_X;
constexpr float TRIPOD_F_Y = 0.15f;
constexpr float TRIPOD_R_Y = TRIPOD_F_Y;
constexpr float TRIPOD_M_X = 0.0f;
constexpr float TRIPOD_M_Y = -0.20f;

TripodConfig config = {
    ServoComm::SERVO_FREQ,   // servo_freq
    0.2f, // gait_period
    0.14f, // base_height
    0.03f, // leg_height
    TripodConfig::Tripod_{
        Eigen::Vector3f(TRIPOD_F_X, TRIPOD_F_Y, 0.0f),
        Eigen::Vector3f(TRIPOD_M_X, TRIPOD_M_Y, 0.0f),
        Eigen::Vector3f(TRIPOD_R_X, TRIPOD_R_Y, 0.0f),
    },
    TripodConfig::Tripod_{
        Eigen::Vector3f(TRIPOD_F_X, -TRIPOD_F_Y, 0.0f),
        Eigen::Vector3f(TRIPOD_M_X, -TRIPOD_M_Y, 0.0f),
        Eigen::Vector3f(TRIPOD_R_X, -TRIPOD_R_Y, 0.0f),
    }
};
// clang-format on
Tripod tripod(hexapod, config);

const ServoComm::LegsServoConfig servo_config = {
    Leg<ServoComm::ServoConfig>{ServoComm::ServoConfig{9, 2150, -1},
                                ServoComm::ServoConfig{10, 1920, 1},
                                ServoComm::ServoConfig{11, 600, -1}},
    Leg<ServoComm::ServoConfig>{ServoComm::ServoConfig{12, 2100, -1},
                                ServoComm::ServoConfig{13, 1800, 1},
                                ServoComm::ServoConfig{14, 600, -1}},
    Leg<ServoComm::ServoConfig>{ServoComm::ServoConfig{21, 2600, -1},
                                ServoComm::ServoConfig{22, 1650, 1},
                                ServoComm::ServoConfig{23, 700, -1}},
    Leg<ServoComm::ServoConfig>{ServoComm::ServoConfig{6, 810, -1},
                                ServoComm::ServoConfig{7, 1150, -1},
                                ServoComm::ServoConfig{8, 2440, 1}},
    Leg<ServoComm::ServoConfig>{ServoComm::ServoConfig{15, 1150, -1},
                                ServoComm::ServoConfig{16, 1370, -1},
                                ServoComm::ServoConfig{17, 2300, 1}},
    Leg<ServoComm::ServoConfig>{ServoComm::ServoConfig{18, 450, -1},
                                ServoComm::ServoConfig{19, 1100, -1},
                                ServoComm::ServoConfig{20, 2450, 1}},
};
ServoComm servo_comm(servo_config);

void SystemCoreClockConfigure();
void SPI_Init();

extern "C" { // Interrupts handlers need to be "visible" in C
void SysTick_Handler(void) {
    HAL_IncTick();

    if (!initialized) {
        return;
    }

    // Send at the start, so that the frequency is more stable
    servo_comm.trigger_transmit();
    Legsf angle;
    tripod.get_next_joints(angle);
    servo_comm.prepare_buffer(angle);
}
void SPI3_IRQHandler(void) { HAL_SPI_IRQHandler(&hspi3); }

void DMA1_Stream2_IRQHandler(void) { HAL_DMA_IRQHandler(hspi3.hdmarx); }

void DMA1_Stream5_IRQHandler(void) { HAL_DMA_IRQHandler(hspi3.hdmatx); }
}

void HAL_IncTick(void) { uwTick += 4; }

bool check_rf24_irq() {
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET;
}

union FloatBytes {
    float f[2];
    uint8_t b[8];
};

int main() {
    uint32_t button_msk = (1U << Buttons_GetCount()) - 1;
    initialized = false;
    SystemInit();

    SystemCoreClockConfigure();
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / ServoComm::SERVO_FREQ);

    LED_Initialize();
    Buttons_Initialize();
    SPI_Init();

    radio.begin();
    radio.enable_irq(true, false, false);

    HAL_Delay(10);
    radio.flush_rx();
    radio.start_listening();
    HAL_Delay(10);

    LED_On(0);
    HAL_Delay(100);
    LED_Off(0);
    HAL_Delay(100);
    LED_On(0);
    HAL_Delay(100);
    LED_Off(0);

    hexapod.leg_dims = Legf{0.0294f, 0.079f, 0.129f};
    hexapod.hip_offsets.fl = Eigen::Vector3f(0.087598f, 0.050575f, 0.0f);
    hexapod.hip_offsets.ml = Eigen::Vector3f(0.0f, 0.06985f, 0.0f);
    hexapod.hip_offsets.rl = Eigen::Vector3f(-0.087598f, 0.050575f, 0.0f);
    hexapod.hip_offsets.fr = Eigen::Vector3f(0.087598f, -0.050575f, 0.0f);
    hexapod.hip_offsets.mr = Eigen::Vector3f(0.0f, -0.06985f, 0.0f);
    hexapod.hip_offsets.rr = Eigen::Vector3f(-0.087598f, -0.050575f, 0.0f);

    servo_comm.init();
    initialized = true;

    constexpr uint32_t TIMEOUT = 2000;
    volatile bool irq = 0;
    bool timeout = false;
    FloatBytes fb;

    while (true) {
        irq = check_rf24_irq();
        timeout = false;
        uint32_t start = HAL_GetTick();
        while (!irq) {
            irq = check_rf24_irq();
            if (HAL_GetTick() - start > TIMEOUT) {
                timeout = true;
                break;
            }
        }
        if (radio.available()) {
            radio.read(fb.b, 8);
            radio.flush_rx();
            radio.clear_irq(true, false, false);
        }
        if (timeout) { // stop on timeout
            fb.f[0] = 0.0f;
            fb.f[1] = 0.0f;
        }

        if (fb.f[0] > 0.01f) {
            LED_On(0);
        } else {
            LED_Off(0);
        }

        __disable_irq();
        // only place where race condition could happen
        // walking straight
        // tripod.set_twist({0.06f, 0.0f, 0.0f, -0.015f});
        // tripod.set_twist({0.06f, 0.0f, 0.0f, -0.015f});
        // tripod.set_twist({0.20f, 0.0f, 0.0f, -0.05f});
        // tripod.set_twist({0.40f, 0.0f, 0.0f, -0.1f});
        tripod.set_twist({fb.f[0], 0.0f, 0.0f, fb.f[1]});
        __enable_irq();
    }
}

/*----------------------------------------------------------------------------
 * SystemCoreClockConfigure: configure SystemCoreClock using HSI
                             (HSE is not populated on Nucleo board)
 *----------------------------------------------------------------------------*/
void SystemCoreClockConfigure() {
    // TODO use HSE
    RCC->CR |= ((uint32_t)RCC_CR_HSION); /* Enable HSI */
    while ((RCC->CR & RCC_CR_HSIRDY) == 0) {
    } /* Wait for HSI Ready */

    RCC->CFGR = RCC_CFGR_SW_HSI; /* HSI is system clock */
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {
    } /* Wait for HSI used as system clock */

    FLASH->ACR = (FLASH_ACR_PRFTEN |      /* Enable Prefetch Buffer */
                  FLASH_ACR_ICEN |        /* Instruction cache enable */
                  FLASH_ACR_DCEN |        /* Data cache enable */
                  FLASH_ACR_LATENCY_5WS); /* Flash 5 wait state */

    RCC->CFGR |= (RCC_CFGR_HPRE_DIV1 |  /* HCLK = SYSCLK */
                  RCC_CFGR_PPRE1_DIV2 | /* APB1 = HCLK/2 */
                  RCC_CFGR_PPRE2_DIV1); /* APB2 = HCLK/1 */

    RCC->CR &= ~RCC_CR_PLLON; /* Disable PLL */

    /* PLL configuration:  VCO = HSI/M * N,  Sysclk = VCO/P */
    RCC->PLLCFGR = (16ul |                     /* PLL_M =  16 */
                    (200ul << 6) |             /* PLL_N = 200 */
                    (0ul << 16) |              /* PLL_P =   2 */
                    (RCC_PLLCFGR_PLLSRC_HSI) | /* PLL_SRC = HSI */
                    (7ul << 24) |              /* PLL_Q =   7 */
                    (2ul << 28));              /* PLL_R =   2 */

    RCC->CR |= RCC_CR_PLLON; /* Enable PLL */
    while ((RCC->CR & RCC_CR_PLLRDY) == 0) {
        __NOP(); /* Wait till PLL is ready */
    }

    RCC->CFGR &= ~RCC_CFGR_SW; /* Select PLL as system clock source */
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    } /* Wait till PLL is system clock src */
}

void SPI_Init() {
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.BaudRatePrescaler =
        SPI_BAUDRATEPRESCALER_128; // 100MHz / 128 = 781.25kHz
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 10;
    HAL_SPI_Init(&hspi3);

    HAL_NVIC_EnableIRQ(SPI3_IRQn);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
    static DMA_HandleTypeDef hdma_tx;
    static DMA_HandleTypeDef hdma_rx;

    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /**SPI3 GPIO Configuration
    PB0      ------> SPI3_MOSI
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PA4      ------> RF24_IRQ
    */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Alternate = GPIO_AF7_SPI3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // TODO interrupt
    // just input for now
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*##-3- Configure the DMA streams
     * ##########################################*/
    /* Configure the DMA handler for Transmission process */
    hdma_tx.Instance = DMA1_Stream5;
    hdma_tx.Init.Channel = DMA_CHANNEL_0;
    hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode = DMA_NORMAL;
    hdma_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_tx.Init.MemBurst = DMA_MBURST_INC4;
    hdma_tx.Init.PeriphBurst = DMA_PBURST_INC4;

    HAL_DMA_Init(&hdma_tx);

    /* Associate the initialized DMA handle to the the SPI handle */
    __HAL_LINKDMA(hspi, hdmatx, hdma_tx);

    /* Configure the DMA handler for Transmission process */
    hdma_rx.Instance = DMA1_Stream2;
    hdma_rx.Init.Channel = DMA_CHANNEL_0;
    hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_rx.Init.Mode = DMA_NORMAL;
    hdma_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_rx.Init.MemBurst = DMA_MBURST_INC4;
    hdma_rx.Init.PeriphBurst = DMA_PBURST_INC4;

    HAL_DMA_Init(&hdma_rx);

    /* Associate the initialized DMA handle to the the SPI handle */
    __HAL_LINKDMA(hspi, hdmarx, hdma_rx);

    /*##-4- Configure the NVIC for DMA
     * #########################################*/
    /* NVIC configuration for DMA transfer complete interrupt (SPI3_TX) */
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

    /* NVIC configuration for DMA transfer complete interrupt (SPI3_RX) */
    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

    /*##-5- Configure the NVIC for SPI
     * #########################################*/
    HAL_NVIC_SetPriority(SPI3_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(SPI3_IRQn);
}
