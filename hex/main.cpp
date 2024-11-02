#include <atomic>
extern "C" {
#include "stm32f4xx.h" // IWYU pragma: keep

#include "Board_Buttons.h" // ::Board Support:Buttons
#include "Board_LED.h"     // ::Board Support:LED
}

#include <Eigen/Dense>

#include "hexapod.hpp"
#include "legs.hpp"
#include "servo_comm.hpp"
#include "tripod.hpp"

std::atomic_bool initialized;
volatile uint32_t msTicks = 0;
volatile uint32_t ledOn = 0;

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

void SystemCoreClockConfigure(void);

extern "C" { // Interrupts handlers need to be "visible" in C
void SysTick_Handler(void) {
    HAL_IncTick();

    if (!initialized) {
        return;
    }

    // Send at the start, so that the frequency is more stable
    servo_comm.trigger_transmit();
    LED_On(0);
    Legsf angle;
    // auto angle = Legsf{
    //     Legf{M_PI_2, 0.0f, 0.0f},  Legf{M_PI_2, 0.0f, 0.0f},
    //     Legf{M_PI_2, 0.0f, 0.0f},  Legf{-M_PI_2, 0.0f, 0.0f},
    //     Legf{-M_PI_2, 0.0f, 0.0f}, Legf{-M_PI_2, 0.0f, 0.0f},
    // };
    tripod.get_next_joints(angle);
    servo_comm.prepare_buffer(angle);
    LED_Off(0);

    ++msTicks;
}
}

void HAL_IncTick(void) { uwTick += 4; }

int main() {
    uint32_t button_msk = (1U << Buttons_GetCount()) - 1;
    initialized = false;
    SystemInit();

    SystemCoreClockConfigure();
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / ServoComm::SERVO_FREQ);

    LED_Initialize();
    Buttons_Initialize();

    hexapod.leg_dims = Legf{0.0294f, 0.079f, 0.129f};
    hexapod.hip_offsets.fl = Eigen::Vector3f(0.087598f, 0.050575f, 0.0f);
    hexapod.hip_offsets.ml = Eigen::Vector3f(0.0f, 0.06985f, 0.0f);
    hexapod.hip_offsets.rl = Eigen::Vector3f(-0.087598f, 0.050575f, 0.0f);
    hexapod.hip_offsets.fr = Eigen::Vector3f(0.087598f, -0.050575f, 0.0f);
    hexapod.hip_offsets.mr = Eigen::Vector3f(0.0f, -0.06985f, 0.0f);
    hexapod.hip_offsets.rr = Eigen::Vector3f(-0.087598f, -0.050575f, 0.0f);

    servo_comm.init();
    initialized = true;

    while (true) {
        // ++dummy;
        if ((Buttons_GetState() & (button_msk))) {
            HAL_Delay(100); // debounce

            __disable_irq();
            // only place where race condition could happen
            // walking straight
            // tripod.set_twist({0.06f, 0.0f, 0.0f, -0.015f});
            // tripod.set_twist({0.06f, 0.0f, 0.0f, -0.015f});
            // tripod.set_twist({0.20f, 0.0f, 0.0f, -0.05f});
            tripod.set_twist({0.40f, 0.0f, 0.0f, -0.1f});
            __enable_irq();

            while ((Buttons_GetState() & (button_msk))) {
                // wait for button release
            }
            HAL_Delay(100); // debounce
        }
        // LED_Off(0);
    }
}

/*----------------------------------------------------------------------------
 * SystemCoreClockConfigure: configure SystemCoreClock using HSI
                             (HSE is not populated on Nucleo board)
 *----------------------------------------------------------------------------*/
void SystemCoreClockConfigure(void) {
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
