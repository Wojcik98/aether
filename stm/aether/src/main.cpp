#include "stm32f4xx.h" // IWYU pragma: keep
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_i2c.h"
#include <cstdint>

extern "C" {
#include "Board_Buttons.h" // ::Board Support:Buttons
#include "Board_LED.h"     // ::Board Support:LED
}

#include "vl53l4cd_stm.hpp"

#include "aether/map_confident.hpp"
#include "aether/map_in_progress.hpp"
#include "aether/mapped_localization.hpp"
#include "aether/robot_config.hpp"

void i2c_config();
void system_core_clock_config();
void error_handler();

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

VL53L4CD_STM tof_i2c;
VL53L4CD_API tof_api(&tof_i2c);

constexpr RobotConfig robot_config = {
    .robot_length = 0.18f,
    .motors_offset = 0.06f,
    .robot_width = 0.12f,
    .wheel_radius = 0.03f,
    .wheel_base = 0.12f,
    .wall_width = 0.012f,
    .cell_size = 0.18f,
    .starting_x = 0.072f,
    .starting_y = -0.09f,
    .starting_yaw = 0.0f,
    .freq_imu_enc = 100,
    .freq_tofs = 10,
    .tofs_poses =
        {
            {0.0f, 0.0f, 0.0f}, // right_side
            {0.0f, 0.0f, 0.0f}, // right_diag
            {0.0f, 0.0f, 0.0f}, // right_front
            {0.0f, 0.0f, 0.0f}, // left_front
            {0.0f, 0.0f, 0.0f}, // left_diag
            {0.0f, 0.0f, 0.0f}, // left_side
        },
};

MapInProgress map_in_progress;
MapConfident map_confident(map_in_progress);
MappedLocalization localization(robot_config, map_confident);

void blink() {
    LED_On(0);
    HAL_Delay(100);
    LED_Off(0);
    HAL_Delay(100);
    LED_On(0);
    HAL_Delay(100);
    LED_Off(0);
    HAL_Delay(100);
}

int main() {
    uint32_t button_msk = (1U << Buttons_GetCount()) - 1;
    SystemInit();

    system_core_clock_config();
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);

    LED_Initialize();
    Buttons_Initialize();

    i2c_config();

    tof_i2c.set_i2c(&hi2c1);
    tof_api.sensor_init();
    tof_api.start_ranging();

    while (true) {
        uint8_t is_data_ready = 0;
        tof_api.check_for_data_ready(&is_data_ready);

        if (is_data_ready) {
            VL53L4CD_ResultsData_t data;
            tof_api.get_result(&data);
            tof_api.clear_interrupt();

            if (data.distance_mm < 200 && data.distance_mm > 5) {
                LED_On(0);
            } else {
                LED_Off(0);
            }
        }
    }
}

extern "C" { // Interrupts handlers need to be "visible" in C
void SysTick_Handler(void) { HAL_IncTick(); }
}

void HAL_IncTick(void) { uwTick += 1; }

void i2c_config() {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        error_handler();
    }
}

void system_core_clock_config() {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure the main internal regulator output voltage
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Initializes the RCC Oscillators according to the specified parameters
    // in the RCC_OscInitTypeDef structure.
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        error_handler();
    }

    // Activate the Over-Drive mode
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        error_handler();
    }

    // Initializes the CPU, AHB and APB buses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        error_handler();
    }
}

void HAL_MspInit(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hi2c->Instance == I2C1) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        // I2C1 GPIO Configuration
        // PB8     ------> I2C1_SCL
        // PB9     ------> I2C1_SDA
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        // Peripheral clock enable
        __HAL_RCC_I2C1_CLK_ENABLE();
    }
}

void error_handler() {
    while (true) {
        // __NOP(); // Error
    }
}
