/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAP_CHG_EN_Pin GPIO_PIN_13
#define CAP_CHG_EN_GPIO_Port GPIOC
#define VBAT_ADC_Pin GPIO_PIN_0
#define VBAT_ADC_GPIO_Port GPIOC
#define MCU_AIN1_Pin GPIO_PIN_1
#define MCU_AIN1_GPIO_Port GPIOC
#define VCAP_ADC_Pin GPIO_PIN_2
#define VCAP_ADC_GPIO_Port GPIOC
#define MCU_DIN1_Pin GPIO_PIN_3
#define MCU_DIN1_GPIO_Port GPIOC
#define BLE_RXD_Pin GPIO_PIN_0
#define BLE_RXD_GPIO_Port GPIOA
#define BLE_TXD_Pin GPIO_PIN_1
#define BLE_TXD_GPIO_Port GPIOA
#define MCU_TXD2_Pin GPIO_PIN_2
#define MCU_TXD2_GPIO_Port GPIOA
#define MCU_RXD2_Pin GPIO_PIN_3
#define MCU_RXD2_GPIO_Port GPIOA
#define MCU_AIN0_Pin GPIO_PIN_4
#define MCU_AIN0_GPIO_Port GPIOA
#define MCU_SCLK1_Pin GPIO_PIN_5
#define MCU_SCLK1_GPIO_Port GPIOA
#define MCU_CTS3_Pin GPIO_PIN_6
#define MCU_CTS3_GPIO_Port GPIOA
#define NRST_PRT_EXP_Pin GPIO_PIN_7
#define NRST_PRT_EXP_GPIO_Port GPIOA
#define MCU_TXD3_Pin GPIO_PIN_4
#define MCU_TXD3_GPIO_Port GPIOC
#define MCU_RXD3_Pin GPIO_PIN_5
#define MCU_RXD3_GPIO_Port GPIOC
#define MCU_NCS1_Pin GPIO_PIN_0
#define MCU_NCS1_GPIO_Port GPIOB
#define MCU_RTS3_Pin GPIO_PIN_1
#define MCU_RTS3_GPIO_Port GPIOB
#define VCAP_ADC_EN_Pin GPIO_PIN_2
#define VCAP_ADC_EN_GPIO_Port GPIOB
#define MCU_RXD_Pin GPIO_PIN_10
#define MCU_RXD_GPIO_Port GPIOB
#define MCU_TXD_Pin GPIO_PIN_11
#define MCU_TXD_GPIO_Port GPIOB
#define RS485A_TX_NRE_Pin GPIO_PIN_12
#define RS485A_TX_NRE_GPIO_Port GPIOB
#define SCL_PRT_EXP_Pin GPIO_PIN_13
#define SCL_PRT_EXP_GPIO_Port GPIOB
#define SDA_PRT_EXP_Pin GPIO_PIN_14
#define SDA_PRT_EXP_GPIO_Port GPIOB
#define MCU_DIN3_Pin GPIO_PIN_15
#define MCU_DIN3_GPIO_Port GPIOB
#define MCU_VI0_SEL_Pin GPIO_PIN_6
#define MCU_VI0_SEL_GPIO_Port GPIOC
#define MCU_VI1_SEL_Pin GPIO_PIN_7
#define MCU_VI1_SEL_GPIO_Port GPIOC
#define RS485A_TX_DE_Pin GPIO_PIN_8
#define RS485A_TX_DE_GPIO_Port GPIOC
#define MCU_PRCH_VEN_Pin GPIO_PIN_9
#define MCU_PRCH_VEN_GPIO_Port GPIOC
#define PS_HV_EN_Pin GPIO_PIN_8
#define PS_HV_EN_GPIO_Port GPIOA
#define AIN0_24V_EN_Pin GPIO_PIN_9
#define AIN0_24V_EN_GPIO_Port GPIOA
#define NINT_PRT_EXP_Pin GPIO_PIN_10
#define NINT_PRT_EXP_GPIO_Port GPIOA
#define AIN1_24V_EN_Pin GPIO_PIN_11
#define AIN1_24V_EN_GPIO_Port GPIOA
#define MCU_PKY_2MDM_Pin GPIO_PIN_12
#define MCU_PKY_2MDM_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define BLE_CTS_Pin GPIO_PIN_15
#define BLE_CTS_GPIO_Port GPIOA
#define MCU_DIN0_Pin GPIO_PIN_10
#define MCU_DIN0_GPIO_Port GPIOC
#define MCU_DIN2_Pin GPIO_PIN_11
#define MCU_DIN2_GPIO_Port GPIOC
#define MCU_DTR_Pin GPIO_PIN_12
#define MCU_DTR_GPIO_Port GPIOC
#define MCU_RST_2MDM_Pin GPIO_PIN_2
#define MCU_RST_2MDM_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define MCU_MISO1_Pin GPIO_PIN_4
#define MCU_MISO1_GPIO_Port GPIOB
#define MCU_MOSI1_Pin GPIO_PIN_5
#define MCU_MOSI1_GPIO_Port GPIOB
#define ADP_BAT_DET_Pin GPIO_PIN_6
#define ADP_BAT_DET_GPIO_Port GPIOB
#define BLE_RTS_Pin GPIO_PIN_7
#define BLE_RTS_GPIO_Port GPIOB
#define BOOT_Pin GPIO_PIN_3
#define BOOT_GPIO_Port GPIOH
#define MCU_PB0_Pin GPIO_PIN_8
#define MCU_PB0_GPIO_Port GPIOB
#define MCU_PB1_Pin GPIO_PIN_9
#define MCU_PB1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
