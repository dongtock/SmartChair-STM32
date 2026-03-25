/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
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
#include "stm32f4xx_hal.h"

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
#define HX_SCK_Pin GPIO_PIN_0
#define HX_SCK_GPIO_Port GPIOA
#define DOUT_1_Pin GPIO_PIN_1
#define DOUT_1_GPIO_Port GPIOA
#define DOUT_2_Pin GPIO_PIN_2
#define DOUT_2_GPIO_Port GPIOA
#define DOUT_3_Pin GPIO_PIN_3
#define DOUT_3_GPIO_Port GPIOA
#define TOF3D_1_NCS_Pin GPIO_PIN_4
#define TOF3D_1_NCS_GPIO_Port GPIOA
#define TOF3D_1_LPN_Pin GPIO_PIN_0
#define TOF3D_1_LPN_GPIO_Port GPIOB
#define TOF3D_1_PWREN_Pin GPIO_PIN_1
#define TOF3D_1_PWREN_GPIO_Port GPIOB
#define TOF3D_2_NCS_Pin GPIO_PIN_2
#define TOF3D_2_NCS_GPIO_Port GPIOB
#define DOUT_11_Pin GPIO_PIN_10
#define DOUT_11_GPIO_Port GPIOB
#define DOUT_12_Pin GPIO_PIN_12
#define DOUT_12_GPIO_Port GPIOB
#define DOUT_7_Pin GPIO_PIN_15
#define DOUT_7_GPIO_Port GPIOB
#define DOUT_4_Pin GPIO_PIN_8
#define DOUT_4_GPIO_Port GPIOA
#define DOUT_5_Pin GPIO_PIN_11
#define DOUT_5_GPIO_Port GPIOA
#define DOUT_6_Pin GPIO_PIN_12
#define DOUT_6_GPIO_Port GPIOA
#define TOF3D_2_LPN_Pin GPIO_PIN_3
#define TOF3D_2_LPN_GPIO_Port GPIOB
#define TOF3D_2_PWREN_Pin GPIO_PIN_4
#define TOF3D_2_PWREN_GPIO_Port GPIOB
#define DOUT_8_Pin GPIO_PIN_5
#define DOUT_8_GPIO_Port GPIOB
#define DOUT_9_Pin GPIO_PIN_8
#define DOUT_9_GPIO_Port GPIOB
#define DOUT_10_Pin GPIO_PIN_9
#define DOUT_10_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
