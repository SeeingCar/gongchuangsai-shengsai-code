/* USER CODE BEGIN Header */
/* can.h —— CAN1 句柄与引脚选择宏。 */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
/* CAN1 引脚组（均 AF9，收发器 D/R 只焊在其中一组）：0 = PB8/PB9，1 = PD0/PD1（C 板实测），2 = PA11/PA12。 ⚠ 接错则帧发不上总线（LEC=5、一帧收不到）；不确定就扫一遍，LEC=3 那组是接对的。 */
#define CAN1_PINS   0
/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

