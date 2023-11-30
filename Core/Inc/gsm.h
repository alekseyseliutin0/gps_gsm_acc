/*
 * gsm.h
 *
 *  Created on: Nov 29, 2023
 *      Author: Alex
 */

#ifndef INC_GSM_H_
#define INC_GSM_H_
#include <stdint.h>
#include <stdbool.h>


#define PWR_GSM_ON  HAL_GPIO_WritePin(GPIOC, PWR_GSM_Pin, GPIO_PIN_SET)
#define PWR_GSM_OFF HAL_GPIO_WritePin(GPIOC, PWR_GSM_Pin, GPIO_PIN_RESET)
#define KEY_GSM_ON  HAL_GPIO_WritePin(GPIOC, PWR_KEY_Pin, GPIO_PIN_SET)
#define KEY_GSM_OFF HAL_GPIO_WritePin(GPIOC, PWR_KEY_Pin, GPIO_PIN_RESET)

void gsm(void);

#endif /* INC_GSM_H_ */
