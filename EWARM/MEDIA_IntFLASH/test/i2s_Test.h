/*
 * I2S_Test.h
 *
 *  Created on: Mar 27, 2012
 *      Author: rkd_1
 */

#ifndef I2S_TEST_H_
#define I2S_TEST_H_

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"
#include "pdm_filter.h"
#include "arm_math.h"
#include "math_helper.h"

//#define NO_FILTER
#define FIR_FILTER

#define PDM_MAX		64   // MAXIMUM value of pdm
#define PCM_MAX		16   // MAXIMUM value of pcm
#define FRAME_MAX	6400 // MAXIMUM value of frame

/* I2S에 사용될 peripheral 설정 */
void I2S_Port_Setting(GPIO_InitTypeDef *GPIO_InitStructure);
void SPI_I2S_Setting(SPI_TypeDef *SPIx, I2S_InitTypeDef *I2S_InitStructure);
void SPIx_NVIC_Setting(NVIC_InitTypeDef *NVIC_InitStructure);

/* I2S 설정 */
void I2S_Configure(SPI_TypeDef *SPIx, I2S_InitTypeDef *I2S_InitStructure, NVIC_InitTypeDef *NVIC_InitStructure);

void SPI2_IRQHandler(void);

#endif /* I2S_TEST_H_ */
