/*
 * conf_spi.c
 *
 *  Created on: May 19, 2020
 *      Author: test
 */

#include "conf_spi.h"
void SPIx_NVIC(SPI_HandleTypeDef* hspi);



void MX_SPIx_Init(SPI_TypeDef * SPI)
{
	if(SPI == SPI2){
	  hspi1.Instance = SPI;
	  hspi1.Init.Mode = SPI_MODE_MASTER;
	  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
	  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
	  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
	  hspi1.Init.CRCPolynomial = 7;
	  if (HAL_SPI_Init(&hspi1) != HAL_OK){
	    Error_Handler();
	  }
	}
}


void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  SPIx_NVIC(hspi);
  if(hspi->Instance==SPI1){
    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA4     ------> SPI1_NSS
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}


void SPIx_NVIC(SPI_HandleTypeDef* hspi)
{
	if(hspi->Instance==SPI1){
    HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
	}
}
