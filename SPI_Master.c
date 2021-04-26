#include <stdio.h>
#include "stm32f446xx.h"
#include "stdint.h"
#include "ARM_SPI_DRIVER.h"
#include "GPIO_Driver.h" 
#include "SPI_Master.h"
GPIO_TypeDef *gpio;
extern SPI_handle_init *SPI_handle_t; 
/************************************************************************************************************ 
@Brief SPI_GPIOS_Initialization api is used to set th MISO,MOSI,CLK pins

@retval None
************************************************************************************************************/
void SPI_GPIOS_Initialization()
{
	
	gpio = GPIOB;
	Gpio_pin_def GpioConfig,GpioConfig1,GpioConfig2;
	
	
	/******CLK pin Initialization******/
	GPIO_Initialization(gpio,&GpioConfig, GPIO_ALT_FUN_MODE, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_MEDIUM_SPEED,GPIO_PIN_AF5_SPI_1TO2,GPIO_PULL_DOWN,SPI_CLK_PIN);
  /******MISO pin Initialization******/	
	GPIO_Initialization(gpio,&GpioConfig1, GPIO_ALT_FUN_MODE, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_MEDIUM_SPEED,GPIO_PIN_AF5_SPI_1TO2,GPIO_PULL_UP,SPI_MISO_PIN);
	/******MOSI pin Initialization******/
	GPIO_Initialization(gpio,&GpioConfig2, GPIO_ALT_FUN_MODE, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_MEDIUM_SPEED,GPIO_PIN_AF5_SPI_1TO2,GPIO_PULL_UP,SPI_MOSI_PIN);
}
/************************************************************************************************************ 
@Brief SPI_Parameter_init api is used to set all parameters of spi

@retval None
************************************************************************************************************/
void SPI_Parameter_init(SPI_handle_init *SPI_handle_t,SPI_TypeDef *SPIConfig,uint32_t Mode,uint32_t baudrt,
	uint32_t cphase,uint32_t copl,uint32_t Data_size,uint32_t Direction,uint32_t LSB_MSB,uint32_t slave_select)
{
	SPI_handle_t->SPI_Config=SPIConfig;
	SPI_handle_t->SPI_Init->SPI_OperatingMode=Mode;
	SPI_handle_t->SPI_Init->BaudratePreScalar=baudrt;
	SPI_handle_t->SPI_Init->SPI_ClkPhase=cphase;
	SPI_handle_t->SPI_Init->SPI_ClkPolarity=copl;
	SPI_handle_t->SPI_Init->SPI_DataSize=Data_size;
	SPI_handle_t->SPI_Init->SPI_Direction=Direction;
	SPI_handle_t->SPI_Init->SPI_FirstBit=LSB_MSB;
	SPI_handle_t->SPI_Init->SPI_NSS=slave_select;
	
	SPI_handle_t->SPI_state=HAL_SPI_STATE_READY;
	
	SPI_init(SPI_handle_t); 
	
	
}
/*************************************************************************************************************
   SPI clock configuartion
************************************************************************************************************/
void SPI_ClockConfig(uint32_t SPI_NO)
{
	
	if(SPI_NO==SPI_1)
	{
		HAL_RCC_SPI1_CL0CK_ENABLE();
	}
	else if(SPI_NO==SPI_2)
	{
		HAL_RCC_SPI2_CL0CK_ENABLE();
		
	}
	else if(SPI_NO==SPI_3)
	{
		HAL_RCC_SPI3_CL0CK_ENABLE();
		
	}
	else if(SPI_NO==SPI_2)
	{
		HAL_RCC_SPI4_CL0CK_ENABLE();
		
	}
  
}
/*************************************************************************************
      Interrupt handler
*************************************************************************************/

void SPI2_IRQHandler(void)
{
	SPI_interrupt_handler(SPI_handle_t );
}