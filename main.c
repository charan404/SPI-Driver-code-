#include <stdio.h>
#include "stdint.h"
#include "ARM_SPI_DRIVER.h"
#include "GPIO_Driver.h" 
#include "SPI_Master.h"

	SPI_handle_init *SPI_handle_t; 
int main()
{
	uint8_t Addrcmd[2];
	/*******enable the GPIO clock***************/
  GPIO_ClockConfig(GPIOPORT_B);
	/*******enable the SPI clock***************/
	SPI_ClockConfig(SPI_2);
	/*******Initialize the GPIO's for clk MISO MOSI***************/
	SPI_GPIOS_Initialization();
		/*******Initialize the SPI's for master***************/
	SPI_Parameter_init(SPI_handle_t,SPI2,SPI_MASTER_CONFIG,SPI_FPCLK_DIV_2,SPI_0_CPOL,SPI_REG_CR1_0_CPHASE,SPI_8BIT_DATAFRME
	,SPI_2_LINE_ENBLE_UNIDIR,SPI_DATA_FRAME_MSB_FIRST,SPI_SOFT_SLAVE_MNG_ENABLE);
	
	/*******Initialize the SPI's for slave***************/
	//SPI_Parameter_init(SPI_handle_t,SPI2,SPI_SLAVE_CONFIG,SPI_FPCLK_DIV_2,SPI_0_CPOL,SPI_REG_CR1_0_CPHASE,SPI_8BIT_DATAFRME
	//,SPI_2_LINE_ENBLE_UNIDIR,SPI_DATA_FRAME_MSB_FIRST,SPI_SOFT_SLAVE_MNG_ENABLE);
	
	/*******Enable the SPI's IRQ***************/
	NVIC_EnableIRQ(SPI2_IRQn);
	
	
	while(SPI_handle_t->SPI_state!=HAL_SPI_STATE_READY);
	Addrcmd[0]=0X12;
	Addrcmd[1]=0X34;
	/*****************spi master send **********************/
  SPI_Master_TX(SPI_handle_t,Addrcmd,2);
	/*****************spi slave Receives **********************/
  //SPI_Slave_RX(SPI_handle_t,Addrcmd,2);
	
	while(SPI_handle_t->SPI_state!=HAL_SPI_STATE_READY);
	/*****************spi master Receives **********************/
	SPI_Master_RX(SPI_handle_t,Addrcmd,2);
	
		/*****************spi slave send **********************/
	//SPI_Slave_TX(SPI_handle_t,Addrcmd,2);
	return 0;
}
