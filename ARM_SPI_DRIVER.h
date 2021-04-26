#ifndef 	__ARM_SPI_DRIVER_H__
#define 	__ARM_SPI_DRIVER_H__
//MCU specific Header file for stm32f446RE Nucleo board.
#include "stm32f446xx.h"
#include "stdint.h"
/***********************************************************************************************************************
															Serial peripheral interface
															Register Bit definations
***********************************************************************************************************************/
/**************************** SPI conrol register bit definations***************************************************/
#define SPI_REG_CR1_BIDIMODE		((uint32_t)1<<15)
#define SPI_2_LINE_ENBLE_UNIDIR 0
#define SPI_1_LINE_ENBLE_BIDIR	1


#define SPI_REG_CR1_BIDIOE					((uint32_t)1<<14)
#define SPI_OUTPUT_DISABLED					0
#define SPI_OUTPUT_ENABLE						1


#define SPI_REG_CR1_CRCEN						((uint32_t)1<<13)
#define SPI_CRC_CALCULATION_DISABLE	0
#define SPI_CRC_CALCULATION_ENABLE	1

#define SPI_REG_CR1_CRCNEXT				  ((uint32_t)1<<12)
#define SPI_NO_CRC_PHASE						0
#define SPI_CRC_PHASE								1
		
#define SPI_REG_CR1_DFF							((uint32_t)1<<11)
#define	SPI_8BIT_DATAFRME						0
#define	SPI_16BIT_DATAFRME					1
		
#define	SPI_REG_CR1_RXONLY					((uint32_t)1<<10)
#define	SPI_FULLDUPLEX  						0
#define	SPI_OUTPUT_DISABLE					1
		
#define	SPI_REG_CR1_SSM							((uint32_t)1<<9)
#define	SPI_SOFT_SLAVE_MNG_ENABLE		1
#define	SPI_SOFT_SLAVE_MNG_DISABLE	0

#define	SPI_REG_CR1_SSI							((uint32_t)1<<8)

#define	SPI_REG_CR1_LSBFIRST				((uint32_t)1<<7)
#define	SPI_DATA_FRAME_MSB_FIRST		0
#define	SPI_DATA_FRAME_LSB_FIRST		1

#define	SPI_REG_CR1_SPE							((uint32_t)1<<6)
#define	SPI_PERIPHERAL_DISABLE			0
#define	SPI_PERIPHERAL_ENABLE				1

#define	SPI_REG_CR1_BAUDRTE_CR			((uint32_t)7<<3)
#define	SPI_FPCLK_DIV_2							0
#define	SPI_FPCLK_DIV_4							1
#define	SPI_FPCLK_DIV_8							2
#define	SPI_FPCLK_DIV_16							3
#define	SPI_FPCLK_DIV_32							4
#define	SPI_FPCLK_DIV_64							5
#define	SPI_FPCLK_DIV_128							6
#define	SPI_FPCLK_DIV_256							7

#define	SPI_REG_CR1_MSTR							((uint32_t)1<<2)
#define	SPI_SLAVE_CONFIG							0
#define	SPI_MASTER_CONFIG							1

#define	SPI_REG_CR1_CPOL							((uint32_t)1<<1)
#define	SPI_0_CPOL										0
#define	SPI_1_CPOL										1


#define	SPI_REG_CR1_0_CPHASE					0
#define	SPI_REG_CR1_1_CPHASE					1

/**************************** SPI conrol register 2 bit definations***************************************************/
#define	SPI_REG_CR2_TXEIE							((uint32_t)1<<7)
#define	SPI_REG_CR2_RXNEIE						((uint32_t)1<<6)
#define	SPI_REG_CR2_ERRIE							((uint32_t)1<<5)

#define	SPI_REG_CR2_FRF								((uint32_t)1<<4)
#define	SPI_MOTOROLA_MODE							0
#define	SPI_TI_MODE										1

#define	SPI_REG_CR2_SSOE							((uint32_t)1<<2)


/**************************** SPI status register  bit definations***************************************************/
#define	SPI_REG_SR_FRE							((uint32_t)1<<8)
#define	SPI_REG_SR_BSY							((uint32_t)1<<7)
#define	SPI_REG_SR_OVR							((uint32_t)1<<6)
#define	SPI_REG_SR_MODF							((uint32_t)	1<<5)
#define	SPI_REG_SR_TXE							((uint32_t)	1<<1)
#define	SPI_REG_SR_RXNE							((uint32_t)	1<<0)

#define SPI_BUSY_FLG								1
#define SPI_NOT_BUSY_FLG						0	
#define	SET							1
#define RESET	!SET

/************************** SPI struct*****************************************************************************/

#define SPI1_INIT	SPI1
#define SPI2_INIT	SPI2
#define SPI3_INIT	SPI3
#define SPI4_INIT	SPI4
/************************** Enables the clock for spi****************************/

#define HAL_RCC_SPI1_CL0CK_ENABLE()			(RCC->APB2ENR|=(1<<12))	
#define HAL_RCC_SPI2_CL0CK_ENABLE()			(RCC->APB1ENR|=(1<<14))	
#define HAL_RCC_SPI3_CL0CK_ENABLE()			(RCC->APB1ENR|=(1<<15))
#define HAL_RCC_SPI4_CL0CK_ENABLE()			(RCC->APB2ENR|=(1<<13))	

/**************************************************************************************************************

 @ SPI Data structure used by the  SPI driver

**************************************************************************************************************/

/************** SPI state structure defination****************************/
 typedef enum{
	
	 HAL_SPI_STATE_RESET       = 0X00, // SPI not yet initialized or diable 
	 HAL_SPI_STATE_READY	     = 0X01, // SPI is initialized and ready for used
	 HAL_SPI_STATE_BUSY	       = 0X02, // SPI process is ongoing
	 HAL_SPI_STATE_BUSY_TX	   = 0X12, // SPI Data transmission process is ongoing
	 HAL_SPI_STATE_BUSY_RX	   = 0X22, // SPI Data Reception process is ongoing
	 HAL_SPI_STATE_BUSY_TX_RX	 = 0X32, // SPI Data transmission and Reception process is ongoing
	 HAL_SPI_STATE_ERROR	     = 0X03  // SPI error state
	 
 }Hal_SPI_state_t;	 
/****************************SPI confiuration structure defination******************************************/
 
typedef struct
{	 
	uint32_t  SPI_OperatingMode; // SPI operating mode master or slave
	uint32_t  SPI_Direction;     // Specifies the SPI direction mode //1. one line bi-directional 2. two line uni direction mode
	uint32_t  SPI_DataSize;      // Specifies the SPI data size	// 8 or 16 bit data transmission 
	uint32_t  SPI_ClkPolarity;   // Specifies the SPI clock formate to be used by the spi 
	uint32_t  SPI_ClkPhase;			// Specifies the SPI clcok active edge for the bit capture
	uint32_t  SPI_NSS;	          // Specifies the SPI Hardware slave mangement or software slave mangement using SSI bit
	uint32_t	 BaudratePreScalar; // Specifies the SPI used to configure the trasmit and receiv sck clock.
	uint32_t  SPI_FirstBit;      // Specifies the SPI data transfer from MSB or LSB
	 
}SPI_Config_def;

/***************************SPI handle structure defination***********************************************/
typedef struct
{
	
	SPI_TypeDef    *SPI_Config;				// SPI register Base Address
	SPI_Config_def *SPI_Init;     // SPI communication parameters
	uint8_t			   *pTX_BufPtr;     // pointer to SPI TX
	uint16_t			 TX_Xfer_Size;    // SPI TX transfer size
	uint16_t			 TX_Xfer_Count;   // SPI TX transfer count
	uint8_t			   *pRX_BufPtr;     // pointer to SPI RX
	uint16_t			 RX_Xfer_Size;    // SPI RX transfer size
	uint16_t			 RX_Xfer_Count;   // SPI RX transfer count
	Hal_SPI_state_t SPI_state;      // SPI communication state 
	
}SPI_handle_init;
 
 /******************************************************************************************************************

																								SPI DRIVER API'S

*******************************************************************************************************************/
 
void SPI_init(SPI_handle_init *SPI_handle_t);

void SPI_Master_TX(SPI_handle_init *SPI_handle_t,uint8_t *TX_Buffer,uint32_t len);

void SPI_Slave_RX(SPI_handle_init *SPI_handle_t,uint8_t *RX_Buffer,uint32_t len);

void SPI_Master_RX(SPI_handle_init *SPI_handle_t,uint8_t *RX_Buffer,uint32_t len);

void SPI_Slave_TX(SPI_handle_init *SPI_handle_t,uint8_t *TX_Buffer,uint32_t len);

void SPI_interrupt_handler(SPI_handle_init *SPI_handle_t);

static void SPI_Configure_DataSize(SPI_TypeDef *SPI_Config,uint32_t Data_size);

void Hal_SPI_TX_Interrupt_Handler(SPI_handle_init *SPI_handle_t);

void Hal_SPI_RX_Interrupt_Handler(SPI_handle_init *SPI_handle_t);

void SPI_GPIOS_Initialization();  
static void SPI_Configure_DeviceMode(SPI_TypeDef *SPI_Config,uint32_t DeviceMode);
static void SPI_Configure_PhasePolrity(SPI_TypeDef *SPI_Config,uint32_t SPI_Polarity,uint32_t SPI_Phase);
	static void SPI_Configure_DataSize(SPI_TypeDef *SPI_Config,uint32_t Data_size);
	static void SPI_Configure_Nss_Master(SPI_TypeDef *SPI_Config,uint32_t NSS_Enable);
	static void SPI_Configure_Nss_Slave(SPI_TypeDef *SPI_Config,uint32_t NSS_Enable);
	static void SPI_Configure_DeviceSpeed(SPI_TypeDef *SPI_Config,uint32_t SPI_speed);
	static void SPI_Configure_device_direction(SPI_TypeDef *SPI_Config,uint32_t LSB_first);
		void SPI_Parameter_init(SPI_handle_init *SPI_handle_t,SPI_TypeDef *SPIConfig,uint32_t Mode,uint32_t baudrt,
	uint32_t cphase,uint32_t copl,uint32_t Data_size,uint32_t Direction,uint32_t LSB_MSB,uint32_t slave_select);
	void SPI_ClockConfig(uint32_t SPI_NO);
#endif//__ARM_SPI_DRIVER_H__