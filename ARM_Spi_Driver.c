
//MCU specific Header file for stm32f446RE Nucleo board.
#include <stdio.h>
#include "stm32f446xx.h"
#include "stdint.h"
#include "ARM_SPI_DRIVER.h" 
#include "Arm_SpiDef.h"

/************************************************************************************************************ 
@Brief Enable the SPI device
@Param SPI_Config	  : SPI base address
@retval None
************************************************************************************************************/
static void SPI_EnableApi(SPI_TypeDef *SPI_Config)
{
	  if(!(SPI_Config->CR1 & SPI_REG_CR1_SPE))
		{
			SPI_Config->CR1|=SPI_REG_CR1_SPE;
		}
}

/************************************************************************************************************ 
@Brief Disaable the SPI device
@Param SPI_Config	  : SPI base address
@retval None
************************************************************************************************************/
static void SPI_DisableApi(SPI_TypeDef *SPI_Config)
{
		SPI_Config->CR1 &= ~SPI_REG_CR1_SPE;
}

/************************************************************************************************************ 
@Brief Configure the phase and polarity of SPI

@Param SPI_Config	  : SPI base address
@Param SPI_Polarity : SPI polarity
@Param SPI_Phase : SPI Phase
@retval None
************************************************************************************************************/
static void SPI_Configure_PhasePolrity(SPI_TypeDef *SPI_Config,uint32_t SPI_Polarity,uint32_t SPI_Phase)
{
	if(SPI_Polarity==SPI_1_CPOL)
	{
		SPI_Config->CR1 |=SPI_REG_CR1_CPOL;
	}
	else if(SPI_Polarity==SPI_0_CPOL)
	{
		SPI_Config->CR1 &= ~SPI_REG_CR1_CPOL;
	}
	
	
	if(SPI_Phase==SPI_REG_CR1_1_CPHASE)
	{
		SPI_Config->CR1 |=SPI_REG_CR1_1_CPHASE;
	}
	else if(SPI_Phase==SPI_REG_CR1_0_CPHASE)
	{
		SPI_Config->CR1 &= ~SPI_REG_CR1_1_CPHASE;
	}
}
/************************************************************************************************************ 
	@Brief Configure the SPI device mode
@Param SPI_Config : Base address of spi
@Param DeviceMode : SPI device mode 0-> slave mode, 1 -> master mode
@retval None
										
************************************************************************************************************/
static void SPI_Configure_DeviceMode(SPI_TypeDef *SPI_Config,uint32_t DeviceMode)
{
	if(DeviceMode == SPI_MASTER_CONFIG)
	{
		SPI_Config->CR1|=SPI_REG_CR1_MSTR;
	}
	else if(DeviceMode ==SPI_SLAVE_CONFIG)
	{
		SPI_Config->CR1&= ~SPI_REG_CR1_MSTR;
	}
	
}
/************************************************************************************************************ 

@Brief Configure the SPI Data size
@Param SPI_Config : Base address of spi
@Param DeviceMode : SPI data size 0-> SPI_8BIT_DATAFRME, 1 -> SPI_16BIT_DATAFRME
@retval None
************************************************************************************************************/
static void SPI_Configure_DataSize(SPI_TypeDef *SPI_Config,uint32_t Data_size)
{
	if(Data_size)
	{
		SPI_Config->CR1|=SPI_REG_CR1_DFF;
	}
	else 
	{
		SPI_Config->CR1&= ~SPI_REG_CR1_DFF;
	}
	
}
/************************************************************************************************************ 
@brief Configure the SPI NSS at Master side 
@Param SPI_handle_t : Base address of the SPI
@param NSS_Enable : Used to configure software or hardware slave management( softwre(1) or hrdware(0) slave select mangement)
                    if software slave select management is selected then the slave select pin in slave is force to low using softare.
										slave select line master is high.
@retval None
************************************************************************************************************/
static void SPI_Configure_Nss_Master(SPI_TypeDef *SPI_Config,uint32_t NSS_Enable)
{
	if(NSS_Enable)// software slave select management
	{
		SPI_Config->CR1 |=SPI_REG_CR1_SSM;
		SPI_Config->CR1 |=SPI_REG_CR1_SSI; 
	}
	else{
		SPI_Config->CR1 &= ~SPI_REG_CR1_SSM;
	}
}
/************************************************************************************************************ 
@brief Configure the SPI NSS at slave side 
@Param SPI_handle_t : Base address of the SPI
@param NSS_Enable : Used to configure software or hardware slave management( softwre(1) or hrdware(0) slave select mangement)
                    if software slave select management is selected then the slave select pin in slave is force to low using softare.
@retval None
************************************************************************************************************/
static void SPI_Configure_Nss_Slave(SPI_TypeDef *SPI_Config,uint32_t NSS_Enable)
{
		if(NSS_Enable)// software slave select management
		{
			
			SPI_Config->CR1 |= SPI_REG_CR1_SSM;
		}
		else   //// Hardware slave select management
		{
			
			SPI_Config->CR1 &= ~SPI_REG_CR1_SSM;
		}
}
/************************************************************************************************************ 


@Brief Configure the SPI device speed
@Param SPI_Config : Base address of spi
@Param DeviceMode : SPI device speed 
@retval None
************************************************************************************************************/
static void SPI_Configure_DeviceSpeed(SPI_TypeDef *SPI_Config,uint32_t SPI_speed)
{
	SPI_Config->CR1|=SPI_REG_CR1_BAUDRTE_CR;
}
/************************************************************************************************************ 


@Brief Configure the  SPI device direction
@Param SPI_Config : Base address of spi
@Param DeviceMode : SPI device direction  0-> Msb first, 1 -> LSB first
@retval None
************************************************************************************************************/
static void SPI_Configure_device_direction(SPI_TypeDef *SPI_Config,uint32_t LSB_first)
{
	 if(LSB_first)
	 {
			SPI_Config->CR1|=SPI_REG_CR1_LSBFIRST;
	 }
	 else 
	 {
		 
		 SPI_Config->CR1&= ~SPI_REG_CR1_LSBFIRST;
	 }
}
/************************************************************************************************************
	@Brief SPI_init api used to initializ the SPI device
	@Param SPI_handle_t : Base address of the SPI
  @retval None

************************************************************************************************************/
void SPI_init(SPI_handle_init *SPI_handle_t)
{
	/******* Configure the phase and polarity of SPI*************/
	 SPI_Configure_PhasePolrity(SPI_handle_t->SPI_Config,SPI_handle_t->SPI_Init->SPI_ClkPhase, SPI_handle_t->SPI_Init->SPI_ClkPhase);
	/******* Configure the SPI device mode*************/
	 SPI_Configure_DeviceMode(SPI_handle_t->SPI_Config,SPI_handle_t->SPI_Init->SPI_OperatingMode);
	/******* Configure the SPI Data size *************/
   SPI_Configure_DataSize(SPI_handle_t->SPI_Config,SPI_handle_t->SPI_Init->SPI_DataSize);
	/******* Configure the SPI slave select line*************/
	 if(SPI_handle_t->SPI_Init->SPI_OperatingMode==SPI_MASTER_CONFIG)
	 {
	   SPI_Configure_Nss_Master(SPI_handle_t->SPI_Config,SPI_handle_t->SPI_Init->SPI_NSS);
	 }
	 else
	 {
	    SPI_Configure_Nss_Slave(SPI_handle_t->SPI_Config,SPI_handle_t->SPI_Init->SPI_NSS);	
	 }
		 /******* Configure the SPI device speed*************/
	 SPI_Configure_DeviceSpeed(SPI_handle_t->SPI_Config,SPI_handle_t->SPI_Init->BaudratePreScalar);
	/******* Configure the  SPI device direction*************/
	 SPI_Configure_device_direction(SPI_handle_t->SPI_Config,SPI_handle_t->SPI_Init->SPI_Direction);
	 
	 SPI_EnableApi(SPI_handle_t->SPI_Config);
}
void hal_spi_enable_txe_interrupt(SPI_TypeDef *SPI_Config)
{
	SPI_Config->CR2|=SPI_REG_CR2_TXEIE;
}

void hal_spi_Disable_txe_interrupt(SPI_TypeDef *SPI_Config)
{
	SPI_Config->CR2&=~SPI_REG_CR2_TXEIE;
}
/************************************************************************************************************
	@Brief SPI_Master_TX api used to do master data transmission
	@Param SPI_handle_t : Base address of the SPI
	@Param *TX_Buffer   : Pointer to the tx buffer
	@Param	len         : Length of tx data
  @retval None

************************************************************************************************************/

void SPI_Master_TX(SPI_handle_init *SPI_handle_t,uint8_t *TX_Buffer,uint32_t len)
{
	SPI_handle_t->pTX_BufPtr=TX_Buffer;
	SPI_handle_t->TX_Xfer_Count =len;
	SPI_handle_t->TX_Xfer_Size=len;
	SPI_handle_t->SPI_state =HAL_SPI_STATE_BUSY_TX;
	SPI_EnableApi(SPI_handle_t->SPI_Config);
	
	hal_spi_enable_txe_interrupt(SPI_handle_t->SPI_Config);
	
}



void hal_spi_enable_rxne_interrupt(SPI_TypeDef *SPI_Config)
{
	
	SPI_Config->CR2|=SPI_REG_CR2_RXNEIE;
	
}

void hal_spi_disable_rxne_interrupt(SPI_TypeDef *SPI_Config)
{
	
	SPI_Config->CR2&=~SPI_REG_CR2_RXNEIE;
	
}
/************************************************************************************************************
@Brief SPI_Slave_RX api used to do slave data reception
	@Param SPI_handle_t : Base address of the SPI
	@Param *RX_Buffer   : Pointer to the Rx buffer
	@Param	len         : Length of Rx data
  @retval None
************************************************************************************************************/
void SPI_Slave_RX(SPI_handle_init *SPI_handle_t,uint8_t *RX_Buffer,uint32_t len)
{
	SPI_handle_t->pTX_BufPtr=RX_Buffer;
	SPI_handle_t->TX_Xfer_Count =len;
	SPI_handle_t->TX_Xfer_Size=len;
	SPI_handle_t->SPI_state =HAL_SPI_STATE_BUSY_RX;
	SPI_EnableApi(SPI_handle_t->SPI_Config);
	
	hal_spi_enable_rxne_interrupt(SPI_handle_t->SPI_Config);
	
}
/************************************************************************************************************
	@Brief SPI_Master_RX api used to do master data reception
	@Param SPI_handle_t : Base address of the SPI
	@Param *RX_Buffer   : Pointer to the Rx buffer
	@Param	len         : Length of Rx data
  @retval None

************************************************************************************************************/
void SPI_Master_RX(SPI_handle_init *SPI_handle_t,uint8_t *RX_Buffer,uint32_t len)
{
	uint32_t val=0;
	/* this is the dummy tx data**/
	SPI_handle_t->pTX_BufPtr=RX_Buffer;
	SPI_handle_t->TX_Xfer_Count =len;
	SPI_handle_t->TX_Xfer_Size=len;
	/****data reaad from from Xed to RX buffer(COPY THE ADDRESS OF rx BUFFER TO rxED)***/
	SPI_handle_t->pRX_BufPtr=RX_Buffer;
	SPI_handle_t->RX_Xfer_Count =len;
	SPI_handle_t->RX_Xfer_Size=len;
	/***DRIVER IS BUSY IN rx***/
	SPI_handle_t->SPI_state =HAL_SPI_STATE_BUSY_RX;
	/**ENABLE THE SPI**/
	SPI_EnableApi(SPI_handle_t->SPI_Config);
	
	/****BEFORE ENBLE THE RX INTERRUPT MKE SURE READ THE DATA REGSIER 
	FOR EMPTY AFTER THAT ENABLE THE RXNE OTHERWISE SPURIUS INTERRUPT OCCURE**/
	val=SPI_handle_t->SPI_Config->DR;
	
	/*****enble the TXe and RXne interrupt***********/
	hal_spi_enable_rxne_interrupt(SPI_handle_t->SPI_Config);
	
	hal_spi_enable_txe_interrupt(SPI_handle_t->SPI_Config);
	
	
}
/************************************************************************************************************
	@Brief SPI_Slave_TX api used to do slave data trasmission
	@Param SPI_handle_t : Base address of the SPI
	@Param *TX_Buffer   : Pointer to the Tx buffer
	@Param	len         : Length of Tx data
  @retval None

************************************************************************************************************/
void SPI_Slave_TX(SPI_handle_init *SPI_handle_t,uint8_t *TX_Buffer,uint32_t len)
{
	
	/* Popultes the pointers and length informtion to tx data**/
	SPI_handle_t->pTX_BufPtr=TX_Buffer;
	SPI_handle_t->TX_Xfer_Count =len;
	SPI_handle_t->TX_Xfer_Size=len;
	/****data reaad from from Xed to RX buffer(COPY THE ADDRESS OF rx BUFFER TO rxED)***/
	SPI_handle_t->pRX_BufPtr=TX_Buffer;
	SPI_handle_t->RX_Xfer_Count =len;
	SPI_handle_t->RX_Xfer_Size=len;
	/***DRIVER IS BUSY IN Tx***/
	SPI_handle_t->SPI_state =HAL_SPI_STATE_BUSY_TX;
	/**ENABLE THE SPI**/
	SPI_EnableApi(SPI_handle_t->SPI_Config);
	
	
	
	/*****enble the TXe and RXne interrupt***********/
	hal_spi_enable_rxne_interrupt(SPI_handle_t->SPI_Config);
	
	hal_spi_enable_txe_interrupt(SPI_handle_t->SPI_Config);
	
}
/************************************************************************************************************
	@Brief SPI_interrupt_handler api used to handle the SPI Interrupt
	@Param SPI_handle_t : Base address of the SPI
  @retval None

************************************************************************************************************/
void SPI_interrupt_handler(SPI_handle_init *SPI_handle_t)
{
	 uint32_t Temp1=0,Temp2=0;
	 /***************** RX Interrupt ******************************************/ 
	 Temp1=(SPI_handle_t->SPI_Config->SR&SPI_REG_SR_RXNE); // check the RXNE(RX interrupt occure or not)
	 Temp2=(SPI_handle_t->SPI_Config->CR2&SPI_REG_CR2_RXNEIE); //check the RXNEIE bit enable or not
	 if((Temp1!=RESET)&&(Temp2!=RESET))
	 {
		 Hal_SPI_RX_Interrupt_Handler(SPI_handle_t);
	 }
	
	  /***************** TX Interrupt ******************************************/ 
	 Temp1=(SPI_handle_t->SPI_Config->SR&SPI_REG_SR_TXE); // check the TXE(TX interrupt occure or not)
	 Temp2=(SPI_handle_t->SPI_Config->CR2&SPI_REG_CR2_TXEIE); //check the TXEIE bit enable or not
	 if((Temp1!=RESET)&&(Temp2!=RESET))
	 {
		 Hal_SPI_TX_Interrupt_Handler(SPI_handle_t);
	 }
}

 /************************************************************************************************************
	@Brief Hal_spi_TX_Intrrupt_close api used to close the SPI TX Interrupt
	@Param SPI_handle_t : pointer to SPI_handle_t structure contains configure informtion of SPI device
  @retval None

************************************************************************************************************/
static void Hal_spi_TX_Intrrupt_close(SPI_handle_init *SPI_handle_t)
{
	
	hal_spi_enable_txe_interrupt(SPI_handle_t->SPI_Config);
	if(SPI_handle_t->SPI_Init->SPI_OperatingMode&&(SPI_handle_t->SPI_state!=HAL_SPI_STATE_BUSY_RX))
	{
		SPI_handle_t->SPI_state=HAL_SPI_STATE_READY;
	}
}

 /************************************************************************************************************
	@Brief Hal_spi_TX_Intrrupt_close api used to close the SPI TX Interrupt
	@Param SPI_handle_t : pointer to SPI_handle_t structure contains configure informtion of SPI device
  @retval None

************************************************************************************************************/
static void Hal_spi_RX_Intrrupt_close(SPI_handle_init *SPI_handle_t)
{
	while(SPI_handle_t->SPI_state==HAL_SPI_STATE_BUSY_RX);
		
	hal_spi_enable_rxne_interrupt(SPI_handle_t->SPI_Config);
	SPI_handle_t->SPI_state=HAL_SPI_STATE_READY;
	
}
 /************************************************************************************************************
	@Brief Hal_SPI_RX_interrupt_handler api used to handle the SPI RX Interrupt
	@Param SPI_handle_t : pointer to SPI_handle_t structure contains configure informtion of SPI device
  @retval None

************************************************************************************************************/
void Hal_SPI_RX_Interrupt_Handler(SPI_handle_init *SPI_handle_t)
{
	 if(SPI_handle_t->SPI_Init->SPI_DataSize==SPI_8BIT_DATAFRME)
	{
		 (*SPI_handle_t->pRX_BufPtr++) =SPI_handle_t->SPI_Config->DR;
		 SPI_handle_t->RX_Xfer_Count--;
		
	}
	else if(SPI_handle_t->SPI_Init->SPI_DataSize==SPI_16BIT_DATAFRME)
	{
		
		*((uint16_t*)SPI_handle_t->pTX_BufPtr)=SPI_handle_t->SPI_Config->DR;
		
		SPI_handle_t->pRX_BufPtr +=2;
		SPI_handle_t->RX_Xfer_Count-=2;
		
	}
	
	if(SPI_handle_t->RX_Xfer_Count==0)
	{
		Hal_spi_RX_Intrrupt_close(SPI_handle_t);
	}
	
	
}

/************************************************************************************************************
	@Brief Hal_SPI_TX_interrupt_handler api used to handle the SPI TX Interrupt
	@Param SPI_handle_t : pointer to SPI_handle_t structure contains configure informtion of SPI device
  @retval None

************************************************************************************************************/
void Hal_SPI_TX_Interrupt_Handler(SPI_handle_init *SPI_handle_t)
{
	if(SPI_handle_t->SPI_Init->SPI_DataSize==SPI_8BIT_DATAFRME)
	{
		 SPI_handle_t->SPI_Config->DR= *(SPI_handle_t->pTX_BufPtr++);
		 SPI_handle_t->TX_Xfer_Count--;
		
	}
	else if(SPI_handle_t->SPI_Init->SPI_DataSize==SPI_16BIT_DATAFRME)
	{
		
		SPI_handle_t->SPI_Config->DR= *((uint16_t*)SPI_handle_t->pTX_BufPtr);
		SPI_handle_t->pTX_BufPtr +=2;
		SPI_handle_t->TX_Xfer_Count-=2;
	}
	
	if(SPI_handle_t->TX_Xfer_Count==0)
	{
		/********************we reach end of trasmmission then close the interrupt************/
		
		 Hal_spi_TX_Intrrupt_close(SPI_handle_t);
	}
}


