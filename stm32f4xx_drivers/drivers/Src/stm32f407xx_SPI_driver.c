/*
 * stm32f407xx_SPI_driver.c
 *
 *  Created on: Mar 22, 2023
 *      Author: jhern
 */

#include "stm32f407xx_SPI_driver.h"


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);


/*
 * SPI Peripheral Clock Setup
 */

//Enable or disables peripheral clock for a given SPI base address

/******************************************************************
 * @fn				-	SPI_PeriClockControl
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port.
 *
 * @param[in]		-	Base address of the GPIO peripheral via GPIO_RegDef_t pointer.
 * @param[in]		-	ENABLE or DISABLE macros.
 * @param[in]		-
 *
 * @return			-	None
 *
 * @Note			-	None

 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}else if (pSPIx == SPI2){
			SPI2_PCLK_EN();
		}else if (pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
	}else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}else if (pSPIx == SPI2){
			SPI2_PCLK_DI();
		}else if (pSPIx == SPI3){
			SPI3_PCLK_DI();

		}
	}
}

/*
 * SPI Initialization
 */

/******************************************************************
 * @fn				-	SPI_Peripheral Control
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port.
 *
 * @param[in]		-	Base address of the GPIO peripheral via GPIO_RegDef_t pointer.
 * @param[in]		-	ENABLE or DISABLE macros.
 * @param[in]		-
 *
 * @return			-	None
 *
 * @Note			-	None

 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx->CR1 |= ( 1 << SPI_CR1_SPE);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}




//User creates a SPI_Handle_t structure and sends a pointer to this function

/******************************************************************
 * @fn				-	SPI_Init
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port.
 *
 * @param[in]		-	Base address of the GPIO peripheral via GPIO_RegDef_t pointer.
 * @param[in]		-	ENABLE or DISABLE macros.
 * @param[in]		-
 *
 * @return			-	None
 *
 * @Note			-	None

 */
void SPI_Init(SPI_Handle_t *pSPIHandle){

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	//first lets configure the SPI_CR1 register


	uint32_t tempreg = 0;

	//1. configure the device mode
	tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	//2.Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SMP_RXONLY){
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//rxonly bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//3. Configure the serial clock speed
	tempreg |=	pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. Configure the data frame format
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;
	//5. Configure the CPOL value
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
	//6. Configure the CPHA value
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//7. Configure SSM mode
	if(pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_DI){
		tempreg &= ~(1 << SPI_CR1_SSM);
	}else if(pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_EN){
		tempreg |= (1 << SPI_CR1_SSM);
	}

	pSPIHandle->pSPIx->CR1 = tempreg; //we can use = instead of |= since we're freshly initializing the structures
}



/******************************************************************
 * @fn				-	SPI_Disable
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port.
 *
 * @param[in]		-	Base address of the GPIO peripheral via GPIO_RegDef_t pointer.
 * @param[in]		-	ENABLE or DISABLE macros.
 * @param[in]		-
 *
 * @return			-	None
 *
 * @Note			-	None

 */

void SPI_Disable(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1){
		SPI1_PCLK_DI();
	}else if(pSPIx == SPI2){
		SPI2_PCLK_DI();
	}else if(pSPIx == SPI3){
		SPI3_PCLK_DI();
	}
}
/*
 * Data Send and Receive,
 * Blocking (Non Interrupt Based), Non-Blocking (Interrupt Based)
 */


/******************************************************************
 * @fn				-	SPI_GetFlagStatus
 *
 * @brief			-	This function checks the value of a bit within the status register
 *
 * @param[in]		-	Base address of the SPI peripheral
 * @param[in]		-	Pointer variable to TxBuffer
 * @param[in]		-	Length of data to transmit
 *
 * @return			-	None
 *
 * @Note			-	None

 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/******************************************************************
 * @fn				-	SPI_SSIConfig
 *
 * @brief			-	This function checks the value of SSI bit within the status register
 *
 * @param[in]		-	Base address of the SPI peripheral
 * @param[in]		-	Pointer variable to TxBuffer
 * @param[in]		-	Length of data to transmit
 *
 * @return			-	None
 *
 * @Note			-	None

 */


void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/******************************************************************
 * @fn				-	SPI_SSOEConfig
 *
 * @brief			-	This function checks the value of the SSOE bit within the status register
 *
 * @param[in]		-	Base address of the SPI peripheral
 * @param[in]		-	Pointer variable to TxBuffer
 * @param[in]		-	Length of data to transmit
 *
 * @return			-	None
 *
 * @Note			-	None

 */


void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}





/******************************************************************
 * @fn				-	SPI_SendData
 *
 * @brief			-	This function also known as the blocking API will wait until all bytes are transmitted
 *
 * @param[in]		-	Base address of the SPI peripheral
 * @param[in]		-	Pointer variable to TxBuffer
 * @param[in]		-	Length of data to transmit in bytes (1, 2, etc)
 *
 * @return			-	None
 *
 * @Note			-	None

 */



//blocking API, will wait until all bytes are transmitted
//has potential that first while loop will block permanently
//and will require watchdog timer to reset
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
	while(Len > 0){
		//1. wait until TXE is set / polling for the TXE flag to SET
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);
		//2. check the DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF))){
			//16 bit DFF
			//1. Load the data in to the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer); //typecast to 16bit for 16 bit data transfer
			Len--; //subtracts length by 1, indicated 1 byte has been read
			Len--; //subtracts length by 1, indicating 2 total bytes have been read, 2 bytes = 16 bits
			(uint16_t*)pTxBuffer++; //increases Txbuffer memory address by 4 so that values are not overwritten
		}else{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/******************************************************************
 * @fn				-	SPI_ReceiveData
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port.
 *
 * @param[in]		-	Base address of the GPIO peripheral via GPIO_RegDef_t pointer.
 * @param[in]		-	ENABLE or DISABLE macros.
 * @param[in]		-
 *
 * @return			-	None
 *
 * @Note			-	None

 */



void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){
	while(Len > 0){
		//1. wait until RXE is full
		while((SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == (uint8_t) FLAG_RESET));
		//2. check the DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF))){
			//16 bit DFF
			//1. Read the data from DR to Rxbuffer address
			*((uint16_t*) pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else{
			//8 bit DFF
			*(pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}


/*
 * IRQ Configuration and ISR Handling
 */

/******************************************************************
 * @fn				-	SPI_IRQInterruptConfig
 *
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port.
 *
 * @param[in]		-	Base address of the GPIO peripheral via GPIO_RegDef_t pointer.
 * @param[in]		-	ENABLE or DISABLE macros.
 * @param[in]		-
 *
 * @return			-	None
 *
 * @Note			-	None

 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){ //used to configure IRQ number of the GPIO Pin
	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64){
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 64 && IRQNumber < 96){
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));
		}
	}else{ //disable the following IRQ numbers within the register
		if(IRQNumber <= 31){
			//program ISER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64){
			//program ISER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			//program ISER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}
	}
}
/******************************************************************
 * @fn				-	SPI_IRQPriorityConfig
 *
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port.
 *
 * @param[in]		-	Base address of the GPIO peripheral via GPIO_RegDef_t pointer.
 * @param[in]		-	ENABLE or DISABLE macros.
 * @param[in]		-
 *
 * @return			-	None
 *
 * @Note			-	None

 */


void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;				//correct IPR register for corresponding IRQ number
	uint8_t iprx_section = (IRQNumber % 4);		//correct IPR section within IPR register

	/*shifting the value of our IRQpriority by
	 * (8 (register bit width) * corresponding section) +
	 * (8 - NO_PR_BITS_IMPLEMENTED), which is 4 in STM32 case
	 *	the latter value allows us to shift our value to the index [8:5] of a IRQ subsection*/
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	//NVIC_PR_BASE_ADDR+iprx does address calculation by adding 4 (which would go onto next 32 bits)
	(*(NVIC_PR_BASE_ADDR+(iprx))) |= (IRQPriority << (shift_amount));
}

/******************************************************************
 * @fn				-	SPI_IRQHandling
 *
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port.
 *
 * @param[in]		-	Base address of the GPIO peripheral via GPIO_RegDef_t pointer.
 * @param[in]		-	ENABLE or DISABLE macros.
 * @param[in]		-
 *
 * @return			-	None
 *
 * @Note			-	None

 */

/*private functions/helper functions */


void SPI_IRQHandling(SPI_Handle_t *pHandle){
	uint8_t temp1, temp2;
	//first lets check for txe
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if(temp1 && temp2){
		spi_txe_interrupt_handle(pHandle);
	}

	//check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2){
		spi_rxne_interrupt_handle(pHandle);
	}

	//check for ovr flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if(temp1 && temp2){
		spi_ovr_interrupt_handle(pHandle);
	}
}



/******************************************************************
 * @fn				-	SPI_SendDataIT
 *
 *
 * @brief			-	This function sends data via SPI based on interrupt timing (not blocking)
 *
 * @param[in]		-	Base address of the GPIO peripheral via GPIO_RegDef_t pointer.
 * @param[in]		-	ENABLE or DISABLE macros.
 * @param[in]		-
 *
 * @return			-	None
 *
 * @Note			-	None

 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){

	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX){
		//1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark the SPI state as busy in transmission so that no
		//   other code is able to take over the currently used SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE);

	}
	return state;
}


/******************************************************************
 * @fn				-	SPI_ReceiveDataIT
 *
 *
 * @brief			-	This function receives data via SPI based on interrupt timing (not blocking)
 *
 * @param[in]		-	Base address of the GPIO peripheral via GPIO_RegDef_t pointer.
 * @param[in]		-	ENABLE or DISABLE macros.
 * @param[in]		-
 *
 * @return			-	None
 *
 * @Note			-	None

 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){

	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX){
		//1. Save the RX buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as busy in transmission so that no
		//   other code is able to take over the currently used SPI peripheral until receiving is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE);

	}
	return state;
}


// Helper Functions Implementations
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	while(pSPIHandle->TxLen > 0){

		//2. check the DFF bit in CR1
		if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))){
			//16 bit DFF
			//1. Load the data in to the DR
			pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer); //typecast to 16bit for 16 bit data transfer
			pSPIHandle->TxLen--; //subtracts length by 1, indicated 1 byte has been read
			pSPIHandle->TxLen--; //subtracts length by 1, indicating 2 total bytes have been read, 2 bytes = 16 bits
			(uint16_t*)pSPIHandle->pTxBuffer++; //increases Txbuffer memory address by 2 so that values are not overwritten
		}else{
			//8 bit DFF
			pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
			pSPIHandle->TxLen--;
			pSPIHandle->pTxBuffer++;
		}

		if(!pSPIHandle->TxLen){
			//TxLen is zero, so close the SPI transmission and inform the application that TX is over.

			SPI_CloseTransmission(pSPIHandle);

			//3. function callback to indicate to the application it is ready for interrupt
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
		}
	}
}



static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
	while(pSPIHandle->RxLen > 0){

		//2. check the DFF bit in CR1
		if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))){
			//16 bit DFF
			//1. Load the data in to the DR
			*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR; //typecast to 16bit for 16 bit data transfer
			pSPIHandle->RxLen--; //subtracts length by 1, indicated 1 byte has been read
			pSPIHandle->RxLen--; //subtracts length by 1, indicating 2 total bytes have been read, 2 bytes = 16 bits
			(uint16_t*)pSPIHandle->pRxBuffer--; //decreases Rxbuffer memory address by 2 so that values are not overwritten
		}else{
			//8 bit DFF
			*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer--;
		}

		if(!pSPIHandle->RxLen){
			//RxLen is zero, so close the SPI transmission and inform the application that TX is over.

			SPI_CloseReception(pSPIHandle);

			//3. function callback to indicate to the application it is ready for interrupt
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
		}
	}
}



static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle){

	uint8_t temp;

	//1. clear the OVR flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	//TxLen is zero, so close the SPI transmission and inform the application that TX is over.

	//1. prevents interrupts from setting up of TXE flag
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);

	//2. reset used values
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;


}


void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	//RxLen is zero, so close the SPI transmission and inform the application that TX is over.

	//1. prevents interrupts from setting up of RXE flag
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);

	//2. reset used values
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}



void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}


__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv){

	//this is a weak implementation. the application may over-ride this function
}

