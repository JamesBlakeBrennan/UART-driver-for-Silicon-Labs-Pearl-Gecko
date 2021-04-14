/**
 * @file leuart.c
 * @author James Brennan
 * @date   4/2/2021
 * @brief Contains all the functions of the LEUART peripheral
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************

//** Standard Library includes
#include <string.h>

//** Silicon Labs include files
#include "em_gpio.h"
#include "em_cmu.h"

//** Developer/user include files
#include "leuart.h"
#include "scheduler.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// private variables
//***********************************************************************************
uint32_t	rx_done_evt;
uint32_t	tx_done_evt;
bool		leuart0_tx_busy;

static LEUART_STATE_MACHINE leuart_State;

enum state{transmit, done};

/***************************************************************************//**
 * @brief LEUART driver
 * @details
 *  This module contains all the functions to support the driver's state
 *  machine to transmit a string of data across the LEUART bus.  There are
 *  additional functions to support the Test Driven Development test that
 *  is used to validate the basic set up of the LEUART peripheral.  The
 *  TDD test for this class assumes that the LEUART is connected to the HM-18
 *  BLE module.  These TDD support functions could be used for any TDD test
 *  to validate the correct setup of the LEUART.
 *
 ******************************************************************************/

//***********************************************************************************
// Private functions
//***********************************************************************************


/***************************************************************************//**
 * @brief
 *   Function handle the TXC interrupt
 *
 * @details
 * 	 This routine is part of the LEUART state machine, it handles the TXC interrupt and
 * 	 all relevant state transitions.
 *
 *
 * @param[in] *leuartState
 *   Pointer to the STRUCT which handles the state machine
 *
 *
 ******************************************************************************/

void leuart_TXC_fun(LEUART_STATE_MACHINE *leuart_State){
	switch(leuart_State->STATE){
		case transmit:{
			EFM_ASSERT(false);
			break;
		}
		case done:
			leuart_State->leuart->IEN &= ~(LEUART_IEN_TXC);
			sleep_unblock_mode(EM2);
			add_scheduled_event(BLE_TX_DONE_CB);
			break;
		default:
			EFM_ASSERT(false);
			break;
	}

}

/***************************************************************************//**
 * @brief
 *   Function handle the TXBL interrupt
 *
 * @details
 * 	 This routine is part of the LEUART state machine, it handles the TXBL interrupt and
 * 	 all relevant state transitions.
 *
 *
 * @param[in] *leuartState
 *   Pointer to the STRUCT which handles the state machine
 *
 *
 ******************************************************************************/
void leuart_TXBL_fun(LEUART_STATE_MACHINE *leuart_State){
	switch(leuart_State->STATE){
		case transmit:{
				leuart_State->leuart->TXDATA = leuart_State->Data[leuart_State->dataCount];
				leuart_State->dataCount++;
				if(leuart_State->dataCount == leuart_State->str_len){
					leuart_State->STATE = done;
					leuart_State->leuart->IEN &= ~(LEUART_IEN_TXBL);
					leuart_State->leuart->IEN |= LEUART_IEN_TXC;
				}
				break;
		}
		case done:{
			EFM_ASSERT(false);
			break;
		}
		default:
			EFM_ASSERT(false);
			break;
	}
}




//***********************************************************************************
// Global functions
//***********************************************************************************

/***************************************************************************//**
 * @brief Opens the LEUART peripheral
 *
 *
 * @details
 * called onc to intialize the leuart
 *
 * @param[in] *leuart
 *   Defines the LEUART peripheral to access.
 *
 *@param[in] *leuart_settings
 *   Struct required to intialize the leuart
 *
 ******************************************************************************/

void leuart_open(LEUART_TypeDef *leuart, LEUART_OPEN_STRUCT *leuart_settings){

	//enabling the clock

		CMU_ClockEnable(cmuClock_LEUART0, true);



		//verifying that the clock has been enabled correctly

		if ((leuart->STARTFRAME & 0x01) == 0) {
			leuart->STARTFRAME = 0x01;
		EFM_ASSERT(leuart->STARTFRAME & 0x01);
		leuart->STARTFRAME = 0x01;
		}
		else {
			leuart->STARTFRAME = 0x01;
		EFM_ASSERT(!(leuart->STARTFRAME & 0x01));
		}

		while(leuart->SYNCBUSY);

		LEUART_Init_TypeDef leuart_init_values;

		leuart_init_values.refFreq = leuart_settings->refFreq;
		leuart_init_values.enable = leuart_settings->enable;
		leuart_init_values.baudrate = leuart_settings->baudrate;
		leuart_init_values.databits = leuart_settings->databits;
		leuart_init_values.parity = leuart_settings->parity;
		leuart_init_values.stopbits = leuart_settings->stopbits;


		LEUART_Init(leuart, &leuart_init_values);
		while(leuart->SYNCBUSY);
		//set up GPIO
		LEUART0->ROUTELOC0 = (leuart_settings->tx_loc << 8) | leuart_settings->rx_loc;
		LEUART0->ROUTEPEN = (leuart_settings->tx_pin_en << 1) | leuart_settings->rx_pin_en;

		//clear the RX and TX buffers
		LEUART0->CMD |= _LEUART_CMD_CLEARTX_MASK;
		LEUART0->CMD |= _LEUART_CMD_CLEARRX_MASK;
		while(leuart->SYNCBUSY);
		//Enable the LEAUART

		LEUART_Enable(leuart, leuartEnable);
		//while(leuart->SYNCBUSY);


		//Checking whether TX and RX have been enabled properly
		EFM_ASSERT(LEUART0->STATUS && ((leuart_settings->tx_en << 1) | leuart_settings->rx_en));

		//LEUART_IntClear(leuart, LEUART_IEN_TXBL |  LEUART_IEN_TXC );
		//LEUART_IntEnable(leuart, LEUART_IEN_TXBL |  LEUART_IEN_TXC );

		if(leuart == LEUART0){
			NVIC_EnableIRQ(LEUART0_IRQn);
		}




}

/***************************************************************************//**
 * @brief LEUART IRQ
 *
 *
 * @details
 * contains the calls to the leuart state machine functions
 *
 ******************************************************************************/

void LEUART0_IRQHandler(void){
	sleep_block_mode(EM3);
	uint32_t int_flag;
	int_flag = LEUART0->IF & LEUART0->IEN;
	LEUART0->IFC = int_flag;

	if (int_flag & LEUART_IEN_TXBL){
		leuart_TXBL_fun(&leuart_State);
	}
	if (int_flag & LEUART_IEN_TXC ){
		leuart_TXC_fun(&leuart_State);
	}







	sleep_unblock_mode(EM3);

}

/***************************************************************************//**
 * @brief
 *   LEUART START function starts the leuart
 *
 * @details
 * Initializes the leuart state struct and starts it by enabling TXBL
 *
 * @param[in] *leuart
 *   Defines the LEUART peripheral to access.
 *
 *@param[in] *string
 *   The string to be transmitted
 *
 *@param[in] *string_len
 *   The length of the transmission string
 *
 *
 *
 ******************************************************************************/

void leuart_start(LEUART_TypeDef *leuart, char *string, uint32_t string_len){
	sleep_block_mode(EM2);
	//EFM_ASSERT((leuart->STATE & _LEUART_STATE_STATE_MASK) == LEUART_STATE_STATE_IDLE); // X = the I2C peripheral #
	leuart_State.STATE = transmit;
	leuart_State.Data = string;
	leuart_State.leuart = leuart;
	leuart_State.dataCount = 0;
	leuart_State.str_len = string_len;

	LEUART_IntClear(leuart, LEUART_IEN_TXBL );
	LEUART_IntEnable(leuart, LEUART_IEN_TXBL);



}

/***************************************************************************//**
 * @brief
 *
 ******************************************************************************/

bool leuart_tx_busy(LEUART_TypeDef *leuart){

}

/***************************************************************************//**
 * @brief
 *   LEUART STATUS function returns the STATUS of the peripheral for the
 *   TDD test
 *
 * @details
 * 	 This function enables the LEUART STATUS register to be provided to
 * 	 a function outside this .c module.
 *
 * @param[in] *leuart
 *   Defines the LEUART peripheral to access.
 *
 * @return
 * 	 Returns the STATUS register value as an uint32_t value
 *
 ******************************************************************************/

uint32_t leuart_status(LEUART_TypeDef *leuart){
	uint32_t	status_reg;
	status_reg = leuart->STATUS;
	return status_reg;
}

/***************************************************************************//**
 * @brief
 *   LEUART CMD Write sends a command to the CMD register
 *
 * @details
 * 	 This function is used by the TDD test function to program the LEUART
 * 	 for the TDD tests.
 *
 * @note
 *   Before exiting this function to update  the CMD register, it must
 *   perform a SYNCBUSY while loop to ensure that the CMD has by synchronized
 *   to the lower frequency LEUART domain.
 *
 * @param[in] *leuart
 *   Defines the LEUART peripheral to access.
 *
 * @param[in] cmd_update
 * 	 The value to write into the CMD register
 *
 ******************************************************************************/

void leuart_cmd_write(LEUART_TypeDef *leuart, uint32_t cmd_update){

	leuart->CMD = cmd_update;
	while(leuart->SYNCBUSY);
}

/***************************************************************************//**
 * @brief
 *   LEUART IF Reset resets all interrupt flag bits that can be cleared
 *   through the Interrupt Flag Clear register
 *
 * @details
 * 	 This function is used by the TDD test program to clear interrupts before
 * 	 the TDD tests and to reset the LEUART interrupts before the TDD
 * 	 exits
 *
 * @param[in] *leuart
 *   Defines the LEUART peripheral to access.
 *
 ******************************************************************************/

void leuart_if_reset(LEUART_TypeDef *leuart){
	leuart->IFC = 0xffffffff;
}

/***************************************************************************//**
 * @brief
 *   LEUART App Transmit Byte transmits a byte for the LEUART TDD test
 *
 * @details
 * 	 The BLE module will respond to AT commands if the BLE module is not
 * 	 connected to the phone app.  To validate the minimal functionality
 * 	 of the LEUART peripheral, write and reads to the LEUART will be
 * 	 performed by polling and not interrupts.
 *
 * @note
 *   In polling a transmit byte, a while statement checking for the TXBL
 *   bit in the Interrupt Flag register is required before writing the
 *   TXDATA register.
 *
 * @param[in] *leuart
 *   Defines the LEUART peripheral to access.
 *
 * @param[in] data_out
 *   Byte to be transmitted by the LEUART peripheral
 *
 ******************************************************************************/

void leuart_app_transmit_byte(LEUART_TypeDef *leuart, uint8_t data_out){
	while (!(leuart->IF & LEUART_IF_TXBL));
	leuart->TXDATA = data_out;
}


/***************************************************************************//**
 * @brief
 *   LEUART App Receive Byte polls a receive byte for the LEUART TDD test
 *
 * @details
 * 	 The BLE module will respond to AT commands if the BLE module is not
 * 	 connected to the phone app.  To validate the minimal functionality
 * 	 of the LEUART peripheral, write and reads to the LEUART will be
 * 	 performed by polling and not interrupts.
 *
 * @note
 *   In polling a receive byte, a while statement checking for the RXDATAV
 *   bit in the Interrupt Flag register is required before reading the
 *   RXDATA register.
 *
 * @param[in] leuart
 *   Defines the LEUART peripheral to access.
 *
 * @return
 * 	 Returns the byte read from the LEUART peripheral
 *
 ******************************************************************************/

uint8_t leuart_app_receive_byte(LEUART_TypeDef *leuart){
	uint8_t leuart_data;
	while (!(leuart->IF & LEUART_IF_RXDATAV));
	leuart_data = leuart->RXDATA;
	return leuart_data;
}
