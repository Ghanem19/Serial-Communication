/*
 * Measurement_ECU.c
 *
 *  Created on: Dec 29, 2019
 *      Author: Ghanem and Tamoh
 */
/************************************************************************/
/*		                        INCLUDES                 		        */
/************************************************************************/
#include "Dio.h"
#include "BTN.h"
#include "softwareDelay.h"
#include "UART.h"
#include "SPI.h"
#include <stdlib.h>
#include "Measurement_ECU.h"
/************************************************************************/
/*		                        Defines                 		        */
/************************************************************************/
#define DEBOUNCING_DELAY 300
#define FITCH_PERIOD 5

#define TIME_RECEIVED  2
#define SPEED_RECEIVED 1
#define WAITING_DATA   0

#define NEW_LINE 13

#define DEBUG 1
/************************************************************************/
/*		              Static Functions Prototype         		        */
/************************************************************************/
static ERROR_STATUS MEASURE_ECU_PrintNumber(uint16 u8_number);

/************************************************************************/
/*		              Static and Global Variables         		        */
/************************************************************************/

uint8 gu8_speed = 0;
bool bool_fetchFlag = 0;
uint8 gu8_spiDataState = WAITING_DATA;

static uint16 gsu16_distance = 0;
static uint16 gsu16_currentTime = 0;
static uint8 gsu8_fetchTime = 0;

static uint8 gsu8_numberOfOverFlows = 0;

/************************************************************************/
/*		               FUNCTIONS' IMPLEMENTATION         		        */
/************************************************************************/
/*
 * Function: MEASURE_ECU_PrintNumber
 * Inputs:	 u8_number -> number to be send by UART
 * Outputs:
 * return: ERROR_STATUS -> check if there any error occurs, would return E_NOK else it would return E_OK.
 * Description: function converts the input number and send it by YART byte by byte
 * */
static ERROR_STATUS MEASURE_ECU_PrintNumber(uint16 u8_number)
{
	char str_stringBuffer[4] = {0};
	/*convert the coming number to string then send it to UART channel */
	utoa(u8_number, str_stringBuffer, 10);

	return UART_SendString(str_stringBuffer);

}
/*
 *Function: MEASURE_ECU_spiCBF
 * Inputs:
 * Outputs:
 * return: ERROR_STATUS -> check if there any error occurs, would return E_NOK else it would return E_OK.
 * Description: function reads the buffer of SPI if speed sent or time.
 */
void MEASURE_ECU_spiCBF(void) /* passed to spi */
{

	gu8_spiDataState++;
	if(gu8_spiDataState == SPEED_RECEIVED)
	{
		/*store the received byte in speed global variable*/
		SPI_ReceiveByte(&gu8_speed);
	}
	else if(gu8_spiDataState == TIME_RECEIVED)
	{
		/* if the received time is coming right after fetch button pressed*/
		if(bool_fetchFlag == TRUE)
		{

			SPI_ReceiveByte(&gsu8_fetchTime);
#if DEBUG
			UART_SendByte(NEW_LINE);
			UART_SendString("Fetch Time = ");
			MEASURE_ECU_PrintNumber(gsu8_fetchTime);
#endif
			gsu16_distance = gu8_speed;
			bool_fetchFlag = FALSE;
		}
		else if(bool_fetchFlag == FALSE)
		{
			SPI_ReceiveByte((uint8*)&gsu16_currentTime);
			/*the time received is only 8 bits so after 255 sec. overflow is occur
			 * to detect that we compared it with fetch time*/
			if(gsu16_currentTime < gsu8_fetchTime)
			{
				gsu8_numberOfOverFlows++;
			}
			else{}
			gsu16_currentTime = gsu16_currentTime  + 255*gsu8_numberOfOverFlows - gsu8_fetchTime;
			/* after receiving time and speed calculate and update the distance */
			/* reinitialize the dataStatus for next time.*/
			gsu16_distance += gu8_speed;
		}
		else
		{}
		/* return to wait initial state waiting for speed to be received */
		gu8_spiDataState = WAITING_DATA;
	}
	else
	{}

	/* check if the received time is fetch time or just update time.*/
}
/*
 *Function: MEASURE_ECU_Init
 * Inputs:
 * Outputs:
 * return: ERROR_STATUS -> check if there any error occurs, would return E_NOK else it would return E_OK.
 * Description: function initiate SPI, UART, DIO_fetch, BTN_0 and BTN_1.
 */
ERROR_STATUS MEASURE_ECU_Init(void)
{
	/* Initialize UART communication with the PC to print speed, time and distance*/
	UART_cfg_s ST_uartTxCfg =
           {UART_POLLING,
			TRANSMITTER,
			UART_NO_DOUBLE_SPEED,
			UART_ONE_STOP_BIT,
			UART_NO_PARITY,
			UART_8_BIT,
			9600,
			NULL,NULL,NULL};
	if(E_NOK == UART_Init(&ST_uartTxCfg))
		{
			return E_NOK;
		}
		else
		{}

	/* Initialize SPI communication with the state estimator ECU*/
	SPI_Cfg_s ST_spiSlaveCfg =
		   {SLAVE,
			MODE_0,
			LSB,
			INTERRUPT,
			SPI_PRESCALER_128,
			MEASURE_ECU_spiCBF};
	if(E_NOK == SPI_Init(&ST_spiSlaveCfg))
	{
		return E_NOK;
	}
	else
	{}

	/* initialize fetch pin as output to send trigger to estimator ECU*/
	DIO_Cfg_s ST_fetchDio = {FETCH_PORT, FETCH_PIN, OUTPUT};
	if(E_NOK == DIO_init(&ST_fetchDio))
	{
		return E_NOK;
	}
	else
	{}

	/* initialize 2 push button one for fetch and the other for print*/
	if(E_NOK == BTN_Init(FETCH_BTN))
	{
		return E_NOK;
	}
	else{}
	if(E_NOK == BTN_Init(PRINT_BTN))
	{
		return E_NOK;
	}
	else{}

	return E_OK;
}
/*
 *Function: MEASURE_ECU_Update
 * Inputs:
 * Outputs:
 * return: ERROR_STATUS -> check if there any error occurs, would return E_NOK else it would return E_OK.
 * Description: function updates the values of the two buttons and take action depending on them.
 */
ERROR_STATUS MEASURE_ECU_Update(void)
{
	uint8 u8_fetchBtnStatus = 0,u8_printBtnStatus = 0;
	if(E_OK == BTN_GetStatus(FETCH_BTN, &u8_fetchBtnStatus))
	{
		/* send fetch signal to state estimator to start send time and speed*/
		if(u8_fetchBtnStatus == PRESSED)
		{
			SwDelay_ms(DEBOUNCING_DELAY);


			DIO_Write(FETCH_PORT, FETCH_PIN, HIGH);
			SwDelay_ms(FITCH_PERIOD);
			DIO_Write(FETCH_PORT, FETCH_PIN, LOW);

			u8_fetchBtnStatus = RELEASED;
			bool_fetchFlag = TRUE;
		}
		else
		{}
	}
	else
	{
		return E_NOK;
	}

	if(E_OK == BTN_GetStatus(PRINT_BTN,  &u8_printBtnStatus))
	{
		if(u8_printBtnStatus == PRESSED)
		{

			SwDelay_ms(DEBOUNCING_DELAY);
			/* start print the value of speed, time and distance*/
			UART_SendByte(NEW_LINE);
			UART_SendString("Speed = ");
			if(E_NOK == MEASURE_ECU_PrintNumber(gu8_speed))
			{
				return E_NOK;
			}
			else
			{
			}

			UART_SendByte(NEW_LINE);
			UART_SendString("Time = ");
			if(E_NOK == MEASURE_ECU_PrintNumber(gsu16_currentTime))
			{
				return E_NOK;
			}
			else
			{
			}
			UART_SendByte(NEW_LINE);
			UART_SendString("Distance = ");
			if(E_NOK == MEASURE_ECU_PrintNumber(gsu16_distance))
			{
				return E_NOK;
			}
			else
			{
			}
			UART_SendByte(NEW_LINE);
		}
		else
		{

		}

	}
	else
	{
		return E_NOK;
	}


return E_OK;
}
