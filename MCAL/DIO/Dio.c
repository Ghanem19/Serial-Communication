/************************************************************************/
/*						      INCLUDES                              	*/
/************************************************************************/

#include "Dio.h"
#include "registers.h"
/************************************************************************/
/*								DEFINES                                	*/
/************************************************************************/
#define PORT_REG_OFFSET 0x3b
#define DIR_REG_OFFSET  0x3a
#define PIN_REG_OFFSET  0x39

#define NUM_OF_PORTS    4
/************************************************************************/
/*						FUNCTIONS IMPLEMENTATION                       	*/
/************************************************************************/
/*
*Input: DIO_Cfg_s -> to get PORT name, pins to be initiated and the required direction
*Output: No output
*In/Out:
*Description: This function can set the direction of a full port, a nibble
* 			  or even one pin.
*/
ERROR_STATUS DIO_init (DIO_Cfg_s *DIO_info)
{	/* get the direction register without switch cases */
	reg_type8 reg8_dirReg = (reg_type8)(DIR_REG_OFFSET  - DIO_info->GPIO*3);
	if(DIO_info->GPIO < NUM_OF_PORTS)
	{
		if(DIO_info->dir == INPUT)
		{
			/*clear pins to set them as input*/
			(*reg8_dirReg) &= ~(DIO_info->pins);
		}
		else if(DIO_info->dir == OUTPUT)
		{
			/*set pins to be outputs*/
			(*reg8_dirReg) |= (DIO_info->pins);
		}
		else
		{
			return E_NOK;
		}

	}
	else
	{
		return E_NOK;
	}


	return E_OK;
}


/*
*Input: GPIO -> to get PORT name
*					- GPIOA
*					- GPIOB
*					- GPIOC
*					- GPIOD
*		pins -> pins to be written at.
*					- PIN0
*					- PIN1
*					- PIN2
*					- PIN3
*					- PIN4
*					- PIN5
*					- PIN6
*					- PIN7
*					- UPPER_NIBBLE
*					- LOWER_NIBBLE
*					- FULL_PORT
*		value 	-> The desired value
*					- HIGH
*					- LOW
*Output: No output
*In/Out: No In/Out
*Description: This function can set the value of a full port, a nibble
* 			  or even one pin.
*/
ERROR_STATUS DIO_Write (uint8 GPIO, uint8 pins, uint8 value)
{
	/* get the port register address*/
	reg_type8 reg8_portReg = (reg_type8)(PORT_REG_OFFSET  - GPIO*3);

	if(GPIO < NUM_OF_PORTS)
	{
		/* specify value that written to the */
		if(value == LOW)
		{
			/*clear selected pins */
			(*reg8_portReg) &= ~(pins);
		}
		else if(value == HIGH)
		{
			/* SET the selected pins */
			(*reg8_portReg) |= (pins);
		}
		else
		{
			return E_NOK;
		}
	}
	else
	{
		return E_NOK;
	}

	return E_OK;
}

/*
*Input: GPIO -> to get PORT name
*					- GPIOA
*					- GPIOB
*					- GPIOC
*					- GPIOD
*		pins -> pins to be written at.
*					- PIN0
*					- PIN1
*					- PIN2
*					- PIN3
*					- PIN4
*					- PIN5
*					- PIN6
*					- PIN7
*					- UPPER_NIBBLE
*					- LOWER_NIBBLE
*					- FULL_PORT
*		value 	-> The desired value
*					- HIGH
*					- LOW
*Output: data -> the acquired data wether it was PORT data or pins data
*In/Out:
*Description: This function gets the value of a full port, a nibble
* 			  or even one pin.
*/
ERROR_STATUS DIO_Read (uint8 GPIO,uint8 pins, uint8 *data)
{
	/* get the pin register address*/
	reg_type8 reg8_pinReg = (reg_type8)(PIN_REG_OFFSET  - GPIO*3);
	if(GPIO < NUM_OF_PORTS)
	{

		if(GET_BIT((*reg8_pinReg),pins) != LOW)
		{
			*data = HIGH;
		}
		else if(GET_BIT((*reg8_pinReg),pins) == LOW)
		{
			*data = LOW;
		}
		else
		{
			return E_NOK;
		}

	}
	else
	{
		return E_NOK;
	}

	return E_OK;
}

/*
*Input: GPIO -> to get PORT name
*					- GPIOA
*					- GPIOB
*					- GPIOC
*					- GPIOD
*		pins -> pins to be written at.
*					- PIN0
*					- PIN1
*					- PIN2
*					- PIN3
*					- PIN4
*					- PIN5
*					- PIN6
*					- PIN7
*					- UPPER_NIBBLE
*					- LOWER_NIBBLE
*					- FULL_PORT
*Output: data -> No output
*In/Out:
*Description: This function toggles the value of a full port, a nibble
* 			  or even one pin.
*/
ERROR_STATUS DIO_Toggle (uint8 GPIO, uint8 pins)
{	/* get the value of passed port register*/
	reg_type8 reg8_portReg = (reg_type8)(PORT_REG_OFFSET  - GPIO*3);
	if(GPIO < NUM_OF_PORTS)
	{
		(*reg8_portReg) ^= pins;
	}
	else
	{
		return E_NOK;
	}
	return E_OK;
}
