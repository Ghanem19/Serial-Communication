/*


*/
/************************************************************************/
/*				                INCLUDES		                      	*/
/************************************************************************/

#include "common_macros.h"
#include "std_types.h"
#include "registers.h"
#include "interrupt.h"
#include "DIO.h"
#include "SPI.h"

/************************************************************************/
/*				                Enums			                    */
/************************************************************************/
typedef enum SPI_modes
{
    SPI_Mode0 = 0x00, SPI_Mode1 = 0x10, SPI_Mode2 = 0x20, SPI_Mode3 = 0x30
}SPI_Modes;


/************************************************************************/
/*				                Global defines                          */
/************************************************************************/

#define SPI2X               BIT0
#define MSTR                BIT4
#define DORD                BIT5
#define SPE                 BIT6
#define SPIE                BIT7
#define SPR0                BIT0
#define SPR1                BIT1
#define SPIF                BIT7
#define MISO_PORT           GPIOB
#define MOSI_PORT           GPIOB
#define SS_PORT             GPIOB
#define CLK_PORT            GPIOB
#define MISO_PIN            BIT6
#define MOSI_PIN            BIT5
#define CLK_PIN             BIT7
#define SS_PIN              BIT4
/************************************************************************/
/*				           CallBack functions pointers                   */
/************************************************************************/

void (*CBK_SPI_Interrupt) (void) = NULL;

/***********************************************************************/
/*				                Global Variables                       */
/***********************************************************************/

uint8 SPI_InitializationCheck  = FALSE;



/***********************************************************************/
/*				         Functions Definitions                         */
/***********************************************************************/

/***********************************************************************/
/* Description : SPI Initialization                                    */
/* Input       : SPI_Cfg_s* "Pointer to Struct of SPI Configration"    */
/* Output      : Error Checking                                        */
/***********************************************************************/
ERROR_STATUS SPI_Init(SPI_Cfg_s* SPI_Confg_ptr)
{
    if (SPI_Confg_ptr == NULL)
    {
        return E_NOK;
    }
    ASSIGN_BIT(SPCR, SPE, HIGH);
    if (SPI_Confg_ptr ->u8_SPIMode == MASTER)
    {
        ASSIGN_BIT(SPCR, MSTR, HIGH);

        DIO_Cfg_s DIO_MISO ={MISO_PORT, MISO_PIN, INPUT};
        DIO_init (&DIO_MISO);

        DIO_Cfg_s DIO_MOSI ={MOSI_PORT, MOSI_PIN, OUTPUT};
        DIO_init (&DIO_MOSI);

        DIO_Cfg_s DIO_CLK ={CLK_PORT, CLK_PIN, OUTPUT};
        DIO_init (&DIO_CLK);

        DIO_Cfg_s DIO_SS ={SS_PORT, SS_PIN, OUTPUT};
        DIO_init (&DIO_SS);
    }
    else if (SPI_Confg_ptr ->u8_SPIMode == SLAVE)
    {
        ASSIGN_BIT(SPCR, MSTR, LOW);

        DIO_Cfg_s DIO_MISO ={MISO_PORT, MISO_PIN, OUTPUT};
        DIO_init (&DIO_MISO);

        DIO_Cfg_s DIO_MOSI ={MOSI_PORT, MOSI_PIN, INPUT};
        DIO_init (&DIO_MOSI);

        DIO_Cfg_s DIO_CLK ={CLK_PORT, CLK_PIN, INPUT};
        DIO_init (&DIO_CLK);

        DIO_Cfg_s DIO_SS ={SS_PORT, SS_PIN, INPUT};
        DIO_init (&DIO_SS);
    }
    else
    {
        return E_NOK;
    }

    switch (SPI_Confg_ptr->u8_DataOrder)
    {
    case LSB:
        ASSIGN_BIT(SPCR, DORD, HIGH);
        break;
    case MSB:
        ASSIGN_BIT(SPCR, DORD, LOW);
        break;
    default:
        return E_NOK;
    }

    switch (SPI_Confg_ptr->u8_InterruptMode)
    {
    case INTERRUPT:
        ASSIGN_BIT(SPCR, SPIE,HIGH);
        break;
    case POLLING:
        ASSIGN_BIT(SPCR, SPIE, LOW);
        break;
    default:
        return E_NOK;
    }
       /* Set prescaler mode */
    switch (SPI_Confg_ptr-> u8_DataMode)
    {
    case MODE_0:
        SPCR |= SPI_Mode0;
        break;
    case MODE_1:
        SPCR |= SPI_Mode1;
        break;
    case MODE_2:
        SPCR |= SPI_Mode2;
        break;
    case MODE_3:
        SPCR |= SPI_Mode3;
        break;
    default:
        return E_NOK;
    }
    switch (SPI_Confg_ptr ->u8_Prescaler)
    {
    case SPI_PRESCALER_4:
        ASSIGN_BIT(SPCR, SPR0, LOW);
        ASSIGN_BIT(SPCR, SPR1, LOW);
        ASSIGN_BIT(SPSR, SPI2X, LOW);
        break;
    case SPI_PRESCALER_16:
        ASSIGN_BIT(SPCR, SPR0, HIGH);
        ASSIGN_BIT(SPCR, SPR1, LOW);
        ASSIGN_BIT(SPSR, SPI2X, LOW);
        break;
    case SPI_PRESCALER_64:
        ASSIGN_BIT(SPCR, SPR0, LOW);
        ASSIGN_BIT(SPCR, SPR1, HIGH);
        ASSIGN_BIT(SPSR, SPI2X, LOW);
        break;
    case SPI_PRESCALER_128:
        ASSIGN_BIT(SPCR, SPR0, HIGH);
        ASSIGN_BIT(SPCR, SPR1, HIGH);
        ASSIGN_BIT(SPSR, SPI2X, LOW);
        break;
    case SPI_PRESCALER_2:
        ASSIGN_BIT(SPCR, SPR0, LOW);
        ASSIGN_BIT(SPCR, SPR1, LOW);
        ASSIGN_BIT(SPSR, SPI2X, HIGH);
        break;
    case SPI_PRESCALER_8:
        ASSIGN_BIT(SPCR, SPR0, HIGH);
        ASSIGN_BIT(SPCR, SPR1, LOW);
        ASSIGN_BIT(SPSR, SPI2X, HIGH);
        break;
    case SPI_PRESCALER_32:
        ASSIGN_BIT(SPCR, SPR0, LOW);
        ASSIGN_BIT(SPCR, SPR1, HIGH);
        ASSIGN_BIT(SPSR, SPI2X, HIGH);
        break;
    default:
        return E_NOK;
    }
    CBK_SPI_Interrupt = SPI_Confg_ptr ->CBK_Func;
    SPI_InitializationCheck = TRUE;
    return E_OK;
}


/**
* @brief: Transmit one byte over SPI
* @param:
* Input : u8_Data "Byte to be sent"
* Output: None
* @return: Error codes if present
*/
ERROR_STATUS SPI_SendByte(uint8 u8_Data)
{
    if (SPI_InitializationCheck != TRUE)
    {
        return E_NOK;
    }
    DIO_Write (SS_PORT, SS_PIN, LOW);
    SPDR = u8_Data;
    while ((GET_BIT(SPSR, SPIF)) != TRUE);
    ASSIGN_BIT(SPSR, SPIF, LOW);
    DIO_Write (SS_PORT, SS_PIN, HIGH);
    return E_OK;
}


/**
* @brief: Receive one byte over SPI
* @param:
* Input : None
* Output: *ptru8_Data "points to where received byte is going to be stored in"
* @return: Error codes if present
*/
ERROR_STATUS SPI_ReceiveByte(uint8 *ptru8_Data)
{
    if (SPI_InitializationCheck != TRUE)
    {
        return E_NOK;
    }
    while ((GET_BIT(SPSR, SPIF)) != TRUE);
    ASSIGN_BIT(SPSR, SPIF, FALSE);
    *ptru8_Data = SPDR;
    return E_OK;
}

/**
* @brief: Get status of SPI interrupt flag
* @param:
* Input : None
* Output: *u8_Data "points to where flag value is going to be stored in"
* @return: Error codes if present
*/
ERROR_STATUS SPI_GetStatus(uint8 *u8_Data)
{
    if (SPI_InitializationCheck != TRUE)
    {
        return E_NOK;
    }
    *u8_Data = GET_BIT(SPSR, SPIF);
    return E_OK;
}

ISR (SPI_STC_vect)
{
    CBK_SPI_Interrupt();
}
