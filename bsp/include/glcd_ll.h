/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    (c) Copyright IAR Systems 2006
 *
 *    File name   : glcd_ll.h
 *    Description : GLCD low level include file
 *
 *    History :
 *    1. Date        : December 2, 2006
 *       Author      : Stanimir Bonev
 *       Description : Create
 *
 **************************************************************************/
#ifndef __GLCD_LL_H
#define __GLCD_LL_H

/* Old values BACKLIGHT_OFF 0x40, BACKLIGHT_ON 0x20 */
#define BACKLIGHT_OFF   0x00
#define BACKLIGHT_ON    0x01
#define SSP_FIFO_SIZE 8

extern Int32U getFpclk(Int32U Periphery);


// Lynch code
#define BIT26 0x04000000
void Backlight( unsigned char state );

/*************************************************************************
 * Function Name: LcdSetReset
 * Parameters: Boolean State
 * Return: none
 *
 * Description: Set reset pin state
 *
 *************************************************************************/
void LcdSetReset (Boolean State);

/*************************************************************************
 * Function Name: LcdSetBacklight
 * Parameters: Int8U Light
 * Return: none
 *
 * Description: Set backlight level
 *
 *************************************************************************/
void LCDSetBacklight (Int8U Light);

/*************************************************************************
 * Function Name: LcdLLInit
 * Parameters: none
 * Return: none
 *
 * Description: Init Reset and Backlight control outputs
 *
 *************************************************************************/
void LcdLLInit (void);

/*************************************************************************
 * Function Name: Ssp0ChipSelect
 * Parameters: Boolean Select
 * Return: none
 *
 * Description: SSP0 Chip select control
 * Select = true  - Chip is enable
 * Select = false - Chip is disable
 *
 *************************************************************************/
void Ssp0ChipSelect(Boolean Select);

/*************************************************************************
 * Function Name: Ssp0SetWordWidth
 * Parameters: Int32U Width
 * Return: Boolean
 *
 * Description: Set SSP 0 word width
 *
 *************************************************************************/
Boolean Ssp0SetWordWidth (Int32U Width);

/*************************************************************************
 * Function Name: Ssp0SetClockFreq
 * Parameters: Int32U Frequency
 * Return: Int32U
 *
 * Description: Set SSP 0 clock
 *
 *************************************************************************/
Int32U Ssp0SetClockFreq (Int32U Frequency);

/*************************************************************************
 * Function Name: Ssp0Init
 * Parameters: Int32U Clk, Int32U Width
 * Return: none
 *
 * Description: Init SSP0
 *
 *************************************************************************/
void Ssp0Init(Int32U Clk, Int32U Width);

/*************************************************************************
 * Function Name: Ssp0TranserByte
 * Parameters: Int32U Data
 * Return: Int32U
 *
 * Description: Transfer byte from SSP0
 *
 *************************************************************************/
Int32U Ssp0TranserByte (Int32U Data);

/*************************************************************************
 * Function Name: Ssp0SendBlock
 * Parameters: pInt8U pData, Int32U Size
 *
 * Return: void
 *
 * Description: Read byte from SSP
 *
 *************************************************************************/
void Ssp0SendBlock (pInt8U pData, Int32U Size);

/*************************************************************************
 * Function Name: Ssp0ReceiveBlock
 * Parameters: pInt8U pData, Int32U Size
 *
 * Return: void
 *
 * Description: Read byte from SSP
 *
 *************************************************************************/
void Ssp0ReceiveBlock (pInt8U pData, Int32U Size);

#endif // __GLCD_LL_H
