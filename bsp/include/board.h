/***************************************************************************
 **
 **    This file defines the board specific definition
 **
 **    Used with ARM IAR C/C++ Compiler and Assembler.
 **
 **    (c) Copyright IAR Systems 2005
 **
 **    $Revision: #7 $
 **
 ***************************************************************************/
#include <intrinsics.h>
#include "arm_comm.h"

#ifndef __BOARD_H
#define __BOARD_H

#define I_RC_OSC_FREQ   (4MHZ)
#define MAIN_OSC_FREQ   (12MHZ)
#define RTC_OSC_FREQ    (32768UL)

#if defined(IAR_LPC_2378_SK)

// USB Link LED
#define USB_LINK_LED_MASK   (1UL<<13)
#define USB_LINK_LED_DIR    IO0DIR
#define USB_LINK_LED_FDIR   FIO0DIR
#define USB_LINK_LED_SET    IO0SET
#define USB_LINK_LED_FSET   FIO0SET
#define USB_LINK_LED_CLR    IO0CLR
#define USB_LINK_LED_FCLR   FIO0CLR
#define USB_LINK_LED_IO     IO0PIN
#define USB_LINK_LED_FIO    FIO0PIN

// USB Connect LED
#define USB_CONNECT_LED_MASK   (1UL<<14)
#define USB_CONNECT_LED_DIR    IO0DIR
#define USB_CONNECT_LED_FDIR   FIO0DIR
#define USB_CONNECT_LED_SET    IO0SET
#define USB_CONNECT_LED_FSET   FIO0SET
#define USB_CONNECT_LED_CLR    IO0CLR
#define USB_CONNECT_LED_FCLR   FIO0CLR
#define USB_CONNECT_LED_IO     IO0PIN
#define USB_CONNECT_LED_FIO    FIO0PIN

// LCD Backlight
#define LCD_BL_MASK         (1UL<<26)
#define LCD_BL_DIR          IO1DIR
#define LCD_BL_FDIR         FIO1DIR
#define LCD_BL_SET          IO1SET
#define LCD_BL_FSET         FIO1SET
#define LCD_BL_CLR          IO1CLR
#define LCD_BL_FCLR         FIO1CLR
#define LCD_BL_IO           IO1PIN
#define LCD_BL_FIO          FIO1PIN
// LCD Reset
#define LCD_RST_MASK        (1UL<<25)
#define LCD_RST_FDIR        FIO3DIR
#define LCD_RST_FSET        FIO3SET
#define LCD_RST_FCLR        FIO3CLR
#define LCD_RST_FIO         FIO3PIN
// LCD CS
#define LCD_CS_MASK         (1UL<<21)
#define LCD_CS_DIR          IO1DIR
#define LCD_CS_FDIR         FIO1DIR
#define LCD_CS_SET          IO1SET
#define LCD_CS_FSET         FIO1SET
#define LCD_CS_CLR          IO1CLR
#define LCD_CS_FCLR         FIO1CLR
#define LCD_CS_IO           IO1PIN
#define LCD_CS_FIO          FIO1PIN

// Buttons
#define B1_MASK             (1UL<<29)
#define B1_DIR              IO0DIR
#define B1_FDIR             FIO0DIR
#define B1_IO               IO0PIN
#define B1_FIO              FIO0PIN

#define B2_MASK             (1UL<<18)
#define B2_DIR              IO0DIR
#define B2_FDIR             FIO0DIR
#define B2_IO               IO0PIN
#define B2_FIO              FIO0PIN

// Joystick
#define JS_RIGHT_MASK       (1UL << 22)
#define JS_LEFT_MASK        (1UL << 27)
#define JS_UP_MASK          (1UL << 18)
#define JS_DOWN_MASK        (1UL << 19)
#define JS_CENTER_MASK      (1UL << 25)
#define JS_DIR              IO1DIR
#define JS_FDIR             FIO1DIR
#define JS_IO               IO1PIN
#define JS_FIO              FIO1PIN

// MMC/SD card switches
// Card present
#define MMC_CP_MASK         (1UL << 17)
#define MMC_CP_DIR          IO0DIR
#define MMC_CP_FDIR         FIO0DIR
#define MMC_CP_IO           IO0PIN
#define MMC_CP_FIO          FIO0PIN
#define MMC_CP_MODE         PINMODE1_bit.P0_17

// Card write protect
#define MMC_WP_MASK         (1UL << 29)
#define MMC_WP_DIR          IO1DIR
#define MMC_WP_FDIR         FIO1DIR
#define MMC_WP_IO           IO1PIN
#define MMC_WP_FIO          FIO1PIN
#define MMC_WP_MODE         PINMODE3_bit.P1_29

//MMA
#define X_CHANNEL           1
#define X_CHANNEL_SEL       PINSEL1_bit.P0_24
#define Y_CHANNEL           0
#define Y_CHANNEL_SEL       PINSEL1_bit.P0_23
#define Z_CHANNEL           6
#define Z_CHANNEL_SEL       PINSEL0_bit.P0_12

// Analog trim
#define ANALOG_TRIM_CHANNEL 5
#define ANALOG_TRIM_CHANNEL_SEL   PINSEL3_bit.P1_31

// MIC Input
#define MIC_IN_CHANNEL      3
#define MIC_IN_CHANNEL_SEL  PINMODE1_bit.P0_25

// Check for board revision.
// The board with PHY routing mistake have 4.7k resistor on port P0.21 and
// internal pull-up can't set logical 1.
// To fix routing mistake U4's pin 9 must be swaped with pin 22 and add pull
// down 1k resistor on a pin 9.
#define BOARD_BUG_CTRL      PINMODE1_bit.P0_21
#define BOARD_BUG_FDATA     FIO0PIN_bit.P0_21
#define BOARD_BUG_DATA      IO0PIN_bit.P0_21

#elif defined(IAR_LPC_P2378_SK)

// USB Link LED
#define USB_LINK_LED_MASK   (1UL<<13)
#define USB_LINK_LED_DIR    IO0DIR
#define USB_LINK_LED_FDIR   FIO0DIR
#define USB_LINK_LED_SET    IO0SET
#define USB_LINK_LED_FSET   FIO0SET
#define USB_LINK_LED_CLR    IO0CLR
#define USB_LINK_LED_FCLR   FIO0CLR
#define USB_LINK_LED_IO     IO0PIN
#define USB_LINK_LED_FIO    FIO0PIN

// USB Connect LED
#define USB_CONNECT_LED_MASK   (1UL<<14)
#define USB_CONNECT_LED_DIR    IO0DIR
#define USB_CONNECT_LED_FDIR   FIO0DIR
#define USB_CONNECT_LED_SET    IO0SET
#define USB_CONNECT_LED_FSET   FIO0SET
#define USB_CONNECT_LED_CLR    IO0CLR
#define USB_CONNECT_LED_FCLR   FIO0CLR
#define USB_CONNECT_LED_IO     IO0PIN
#define USB_CONNECT_LED_FIO    FIO0PIN

// Status LED
#define STATUS_LED_MASK     (1UL<<19)
#define STATUS_LED_DIR      IO1DIR
#define STATUS_LED_FDIR     FIO1DIR
#define STATUS_LED_SET      IO1SET
#define STATUS_LED_FSET     FIO1SET
#define STATUS_LED_CLR      IO1CLR
#define STATUS_LED_FCLR     FIO1CLR
#define STATUS_LED_IO       IO1PIN
#define STATUS_LED_FIO      FIO1PIN

// Buttons
#define B1_MASK             (1UL<<30)
#define B1_DIR              IO0DIR
#define B1_FDIR             FIO0DIR
#define B1_IO               IO0PIN
#define B1_FIO              FIO0PIN

#define B2_MASK             (1UL<<6)
#define B2_DIR              IO0DIR
#define B2_FDIR             FIO0DIR
#define B2_IO               IO0PIN
#define B2_FIO              FIO0PIN

// MMC/SD card switches
// Card present
#define MMC_CP_MASK         (1UL << 29)
#define MMC_CP_DIR          IO0DIR
#define MMC_CP_FDIR         FIO0DIR
#define MMC_CP_IO           IO0PIN
#define MMC_CP_FIO          FIO0PIN
#define MMC_CP_MODE         PINMODE1_bit.P0_29

// Card write protect
#define MMC_WP_MASK         (1UL << 28)
#define MMC_WP_DIR          IO1DIR
#define MMC_WP_FDIR         FIO1DIR
#define MMC_WP_IO           IO1PIN
#define MMC_WP_FIO          FIO1PIN
#define MMC_WP_MODE         PINMODE3_bit.P1_28

// Check for board revision.
// The board with PHY routing mistake have 4.7k resistor on port P0.21 and
// internal pull-up can't set logical 1.
// To fix routing mistake U4's pin 9 must be swaped with pin 22 and add pull
// down 1k resistor on a pin 9.
#define BOARD_BUG_CTRL      PINMODE1_bit.P0_21
#define BOARD_BUG_FDATA     FIO0PIN_bit.P0_21
#define BOARD_BUG_DATA      IO0PIN_bit.P0_21

#else
#error define type of the board
#endif

// PCLK offset
#define WDT_PCLK_OFFSET     0
#define TIMER0_PCLK_OFFSET  2
#define TIMER1_PCLK_OFFSET  4
#define UART0_PCLK_OFFSET   6
#define UART1_PCLK_OFFSET   8
#define PWM0_PCLK_OFFSET    10
#define PWM1_PCLK_OFFSET    12
#define I2C0_PCLK_OFFSET    14
#define SPI_PCLK_OFFSET     16
#define RTC_PCLK_OFFSET     18
#define SSP1_PCLK_OFFSET    20
#define DAC_PCLK_OFFSET     22
#define ADC_PCLK_OFFSET     24
#define CAN1_PCLK_OFFSET    26
#define CAN2_PCLK_OFFSET    28
#define ACF_PCLK_OFFSET     30
#define BAT_RAM_PCLK_OFFSET 32
#define GPIO_PCLK_OFFSET    34
#define PCB_PCLK_OFFSET     36
#define I2C1_PCLK_OFFSET    38
//#define                   40
#define SSP0_PCLK_OFFSET    42
#define TIMER2_PCLK_OFFSET  44
#define TIMER3_PCLK_OFFSET  46
#define UART2_PCLK_OFFSET   48
#define UART3_PCLK_OFFSET   50
#define I2C2_PCLK_OFFSET    52
#define I2S_PCLK_OFFSET     54
#define MCI_PCLK_OFFSET     56
//#define                   58
#define PCLK_PCLK_OFFSET    60
//#define                   62

#define IRQ_FLAG            0x80
#define FIQ_FLAG            0x40

#endif /* __BOARD_H */
