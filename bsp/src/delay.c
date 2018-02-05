#include <stdint.h>
#include "delay.h"

/*************************************************************************
 * Function Name: dly100us
 * Parameters: uint32_t arg
 * Return: void
 *
 * Description: Delay [100us] approximately!
 *
 * 
 *		
 *************************************************************************/
void dly100us(uint32_t arg) {
  volatile uint32_t dly;
  volatile uint32_t dly100;

  for(dly = arg; dly > 0; dly--) {
    for(dly100 = 460; dly100 > 0; dly100--) {
      // skip
    }
  }
}


