#include <stdbool.h>
#include <intrinsics.h>
#include <os_cfg.h>
#include <ucos_ii.h>
#include <os_cpu.h>
#include <cpu.h>
#include <iolpc2378.h>
#include <timers.h>
#include <osutils.h>


/*
*********************************************************************************************************
*                                          OS_CPU_ExceptHndlr()
*
* Description : Handle any exceptions.
*
* Argument(s) : except_id     ARM exception type:
*
*                                  OS_CPU_ARM_EXCEPT_RESET             0x00
*                                  OS_CPU_ARM_EXCEPT_UNDEF_INSTR       0x01
*                                  OS_CPU_ARM_EXCEPT_SWI               0x02
*                                  OS_CPU_ARM_EXCEPT_PREFETCH_ABORT    0x03
*                                  OS_CPU_ARM_EXCEPT_DATA_ABORT        0x04
*                                  OS_CPU_ARM_EXCEPT_ADDR_ABORT        0x05
*                                  OS_CPU_ARM_EXCEPT_IRQ               0x06
*                                  OS_CPU_ARM_EXCEPT_FIQ               0x07
*
* Return(s)   : none.
*
* Caller(s)   : OS_CPU_ARM_EXCEPT_HANDLER(), which is declared in os_cpu_a.s.
*********************************************************************************************************
*/

void  OS_CPU_ExceptHndlr (CPU_DATA except_id)
{
    CPU_FNCT_VOID  pfnct;
                                                                        /* If this exception is either an IRQ or FIQ        */
    if ((except_id == OS_CPU_ARM_EXCEPT_IRQ) || (except_id == OS_CPU_ARM_EXCEPT_FIQ)) {
        pfnct = (CPU_FNCT_VOID)VICADDRESS;                              /* Read the interrupt vector from the VIC           */
        if (pfnct != (CPU_FNCT_VOID)0) {                                /* Make sure we don't have a NULL pointer           */
          (*pfnct)();                                                   /* Execute the ISR for the interrupting device      */
          VICADDRESS = 0;                                               /* Acknowlege the VIC interrupt                     */
        }
    } else {                                                                
        while (true) {
            // skip
        }
    }
}


void osStartTick(void) {
  initTimer(TIMER0, OSTimeTick, OS_TICKS_PER_SEC);
  startTimer(TIMER0);
  __enable_interrupt();
}

