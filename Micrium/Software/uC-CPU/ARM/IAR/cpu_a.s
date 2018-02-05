;********************************************************************************************************
;                                               uC/CPU
;                                    CPU CONFIGURATION & PORT LAYER
;
;                          (c) Copyright 2004-2007; Micrium, Inc.; Weston, FL
;
;               All rights reserved.  Protected by international copyright laws.
;
;               uC/CPU is provided in source form for FREE evaluation, for educational
;               use or peaceful research.  If you plan on using uC/CPU in a commercial
;               product you need to contact Micrium to properly license its use in your
;               product.  We provide ALL the source code for your convenience and to
;               help you experience uC/CPU.  The fact that the source code is provided
;               does NOT mean that you can use it without paying a licensing fee.
;
;               Knowledge of the source code may NOT be used to develop a similar product.
;
;               Please help us continue to provide the Embedded community with the finest
;               software available.  Your honesty is greatly appreciated.
;********************************************************************************************************

;********************************************************************************************************
;
;                                            CPU PORT FILE
;
;                                                 ARM
;                                            IAR C Compiler
;
; Filename      : cpu_a.s
; Version       : V1.17
; Programmer(s) : JJL
;                 JDH
;                 ITJ
;********************************************************************************************************


;********************************************************************************************************
;                                           PUBLIC FUNCTIONS
;********************************************************************************************************

    PUBLIC  CPU_SR_Save
    PUBLIC  CPU_SR_Restore


;********************************************************************************************************
;                                                EQUATES
;********************************************************************************************************

CPU_ARM_CTRL_INT_DIS  EQU       0xC0                            ; Disable both FIQ & IRQ


;********************************************************************************************************
;                                      CODE GENERATION DIRECTIVES
;********************************************************************************************************

    RSEG CODE:CODE:NOROOT(2)
    CODE32


;$PAGE
;********************************************************************************************************
;                                      CRITICAL SECTION FUNCTIONS
;
; Description : Disable/Enable interrupts by preserving the state of interrupts.  Generally speaking, the 
;               state of the interrupt disable flag is stored in the local variable 'cpu_sr' & interrupts
;               are then disabled ('cpu_sr' is allocated in all functions that need to disable interrupts).
;               The previous interrupt state is restored by copying 'cpu_sr' into the CPU's status register.
;
; Prototypes  : CPU_SR  CPU_SR_Save   (void);
;               void    CPU_SR_Restore(CPU_SR  cpu_sr);
;
; Note(s)     : (1) These functions are used in general like this :
;
;                       void  Task (void  *p_arg)
;                       {
;                                                               /* Allocate storage for CPU status register */
;                       #if (CPU_CFG_CRITICAL_METHOD == CPU_CRITICAL_METHOD_STATUS_LOCAL)
;                           CPU_SR  cpu_sr;
;                       #endif
;
;                                :
;                                :
;                           CPU_CRITICAL_ENTER();               /* cpu_sr = CPU_SR_Save();                  */
;                                :
;                                :
;                           CPU_CRITICAL_EXIT();                /* CPU_SR_Restore(cpu_sr);                  */
;                                :
;                                :
;                       }
;
;               (2) CPU_SR_Restore() is implemented as recommended by Atmel's application note :
;
;                       "Disabling Interrupts at Processor Level"
;********************************************************************************************************

CPU_SR_Save
        MRS     R0, CPSR

CPU_SR_Save_Loop
                                                                ; Set IRQ & FIQ bits in CPSR to DISABLE all interrupts
        ORR     R1, R0, #CPU_ARM_CTRL_INT_DIS
        MSR     CPSR_c, R1
        MRS     R1, CPSR                                        ; Confirm that CPSR contains the proper interrupt disable flags
        AND     R1, R1, #CPU_ARM_CTRL_INT_DIS
        CMP     R1,     #CPU_ARM_CTRL_INT_DIS
        BNE     CPU_SR_Save_Loop                                ; NOT properly DISABLED (try again)
        BX      LR                                              ; DISABLED, return the original CPSR contents in R0


CPU_SR_Restore                                                  ; See Note #2
        MSR     CPSR_c, R0
        BX      LR


;$PAGE
;********************************************************************************************************
;                                     CPU ASSEMBLY PORT FILE END
;********************************************************************************************************

        END

