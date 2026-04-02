/**
 * GD32F130C6T6 startup file for GCC
 *
 * Matches the original SCServo21 firmware's reset handler pattern:
 *   1. Load SP from literal pool
 *   2. Copy .data from flash to SRAM
 *   3. Zero .bss
 *   4. Call SystemInit (clock setup)
 *   5. Call __libc_init_array (C runtime)
 *   6. Call main
 *   7. Infinite loop
 */

  .syntax unified
  .cpu cortex-m3
  .fpu softvfp
  .thumb

.global g_pfnVectors
.global Default_Handler

/**
 * Reset handler — entry point after reset.
 * Matches the original firmware at flash offset 0xCC.
 */
  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0

  /* Copy .data initializers from flash to SRAM */
  movs  r1, #0
  b     LoopCopyDataInit

CopyDataInit:
  ldr   r3, =_sidata
  ldr   r3, [r3, r1]
  str   r3, [r0, r1]
  adds  r1, #4

LoopCopyDataInit:
  ldr   r0, =_sdata
  ldr   r3, =_edata
  adds  r2, r0, r1
  cmp   r2, r3
  bcc   CopyDataInit

  /* Zero fill .bss */
  ldr   r2, =_sbss
  b     LoopFillZerobss

FillZerobss:
  movs  r3, #0
  str   r3, [r2, #0]
  adds  r2, #4

LoopFillZerobss:
  ldr   r3, =_ebss
  cmp   r2, r3
  bcc   FillZerobss

  /* Call system init (clock config) */
  bl    SystemInit
  /* Call C runtime init */
  bl    __libc_init_array
  /* Call application entry point */
  bl    main
  /* Should never return, but loop if it does */
  b     .

  .size Reset_Handler, .-Reset_Handler

/**
 * Default handler for unimplemented interrupts
 */
  .section .text.Default_Handler, "ax", %progbits
Default_Handler:
  b     .
  .size Default_Handler, .-Default_Handler

/**
 * Vector table — placed at flash offset 0x00 by the linker script.
 */
  .section .isr_vector, "a", %progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
  .word _estack
  .word 0x08007801        /* Bootloader entry (thumb). Bootloader jumps to 0x0CC. */
  .word NMI_Handler
  .word HardFault_Handler
  .word MemManage_Handler
  .word BusFault_Handler
  .word UsageFault_Handler
  .word 0, 0, 0, 0
  .word SVC_Handler
  .word DebugMon_Handler
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler

  /* External interrupts (GD32F130 IRQ 0-31) */
  .word WWDGT_IRQHandler            /* 0  */
  .word LVD_IRQHandler              /* 1  */
  .word RTC_IRQHandler              /* 2  */
  .word FMC_IRQHandler              /* 3  */
  .word RCU_IRQHandler              /* 4  */
  .word EXTI0_1_IRQHandler          /* 5  */
  .word EXTI2_3_IRQHandler          /* 6  */
  .word EXTI4_15_IRQHandler         /* 7  */
  .word 0                           /* 8  */
  .word DMA_Channel0_IRQHandler     /* 9  */
  .word DMA_Channel1_2_IRQHandler   /* 10 */
  .word DMA_Channel3_4_IRQHandler   /* 11 */
  .word ADC_CMP_IRQHandler          /* 12 */
  .word TIMER0_BRK_UP_TRG_COM_IRQHandler /* 13 */
  .word TIMER0_Channel_IRQHandler   /* 14 */
  .word TIMER2_IRQHandler           /* 15 */
  .word TIMER5_IRQHandler           /* 16 */
  .word 0                           /* 17 */
  .word TIMER13_IRQHandler          /* 18 */
  .word TIMER14_IRQHandler          /* 19 */
  .word TIMER15_IRQHandler          /* 20 */
  .word TIMER16_IRQHandler          /* 21 */
  .word I2C0_EV_IRQHandler          /* 22 */
  .word I2C0_ER_IRQHandler          /* 23 */
  .word 0                           /* 24 */
  .word SPI0_IRQHandler             /* 25 */
  .word SPI1_IRQHandler             /* 26 */
  .word USART0_IRQHandler           /* 27 */
  .word USART1_IRQHandler           /* 28 */
  .word 0                           /* 29 */
  .word 0                           /* 30 */
  .word 0                           /* 31 */
  /* Pad to 0xCC so Reset_Handler is at the bootloader's hardcoded jump target */
  .word 0                           /* 32 (padding) */
  .word 0                           /* 33 (padding) */
  .word 0                           /* 34 (padding) */

/**
 * Weak aliases — all default to Default_Handler unless overridden.
 */
  .weak NMI_Handler
  .thumb_set NMI_Handler, Default_Handler
  .weak HardFault_Handler
  .thumb_set HardFault_Handler, Default_Handler
  .weak MemManage_Handler
  .thumb_set MemManage_Handler, Default_Handler
  .weak BusFault_Handler
  .thumb_set BusFault_Handler, Default_Handler
  .weak UsageFault_Handler
  .thumb_set UsageFault_Handler, Default_Handler
  .weak SVC_Handler
  .thumb_set SVC_Handler, Default_Handler
  .weak DebugMon_Handler
  .thumb_set DebugMon_Handler, Default_Handler
  .weak PendSV_Handler
  .thumb_set PendSV_Handler, Default_Handler
  .weak SysTick_Handler
  .thumb_set SysTick_Handler, Default_Handler
  .weak WWDGT_IRQHandler
  .thumb_set WWDGT_IRQHandler, Default_Handler
  .weak LVD_IRQHandler
  .thumb_set LVD_IRQHandler, Default_Handler
  .weak RTC_IRQHandler
  .thumb_set RTC_IRQHandler, Default_Handler
  .weak FMC_IRQHandler
  .thumb_set FMC_IRQHandler, Default_Handler
  .weak RCU_IRQHandler
  .thumb_set RCU_IRQHandler, Default_Handler
  .weak EXTI0_1_IRQHandler
  .thumb_set EXTI0_1_IRQHandler, Default_Handler
  .weak EXTI2_3_IRQHandler
  .thumb_set EXTI2_3_IRQHandler, Default_Handler
  .weak EXTI4_15_IRQHandler
  .thumb_set EXTI4_15_IRQHandler, Default_Handler
  .weak DMA_Channel0_IRQHandler
  .thumb_set DMA_Channel0_IRQHandler, Default_Handler
  .weak DMA_Channel1_2_IRQHandler
  .thumb_set DMA_Channel1_2_IRQHandler, Default_Handler
  .weak DMA_Channel3_4_IRQHandler
  .thumb_set DMA_Channel3_4_IRQHandler, Default_Handler
  .weak ADC_CMP_IRQHandler
  .thumb_set ADC_CMP_IRQHandler, Default_Handler
  .weak TIMER0_BRK_UP_TRG_COM_IRQHandler
  .thumb_set TIMER0_BRK_UP_TRG_COM_IRQHandler, Default_Handler
  .weak TIMER0_Channel_IRQHandler
  .thumb_set TIMER0_Channel_IRQHandler, Default_Handler
  .weak TIMER2_IRQHandler
  .thumb_set TIMER2_IRQHandler, Default_Handler
  .weak TIMER5_IRQHandler
  .thumb_set TIMER5_IRQHandler, Default_Handler
  .weak TIMER13_IRQHandler
  .thumb_set TIMER13_IRQHandler, Default_Handler
  .weak TIMER14_IRQHandler
  .thumb_set TIMER14_IRQHandler, Default_Handler
  .weak TIMER15_IRQHandler
  .thumb_set TIMER15_IRQHandler, Default_Handler
  .weak TIMER16_IRQHandler
  .thumb_set TIMER16_IRQHandler, Default_Handler
  .weak I2C0_EV_IRQHandler
  .thumb_set I2C0_EV_IRQHandler, Default_Handler
  .weak I2C0_ER_IRQHandler
  .thumb_set I2C0_ER_IRQHandler, Default_Handler
  .weak SPI0_IRQHandler
  .thumb_set SPI0_IRQHandler, Default_Handler
  .weak SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler, Default_Handler
  .weak USART0_IRQHandler
  .thumb_set USART0_IRQHandler, Default_Handler
  .weak USART1_IRQHandler
  .thumb_set USART1_IRQHandler, Default_Handler
