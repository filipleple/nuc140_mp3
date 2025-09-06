/* NUC140 (Cortex-M0) GCC startup, no IT blocks, Keil IRQ order */
.syntax unified
.cpu cortex-m0
.thumb

/* Linker symbols */
.extern _estack
.extern _sidata
.extern _sdata
.extern _edata
.extern _sbss
.extern _ebss

/* C functions */
.extern SystemInit
.extern main

/* ---------------- Vector table ---------------- */
.section .isr_vector, "a", %progbits
.align 2
.global g_pfnVectors
.type   g_pfnVectors, %object

g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word 0
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler

  /* External IRQs (exact Keil order/names) */
  .word BOD_IRQHandler
  .word WDT_IRQHandler
  .word EINT0_IRQHandler
  .word EINT1_IRQHandler
  .word GPAB_IRQHandler
  .word GPCDE_IRQHandler
  .word PWMA_IRQHandler
  .word PWMB_IRQHandler
  .word TMR0_IRQHandler
  .word TMR1_IRQHandler
  .word TMR2_IRQHandler
  .word TMR3_IRQHandler
  .word UART02_IRQHandler
  .word UART1_IRQHandler
  .word SPI0_IRQHandler
  .word SPI1_IRQHandler
  .word SPI2_IRQHandler
  .word SPI3_IRQHandler
  .word I2C0_IRQHandler
  .word I2C1_IRQHandler
  .word CAN0_IRQHandler
  .word Reserved22_IRQHandler
  .word Reserved23_IRQHandler
  .word USBD_IRQHandler
  .word PS2_IRQHandler
  .word ACMP_IRQHandler
  .word PDMA_IRQHandler
  .word I2S_IRQHandler
  .word PWRWU_IRQHandler
  .word ADC_IRQHandler
  .word Reserved30_IRQHandler
  .word RTC_IRQHandler
.size g_pfnVectors, .-g_pfnVectors

/* ---------------- Reset ---------------- */
.text
.thumb
.align 2
.global Reset_Handler
.type Reset_Handler, %function
Reset_Handler:
/* .data copy */
  ldr r0, =_sidata
  ldr r1, =_sdata
  ldr r2, =_edata
1:
  cmp r1, r2
  bge 2f
  ldr r3, [r0]
  adds r0, #4          /* or: adds r0, r0, #4 */
  str r3, [r1]
  adds r1, #4
  b   1b
2:

/* .bss zero */
  ldr r1, =_sbss
  ldr r2, =_ebss
  movs r3, #0
3:
  cmp r1, r2
  bge 4f
  str r3, [r1]
  adds r1, #4
  b   3b
4:
  /* System init and main */
  bl  SystemInit
  bl  main
5: b   5b
.size Reset_Handler, .-Reset_Handler

/* ---------------- Default & weak handlers ---------------- */
.global Default_Handler
.type Default_Handler, %function
Default_Handler:
  b .

/* Core exceptions */
.weak NMI_Handler
.thumb_set NMI_Handler, Default_Handler
.weak HardFault_Handler
.thumb_set HardFault_Handler, Default_Handler
.weak SVC_Handler
.thumb_set SVC_Handler, Default_Handler
.weak PendSV_Handler
.thumb_set PendSV_Handler, Default_Handler
.weak SysTick_Handler
.thumb_set SysTick_Handler, Default_Handler

/* External IRQs */
.weak BOD_IRQHandler
.thumb_set BOD_IRQHandler, Default_Handler
.weak WDT_IRQHandler
.thumb_set WDT_IRQHandler, Default_Handler
.weak EINT0_IRQHandler
.thumb_set EINT0_IRQHandler, Default_Handler
.weak EINT1_IRQHandler
.thumb_set EINT1_IRQHandler, Default_Handler
.weak GPAB_IRQHandler
.thumb_set GPAB_IRQHandler, Default_Handler
.weak GPCDE_IRQHandler
.thumb_set GPCDE_IRQHandler, Default_Handler
.weak PWMA_IRQHandler
.thumb_set PWMA_IRQHandler, Default_Handler
.weak PWMB_IRQHandler
.thumb_set PWMB_IRQHandler, Default_Handler
.weak TMR0_IRQHandler
.thumb_set TMR0_IRQHandler, Default_Handler
.weak TMR1_IRQHandler
.thumb_set TMR1_IRQHandler, Default_Handler
.weak TMR2_IRQHandler
.thumb_set TMR2_IRQHandler, Default_Handler
.weak TMR3_IRQHandler
.thumb_set TMR3_IRQHandler, Default_Handler
.weak UART02_IRQHandler
.thumb_set UART02_IRQHandler, Default_Handler
.weak UART1_IRQHandler
.thumb_set UART1_IRQHandler, Default_Handler
.weak SPI0_IRQHandler
.thumb_set SPI0_IRQHandler, Default_Handler
.weak SPI1_IRQHandler
.thumb_set SPI1_IRQHandler, Default_Handler
.weak SPI2_IRQHandler
.thumb_set SPI2_IRQHandler, Default_Handler
.weak SPI3_IRQHandler
.thumb_set SPI3_IRQHandler, Default_Handler
.weak I2C0_IRQHandler
.thumb_set I2C0_IRQHandler, Default_Handler
.weak I2C1_IRQHandler
.thumb_set I2C1_IRQHandler, Default_Handler
.weak CAN0_IRQHandler
.thumb_set CAN0_IRQHandler, Default_Handler
.weak Reserved22_IRQHandler
.thumb_set Reserved22_IRQHandler, Default_Handler
.weak Reserved23_IRQHandler
.thumb_set Reserved23_IRQHandler, Default_Handler
.weak USBD_IRQHandler
.thumb_set USBD_IRQHandler, Default_Handler
.weak PS2_IRQHandler
.thumb_set PS2_IRQHandler, Default_Handler
.weak ACMP_IRQHandler
.thumb_set ACMP_IRQHandler, Default_Handler
.weak PDMA_IRQHandler
.thumb_set PDMA_IRQHandler, Default_Handler
.weak I2S_IRQHandler
.thumb_set I2S_IRQHandler, Default_Handler
.weak PWRWU_IRQHandler
.thumb_set PWRWU_IRQHandler, Default_Handler
.weak ADC_IRQHandler
.thumb_set ADC_IRQHandler, Default_Handler
.weak Reserved30_IRQHandler
.thumb_set Reserved30_IRQHandler, Default_Handler
.weak RTC_IRQHandler
.thumb_set RTC_IRQHandler, Default_Handler
