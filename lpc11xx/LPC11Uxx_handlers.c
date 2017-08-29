/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2010, Roel Verdult
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

// The GCC compiler defines the current architecture derived from the -mcpu argument.
// When target cpu is the cortex-m0, it automatically defines __ARM_ARCH_6M__
// Some versions define __ARM_ARCH_6SM__ instead
#if !defined(__ARM_ARCH_6M__) && !defined(__ARM_ARCH_6SM__)
  #error "The target ARM cpu must be Cortex-M0 compatible (-mcpu=cortex-m0)"
#endif

// Declare a weak alias macro as described in the GCC manual[1][2]
#define WEAK_ALIAS(f) __attribute__ ((weak, alias (#f)));
#define SECTION(s) __attribute__ ((section(s)))

/******************************************************************************
 * Forward undefined IRQ handlers to an infinite loop function. The Handlers
 * are weakly aliased which means that (re)definitions will overide these.
 *****************************************************************************/
#include <LPC11Uxx.h>

void irq_undefined() {
	while(1);
}


void FLEX_INT0_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void FLEX_INT1_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void FLEX_INT2_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void FLEX_INT3_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void FLEX_INT4_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void FLEX_INT5_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void FLEX_INT6_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void FLEX_INT7_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void GINT0_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void GINT1_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void SSP1_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void I2C_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void TIMER16_0_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void TIMER16_1_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void TIMER32_0_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void TIMER32_1_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void SSP0_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void UART_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void USB_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void USB_FIQHandler (void) WEAK_ALIAS(irq_undefined);
void ADC_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void WDT_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void BOD_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void FMC_IRQHandler (void) WEAK_ALIAS(irq_undefined);
void USBWakeup_IRQHandler (void) WEAK_ALIAS(irq_undefined);


/*****************************************************************************
 * Forward undefined fault handlers to an infinite loop function. The Handlers
 * are weakly aliased which means that (re)definitions will overide these.
 ****************************************************************************/
extern __attribute((weak)) void fault_handler() {}

void fault_undefined() {
	fault_handler();
	while(1);
}

void NMI_Handler(void)          WEAK_ALIAS(fault_undefined);
void HardFault_Handler(void)    WEAK_ALIAS(fault_undefined);
void SVCall_Handler(void)       WEAK_ALIAS(fault_undefined);
void PendSV_Handler(void)       WEAK_ALIAS(fault_undefined);
void SysTick_Handler(void)      WEAK_ALIAS(fault_undefined);

/******************************************************************************
 * Forward undefined IRQ handlers to an infinite loop function. The Handlers
 * are weakly aliased which means that (re)definitions will overide these.
 *****************************************************************************/

// Prototype the entry values, which are handled by the linker script
extern uint32_t __main_stack_end__;
extern void Reset_Handler(void);

// Defined irq vectors using simple c code following the description in a white
// paper from ARM[3] and code example from Simonsson Fun Technologies[4].
// These vectors are placed at the memory location defined in the linker script
const void *vectors[] SECTION(".irq_vectors") =
{
  // Stack and program reset entry point
  &__main_stack_end__,          // The initial stack pointer
  Reset_Handler,            // The reset handler

  NMI_Handler, // The NMI handler
  HardFault_Handler, // The hard fault handler
  0, // Reserved
  0, // Reserved
  0, // Reserved
  0, // Reserved
  0, // Reserved
  0, // Reserved
  0, // Reserved
  SVCall_Handler, // SVCall handler
  0, // Reserved
  0, // Reserved
  PendSV_Handler, // The PendSV handler
  SysTick_Handler, // The SysTick handler

  // LPC11U specific handlers
  FLEX_INT0_IRQHandler, // 0 - GPIO pin interrupt 0
  FLEX_INT1_IRQHandler, // 1 - GPIO pin interrupt 1
  FLEX_INT2_IRQHandler, // 2 - GPIO pin interrupt 2
  FLEX_INT3_IRQHandler, // 3 - GPIO pin interrupt 3
  FLEX_INT4_IRQHandler, // 4 - GPIO pin interrupt 4
  FLEX_INT5_IRQHandler, // 5 - GPIO pin interrupt 5
  FLEX_INT6_IRQHandler, // 6 - GPIO pin interrupt 6
  FLEX_INT7_IRQHandler, // 7 - GPIO pin interrupt 7
  GINT0_IRQHandler, // 8 - GPIO GROUP0 interrupt
  GINT1_IRQHandler, // 9 - GPIO GROUP1 interrupt
  0, // 10 - Reserved
  0, // 11 - Reserved
  0, // 12 - Reserved
  0, // 13 - Reserved
  SSP1_IRQHandler, // 14 - SPI/SSP1 Interrupt
  I2C_IRQHandler, // 15 - I2C0
  TIMER16_0_IRQHandler, // 16 - CT16B0 (16-bit Timer 0)
  TIMER16_1_IRQHandler, // 17 - CT16B1 (16-bit Timer 1)
  TIMER32_0_IRQHandler, // 18 - CT32B0 (32-bit Timer 0)
  TIMER32_1_IRQHandler, // 19 - CT32B1 (32-bit Timer 1)
  SSP0_IRQHandler, // 20 - SPI/SSP0 Interrupt
  UART_IRQHandler, // 21 - UART0
  USB_IRQHandler, // 22 - USB IRQ
  USB_FIQHandler, // 23 - USB FIQ
  ADC_IRQHandler, // 24 - ADC (A/D Converter)
  WDT_IRQHandler, // 25 - WDT (Watchdog Timer)
  BOD_IRQHandler, // 26 - BOD (Brownout Detect)
  FMC_IRQHandler, // 27 - IP2111 Flash Memory Controller
  0, // 28 - Reserved
  0, // 29 - Reserved
  USBWakeup_IRQHandler, // 30 - USB wake-up interrupt
  0, // 31 - Reserved
};

/******************************************************************************
 * References
 *  [1] http://gcc.gnu.org/onlinedocs/gcc/Function-Attributes.html
 *  [2] http://gcc.gnu.org/onlinedocs/gcc/Variable-Attributes.html
 *  [3] http://www.arm.com/files/pdf/Cortex-M3_programming_for_ARM7_developers.pdf
 *  [4] http://fun-tech.se/stm32/OlimexBlinky/mini.php
 *****************************************************************************/
