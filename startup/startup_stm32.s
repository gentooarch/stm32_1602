/**
  ******************************************************************************
  * @file      startup_stm32.s
  * @author    Ac6
  * @version   V1.0.0
  * @date      12-June-2014
  ******************************************************************************
  */

  .syntax unified
  .cpu cortex-m3
  .thumb

.global	g_pfnVectors
.global	Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word	_sidata
/* start address for the .data section. defined in linker script */
.word	_sdata
/* end address for the .data section. defined in linker script */
.word	_edata
/* start address for the .bss section. defined in linker script */
.word	_sbss
/* end address for the .bss section. defined in linker script */
.word	_ebss

.equ  BootRAM,        0xF1E0F85F
/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval : None
*/

    .section	.text.Reset_Handler
	.weak	Reset_Handler
	.type	Reset_Handler, %function
    .thumb_func          /* ← 添加这一行 */
Reset_Handler:

/* Copy the data segment initializers from flash to SRAM */
  movs	r1, #0
  b	LoopCopyDataInit

CopyDataInit:
	ldr	r3, =_sidata
	ldr	r3, [r3, r1]
	str	r3, [r0, r1]
	adds	r1, r1, #4

LoopCopyDataInit:
	ldr	r0, =_sdata
	ldr	r3, =_edata
	adds	r2, r0, r1
	cmp	r2, r3
	bcc	CopyDataInit
	ldr	r2, =_sbss
	b	LoopFillZerobss
/* Zero fill the bss segment. */
FillZerobss:
	movs r3, #0
 	str  r3, [r2]
	adds r2, r2, #4

LoopFillZerobss:
	ldr	r3, = _ebss
	cmp	r2, r3
	bcc	FillZerobss

/* Call the clock system intitialization function.*/
    bl  SystemInit
/* Call the application's entry point.*/
	bl	main

LoopForever:
    b LoopForever

.size	Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
    .section	.text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
	b	Infinite_Loop
	.size	Default_Handler, .-Default_Handler
/******************************************************************************
*
* The minimal vector table for a Cortex-M.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
 	.section	.isr_vector,"a",%progbits
	.type	g_pfnVectors, %object
	.size	g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
	.word	_estack
	.word	Reset_Handler
	.word	NMI_Handler
	.word	HardFault_Handler
	.word	MemManage_Handler
	.word	BusFault_Handler
	.word	UsageFault_Handler
	.word	0
	.word	0
	.word	0
	.word	0
	.word	SVC_Handler
	.word	DebugMon_Handler
	.word	0
	.word	PendSV_Handler
	.word	SysTick_Handler
	.word	WWDG_IRQHandler              /*!<  0: Window Watchdog                      */
	.word   PVD_IRQHandler               /*!<  1: PVD through EXTI Line detect         */
	.word   TAMPER_IRQHandler            /*!<  2: Tamper                               */
	.word   RTC_IRQHandler               /*!<  3: RTC                                  */
	.word   FLASH_IRQHandler             /*!<  4: Flash                                */
	.word   RCC_IRQHandler               /*!<  5: RCC                                  */
	.word   EXTI0_IRQHandler             /*!<  6: EXTI Line 0                          */
	.word   EXTI1_IRQHandler             /*!<  7: EXTI Line 1                          */
	.word   EXTI2_IRQHandler             /*!<  8: EXTI Line 2                          */
	.word   EXTI3_IRQHandler             /*!<  9: EXTI Line 3                          */
	.word   EXTI4_IRQHandler             /*!< 10: EXTI Line 4                          */
	.word   DMA1_Channel1_IRQHandler     /*!< 11: DMA1 Channel 1                       */
	.word   DMA1_Channel2_IRQHandler     /*!< 12: DMA1 Channel 2                       */
	.word   DMA1_Channel3_IRQHandler     /*!< 13: DMA1 Channel 3                       */
	.word   DMA1_Channel4_IRQHandler     /*!< 14: DMA1 Channel 4                       */
	.word   DMA1_Channel5_IRQHandler     /*!< 15: DMA1 Channel 5                       */
	.word   DMA1_Channel6_IRQHandler     /*!< 16: DMA1 Channel 6                       */
	.word   DMA1_Channel7_IRQHandler     /*!< 17: DMA1 Channel 7                       */
	.word   ADC1_2_IRQHandler            /*!< 18: ADC1 & ADC2                          */
	.word   USB_HP_CAN1_TX_IRQHandler    /*!< 19: USB High Priority or CAN1 TX         */
	.word   USB_LP_CAN1_RX0_IRQHandler   /*!< 20: USB Low  Priority or CAN1 RX0        */
	.word   CAN1_RX1_IRQHandler          /*!< 21: CAN1 RX1                             */
	.word   CAN1_SCE_IRQHandler          /*!< 22: CAN1 SCE                             */
	.word   EXTI9_5_IRQHandler           /*!< 23: EXTI Line 9..5                       */
	.word   TIM1_BRK_IRQHandler          /*!< 24: TIM1 Break                           */
	.word   TIM1_UP_IRQHandler           /*!< 25: TIM1 Update                          */
	.word   TIM1_TRG_COM_IRQHandler      /*!< 26: TIM1 Trigger and Commutation         */
	.word   TIM1_CC_IRQHandler           /*!< 27: TIM1 Capture Compare                 */
	.word   TIM2_IRQHandler              /*!< 28: TIM2                                 */
	.word   TIM3_IRQHandler              /*!< 29: TIM3                                 */
	.word   TIM4_IRQHandler              /*!< 30: TIM4                                 */
	.word   I2C1_EV_IRQHandler           /*!< 31: I2C1 Event                           */
	.word   I2C1_ER_IRQHandler           /*!< 32: I2C1 Error                           */
	.word   I2C2_EV_IRQHandler           /*!< 33: I2C2 Event                           */
	.word   I2C2_ER_IRQHandler           /*!< 34: I2C2 Error                           */
	.word   SPI1_IRQHandler              /*!< 35: SPI1                                 */
	.word   SPI2_IRQHandler              /*!< 36: SPI2                                 */
	.word   USART1_IRQHandler            /*!< 37: USART1                               */
	.word   USART2_IRQHandler            /*!< 38: USART2                               */
	.word   USART3_IRQHandler            /*!< 39: USART3                               */
	.word   EXTI15_10_IRQHandler         /*!< 40: EXTI Line 15..10                     */
	.word   RTCAlarm_IRQHandler          /*!< 41: RTC Alarm through EXTI Line          */
	.word   USBWakeUp_IRQHandler         /*!< 42: USB Wakeup from suspend              */
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

  	.weak	NMI_Handler
	.thumb_set NMI_Handler,Default_Handler

  	.weak	HardFault_Handler
	.thumb_set HardFault_Handler,Default_Handler

  	.weak	MemManage_Handler
	.thumb_set MemManage_Handler,Default_Handler

  	.weak	BusFault_Handler
	.thumb_set BusFault_Handler,Default_Handler

	.weak	UsageFault_Handler
	.thumb_set UsageFault_Handler,Default_Handler

	.weak	SVC_Handler
	.thumb_set SVC_Handler,Default_Handler

	.weak	DebugMon_Handler
	.thumb_set DebugMon_Handler,Default_Handler

	.weak	PendSV_Handler
	.thumb_set PendSV_Handler,Default_Handler

	.weak	SysTick_Handler
	.thumb_set SysTick_Handler,Default_Handler

	.weak	WWDG_IRQHandler            
  .thumb_set WWDG_IRQHandler,Default_Handler
  
	.weak PVD_IRQHandler              
  .thumb_set PVD_IRQHandler,Default_Handler
	
  .weak TAMPER_IRQHandler
  .thumb_set TAMPER_IRQHandler,Default_Handler
	
  .weak RTC_IRQHandler              
  .thumb_set RTC_IRQHandler,Default_Handler
	
  .weak FLASH_IRQHandler            
  .thumb_set FLASH_IRQHandler,Default_Handler
	
  .weak RCC_IRQHandler              
  .thumb_set RCC_IRQHandler,Default_Handler
	
  .weak EXTI0_IRQHandler            
  .thumb_set EXTI0_IRQHandler,Default_Handler
	
  .weak EXTI1_IRQHandler            
  .thumb_set EXTI1_IRQHandler,Default_Handler
	
  .weak EXTI2_IRQHandler            
  .thumb_set EXTI2_IRQHandler,Default_Handler
	
  .weak EXTI3_IRQHandler            
  .thumb_set EXTI3_IRQHandler,Default_Handler
	
  .weak EXTI4_IRQHandler            
  .thumb_set EXTI4_IRQHandler,Default_Handler
	
  .weak DMA1_Channel1_IRQHandler    
  .thumb_set DMA1_Channel1_IRQHandler,Default_Handler
	
  .weak DMA1_Channel2_IRQHandler    
  .thumb_set DMA1_Channel2_IRQHandler,Default_Handler
	
  .weak DMA1_Channel3_IRQHandler    
  .thumb_set DMA1_Channel3_IRQHandler,Default_Handler
	
  .weak DMA1_Channel4_IRQHandler    
  .thumb_set DMA1_Channel4_IRQHandler,Default_Handler
	
  .weak DMA1_Channel5_IRQHandler    
  .thumb_set DMA1_Channel5_IRQHandler,Default_Handler
	
  .weak DMA1_Channel6_IRQHandler    
  .thumb_set DMA1_Channel6_IRQHandler,Default_Handler
	
  .weak DMA1_Channel7_IRQHandler    
  .thumb_set DMA1_Channel7_IRQHandler,Default_Handler
	
  .weak ADC1_2_IRQHandler           
  .thumb_set ADC1_2_IRQHandler,Default_Handler
	
  .weak USB_HP_CAN1_TX_IRQHandler   
  .thumb_set USB_HP_CAN1_TX_IRQHandler,Default_Handler
	
  .weak USB_LP_CAN1_RX0_IRQHandler  
  .thumb_set USB_LP_CAN1_RX0_IRQHandler,Default_Handler
	
  .weak CAN1_RX1_IRQHandler         
  .thumb_set CAN1_RX1_IRQHandler,Default_Handler
	
  .weak CAN1_SCE_IRQHandler         
  .thumb_set CAN1_SCE_IRQHandler,Default_Handler
	
  .weak EXTI9_5_IRQHandler          
  .thumb_set EXTI9_5_IRQHandler,Default_Handler
	
  .weak TIM1_BRK_IRQHandler         
  .thumb_set TIM1_BRK_IRQHandler,Default_Handler
	
  .weak TIM1_UP_IRQHandler          
  .thumb_set TIM1_UP_IRQHandler,Default_Handler
	
  .weak TIM1_TRG_COM_IRQHandler     
  .thumb_set TIM1_TRG_COM_IRQHandler,Default_Handler
	
  .weak TIM1_CC_IRQHandler          
  .thumb_set TIM1_CC_IRQHandler,Default_Handler
	
  .weak TIM2_IRQHandler             
  .thumb_set TIM2_IRQHandler,Default_Handler
	
  .weak TIM3_IRQHandler             
  .thumb_set TIM3_IRQHandler,Default_Handler
	
  .weak TIM4_IRQHandler             
  .thumb_set TIM4_IRQHandler,Default_Handler
	
  .weak I2C1_EV_IRQHandler          
  .thumb_set I2C1_EV_IRQHandler,Default_Handler
	
  .weak I2C1_ER_IRQHandler          
  .thumb_set I2C1_ER_IRQHandler,Default_Handler
	
  .weak I2C2_EV_IRQHandler          
  .thumb_set I2C2_EV_IRQHandler,Default_Handler
	
  .weak I2C2_ER_IRQHandler          
  .thumb_set I2C2_ER_IRQHandler,Default_Handler
	
  .weak SPI1_IRQHandler             
  .thumb_set SPI1_IRQHandler,Default_Handler
	
  .weak SPI2_IRQHandler             
  .thumb_set SPI2_IRQHandler,Default_Handler
	
  .weak USART1_IRQHandler           
  .thumb_set USART1_IRQHandler,Default_Handler
	
  .weak USART2_IRQHandler           
  .thumb_set USART2_IRQHandler,Default_Handler
	
  .weak USART3_IRQHandler           
  .thumb_set USART3_IRQHandler,Default_Handler
  
  .weak EXTI15_10_IRQHandler        
  .thumb_set EXTI15_10_IRQHandler,Default_Handler
  
  .weak RTCAlarm_IRQHandler         
  .thumb_set RTCAlarm_IRQHandler,Default_Handler
  
  .weak USBWakeUp_IRQHandler        
  .thumb_set USBWakeUp_IRQHandler,Default_Handler
 
	.weak	SystemInit

/************************ (C) COPYRIGHT Ac6 *****END OF FILE****/
