/*Author: Danil Fomin */

typedef unsigned int uint32_t;

//Assign external variables
extern const int main();
//extern const void SPI1_IRQHandler();
//End of stack
extern uint32_t _estack;
// .data section addresses. _basedata - adress in FLASH, need to copy to RAM               
extern uint32_t _sdata, _edata, _basedata;   
// .bss section addresses 
extern uint32_t _sbss, _ebss;

extern uint32_t __heap_size, __bss_start__, __bss_end__, end, __end__, __heap_addr, __HeapBase, __HeapLimit;


//SRAM is unpredicted every time MCU restart
//Copy .data section to RAM
static inline void copy_data_to_ram(void){
    //Clear up BSS section
    unsigned int *iter = &_sbss;
    // unsigned int *end = &_ebss;
    do {
        *iter = 0;
        //iter++;
    } while (iter++ != &_ebss);

    //Load .data content from FLASH to RAM
    //_basedata -> address in RAM
    unsigned int *base = &_basedata;
    iter = &_sdata;
    do {
        *iter = *base++;
    } while(iter++ != &_edata);

}

void Default_Handler(void){
    __asm("nop");
    __builtin_unreachable();
}

//Yep, that a reset handler goes right at 0x04 program counter
__attribute__((weak, noreturn))
void Reset_Handler(void) {
    copy_data_to_ram();
    __asm("b main");
    __builtin_unreachable();
}

const void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
const void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
const void MemManage_Handler(void) __attribute__((weak, alias("Default_Handler")));
const void BusFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
const void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));

const void SVCall_Handler(void) __attribute__((weak, alias("Default_Handler")));
const void DebugMonitor_Handler(void) __attribute__((weak, alias("Default_Handler")));

const void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));
const void SysTick_Handler(void) __attribute__((weak, alias("Default_Handler")));
const void WWDG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void PVD_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void TAMPER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void RTC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void FLASH_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void RCC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void EXTI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void EXTI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void EXTI2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void EXTI3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void EXTI4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void DMA1_Channel1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void DMA1_Channel2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void DMA1_Channel3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void DMA1_Channel4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void DMA1_Channel5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void DMA1_Channel6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void DMA1_Channel7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void ADC1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void USB_HP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void USB_LP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void DAC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void COMP_TSC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void EXTI9_5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void LCD_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void TIM9_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void TIM10_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void TIM11_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void TIM2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void TIM3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void TIM4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void I2C1_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void I2C1_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void I2C2_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void I2C2_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void SPI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void SPI2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void USART1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void USART2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void USART3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void EXTI15_10_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void RTC_Alarm_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void USB_FS_WKUP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void TIM6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void TIM7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
/*
const void TIM1_BRK_TIM9_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void TIM1_UP_TIM10_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void TIM1_TRG_COM_TIM11_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void TIM1_CC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void DMA2_Channel4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void I2C3_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void EXIT18_OTG_FS_WKUP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void I2C3_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void FPU_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void SPI4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void EXTI17_RTC_Alarm_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void DMA2_Channel5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void SPI5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void SDIO_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void SPI3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void UART4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void UART5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void DMA1_Channel7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void DMA2_Channel1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void DMA2_Channel2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void DMA2_Channel3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
const void DMA2_Channel0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
*/
const uint32_t isr_vector[] __attribute__((section(".isr_vector"), used)) = {
    (const uint32_t) &_estack,
    (const uint32_t) &Reset_Handler,
    (const uint32_t) &NMI_Handler,
    (const uint32_t) &HardFault_Handler,
    (const uint32_t) &MemManage_Handler,
    (const uint32_t) &BusFault_Handler,
    (const uint32_t) &UsageFault_Handler,
    0,
    0,
    0,
    0,
    (const uint32_t) &SVCall_Handler,
    (const uint32_t) &DebugMonitor_Handler,
    0,
    (const uint32_t) &PendSV_Handler,
    (const uint32_t) &SysTick_Handler,
    (const uint32_t) &WWDG_IRQHandler,
    (const uint32_t) &PVD_IRQHandler,
    (const uint32_t) &TAMPER_IRQHandler,
    (const uint32_t) &RTC_IRQHandler,
    (const uint32_t) &FLASH_IRQHandler,
    (const uint32_t) &RCC_IRQHandler,
    (const uint32_t) &EXTI0_IRQHandler,
    (const uint32_t) &EXTI1_IRQHandler,
    (const uint32_t) &EXTI2_IRQHandler,
    (const uint32_t) &EXTI3_IRQHandler,
    (const uint32_t) &EXTI4_IRQHandler,
    (const uint32_t) &DMA1_Channel1_IRQHandler,
    (const uint32_t) &DMA1_Channel2_IRQHandler,
    (const uint32_t) &DMA1_Channel3_IRQHandler,
    (const uint32_t) &DMA1_Channel4_IRQHandler,
    (const uint32_t) &DMA1_Channel5_IRQHandler,
    (const uint32_t) &DMA1_Channel6_IRQHandler,
    (const uint32_t) &DMA1_Channel7_IRQHandler,
    (const uint32_t) &ADC1_IRQHandler,
    (const uint32_t) &USB_HP_IRQHandler,
    (const uint32_t) &USB_LP_IRQHandler,
    (const uint32_t) &DAC_IRQHandler,
    (const uint32_t) &COMP_TSC_IRQHandler,
    (const uint32_t) &EXTI9_5_IRQHandler,
    (const uint32_t) &LCD_IRQHandler,
    (const uint32_t) &TIM9_IRQHandler,
    (const uint32_t) &TIM10_IRQHandler,
    (const uint32_t) &TIM11_IRQHandler,
    (const uint32_t) &TIM2_IRQHandler,
    (const uint32_t) &TIM3_IRQHandler,
    (const uint32_t) &TIM4_IRQHandler,
    (const uint32_t) &I2C1_EV_IRQHandler,
    (const uint32_t) &I2C1_ER_IRQHandler,
    (const uint32_t) &I2C2_EV_IRQHandler,
    (const uint32_t) &I2C2_ER_IRQHandler,
    (const uint32_t) &SPI1_IRQHandler,
    (const uint32_t) &SPI2_IRQHandler,
    (const uint32_t) &USART1_IRQHandler,
    (const uint32_t) &USART2_IRQHandler,
    (const uint32_t) &USART3_IRQHandler,
    (const uint32_t) &EXTI15_10_IRQHandler,
    (const uint32_t) &RTC_Alarm_IRQHandler,
    (const uint32_t) &USB_FS_WKUP_IRQHandler,
    (const uint32_t) &TIM6_IRQHandler,
    (const uint32_t) &TIM7_IRQHandler,
};

// //Defining memory laout ))
// typedef struct{
//     const unsigned int *stack_top;
//     const void (*reset);
//     const void (*NMI_function);
//     const void (*HardFault_function);
//     const void (*MemManage_function);
//     const void (*BusFault_function);
//     const void (*UsageFault_function);

//     const unsigned int Reserved1[4];

//     const void (*SVCall_function);
//     const void (*DebugMonitor_function);
    
//     const unsigned int Reserved2;

//     const void (*PendSV_function);
//     const void (*SysTick_function);
//     const void (*WWDG_function);
//     const void (*PVD_function);
//     const void (*TAMPER_function);
//     const void (*RTC_function);
//     const void (*FLASH_function);
//     const void (*RCC_function);
//     const void (*EXTI0_function);
//     const void (*EXTI1_function);
//     const void (*EXTI2_function);
//     const void (*EXTI3_function);
//     const void (*EXTI4_function);
//     const void (*DMA1_Channel1_function);
//     const void (*DMA1_Channel2_function);
//     const void (*DMA1_Channel3_function);
//     const void (*DMA1_Channel4_function);
//     const void (*DMA1_Channel5_function);
//     const void (*DMA1_Channel6_function);
//     const void (*DMA1_Channel7_function);
//     const void (*ADC1_2_function);
//     const void (*USB_HP_CAN_TX_function);
//     const void (*USB_LP_CAN_RX0_function);
//     const void (*CAN_RX1_function);
//     const void (*CAN_SCE_function);
//     const void (*EXTI9_5_function);
//     const void (*TIM1_BRK_function);
//     const void (*TIM1_UP_function);
//     const void (*TIM1_TRG_COM_function);
//     const void (*TIM1_CC_function);
//     const void (*TIM2_function);
//     const void (*TIM3_function);
//     const void (*TIM4_function);
//     const void (*I2C1_EV_function);
//     const void (*I2C1_ER_function);
//     const void (*I2C2_EV_function);
//     const void (*I2C2_ER_function);
//     const void (*SPI1_function);
//     const void (*SPI2_function);
//     const void (*USART1_function);
//     const void (*USART2_function);
//     const void (*USART3_function);
//     const void (*EXTI15_10_function);
//     const void (*RTCAlarm_function);
//     const void (*USBWakeup_function);
//     const void (*TIM8_BRK_function);
//     const void (*TIM8_UP_function);
//     const void (*TIM8_TRG_COM_function);
//     const void (*TIM8_CC_function);
//     const void (*ADC3_function);
//     const void (*FSMC_function);
//     const void (*SDIO_function);
//     const void (*TIM5_function);
//     const void (*SPI3_function);
//     const void (*UART4_function);
//     const void (*UART5_function);
//     const void (*TIM6_function);
//     const void (*TIM7_function);
//     const void (*DMA2_Channel1_function);
//     const void (*DMA2_Channel2_function);
//     const void (*DMA2_Channel3_function);
//     const void (*DMA2_Channel4_5_function);
// }isr_typedef;
/*/
//Yes, that our interrupt servise routine
__attribute__((section(".isr_vector"), used))
const isr_typedef isr_vector = {
    .stack_top =    &_estack,
    .reset =        &Reset_Handler,
    .NMI_function = &hang,
    .HardFault_function =   &hang,
    .MemManage_function =   &hang,
    .BusFault_function  =   &hang,
    .UsageFault_function=   &hang,
    .SVCall_function    =   &hang,
    .DebugMonitor_function= &hang,
    .PendSV_function    =   &hang,
    .SysTick_function   =   &hang,
    
    //.USB_LP_CAN_RX0_function = &USB_LP_CAN_RX0_IRQn,
};

*/
/*
typedef void (*reset_function)();
typedef void (*NMI_function)();
typedef void (*HardFault_function)();
typedef void (*MemManage_function)();
typedef void (*BusFault_function)();
typedef void (*UsageFault_function)();
typedef void (*SVCall_function)();
typedef void (*DebugMonitor_function)();
typedef void (*PendSV_function)();
typedef void (*SysTick_function)();
typedef void (*WWDG_function)();
typedef void (*PVD_function)();
typedef void (*TAMPER_function)();
typedef void (*RTC_function)();
typedef void (*FLASH_function)();
typedef void (*EXTI0_function)();
typedef void (*EXTI1_function)();
typedef void (*EXTI2_function)();
typedef void (*EXTI3_function)();
typedef void (*EXTI4_function)();
typedef void (*DMA1_Channel1_function)();
typedef void (*DMA1_Channel2_function)();
typedef void (*DMA1_Channel3_function)();
typedef void (*DMA1_Channel4_function)();
typedef void (*DMA1_Channel5_function)();
typedef void (*DMA1_Channel6_function)();
typedef void (*DMA1_Channel7_function)();
typedef void (*ADC1_2_function)();
typedef void (*USB_HP_CAN_TX_function)();
typedef void (*USB_LP_CAN_RX0_function)();
typedef void (*CAN_RX1_function)();
typedef void (*CAN_SCE_function)();
typedef void (*EXTI9_5_function)();
typedef void (*TIM1_BRK_function)();
typedef void (*TIM1_UP_function)();
typedef void (*TIM1_TRG_COM_function)();
typedef void (*TIM1_CC_function)();
typedef void (*TIM2_function)();
typedef void (*TIM3_function)();
typedef void (*TIM4_function)();
typedef void (*I2C1_EV_function)();
typedef void (*I2C1_ER_function)();
typedef void (*I2C2_EV_function)();
typedef void (*I2C2_ER_function)();
typedef void (*SPI1_function)();
typedef void (*SPI2_function)();
typedef void (*USART1_function)();
typedef void (*USART2_function)();
typedef void (*USART3_function)();
typedef void (*EXTI15_10_function)();
typedef void (*RTCAlarm_function)();
typedef void (*USBWakeup_function)();
typedef void (*TIM8_BRK_function)();
typedef void (*TIM8_UP_function)();
typedef void (*TIM8_TRG_COM_function)();
typedef void (*TIM8_CC_function)();
typedef void (*ADC3_function)();
typedef void (*FSMC_function)();
typedef void (*SDIO_function)();
typedef void (*TIM5_function)();
typedef void (*SPI3_function)();
typedef void (*UART4_function)();
typedef void (*UART5_function)();
typedef void (*TIM6_function)();
typedef void (*TIM7_function)();
typedef void (*DMA2_Channel1_function)();
typedef void (*DMA2_Channel2_function)();
typedef void (*DMA2_Channel3_function)();
typedef void (*DMA2_Channel4_5_function)();
*/
