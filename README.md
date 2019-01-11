# AmbiqMicroApolloSDK (beta)
Alternative SDK for Ambiq Micro Apollo 1 and 2 MCU from Fujitsu (beta)

The alternative SDK for Ambiq Micro Apollo 1 and 2 MCUs is offering CMSIS compatible implementations. In the first step the API is not yet CMSIS driver API compatible, but this feature will be added as next step.

The SDK can be used with FEEU MCU Templates, already configured for the correct device.

# Usage with MCU Templates
Low-Level drivers are put into <root>\library\lowlevel
Configuration of low-level drivers is done in example\source\config\RTE_Device.h
 
# Configuration
Configuration of low-level drivers is done in example\source\config\RTE_Device.h

## Debug
set DEBUG_OUTPUT to 1 to enable debug output by adding following line in example\source\config\RTE_Device.h 

<code>
#define DEBUG_OUTPUT 1
</code>

Debug print is realized automatically if debugprint is not defined via printf:


<code>
#if (DEBUG_OUTPUT == 1) && (!defined(debugprint))
#define debugprint(...) if ((CoreDebug->DHCSR & (1 << CoreDebug_DHCSR_C_DEBUGEN_Pos)) != 0) printf(__VA_ARGS__)
#define debugprintln(...) debugprint(__VA_ARGS__); debugprint("\r\n")
#endif
</code>

## ADC

Enable ADC by adding following line in example\source\config\RTE_Device.h
<code>
#define APOLLOADC_ENABLED 1
</code>

Enable ADC debug by adding following line in example\source\config\RTE_Device.h
<code>
#define ADC_DEBUG 1
</code>

## CTIMER

Enable all CTIMERs by adding following line in example\source\config\RTE_Device.h
<code>
#define APOLLOCTIMER_ENABLED 1
</code>

Enable just selected CTIMER0..3 by adding following line(s) in example\source\config\RTE_Device.h
<code>
#define CTIMER0_ENABLED 1
#define CTIMER1_ENABLED 1
#define CTIMER2_ENABLED 1
#define CTIMER3_ENABLED 1
</code>

Add the GPIO module (ApolloGpio) and enable it to have advanced functionalities in example\source\config\RTE_Device.h
<code>
#define APOLLOGPIO_ENABLED 1
</code>
  
Enable CTIMER debug by adding following line in example\source\config\RTE_Device.h
<code>
#define CTIMER_DEBUG 1
</code>

## IOM

Enable all IOMs by adding following line in example\source\config\RTE_Device.h
<code>
#define APOLLOIOM_ENABLED 1
</code>

Enable just selected IOM0..5 by adding following line(s) in example\source\config\RTE_Device.h
<code>
#define IOMSTR0_ENABLED 1
#define IOMSTR1_ENABLED 1
#define IOMSTR2_ENABLED 1
#define IOMSTR3_ENABLED 1
#define IOMSTR4_ENABLED 1
#define IOMSTR5_ENABLED 1
</code>

To enable IRQ handling inside of the IOM module (ApolloIom) and possible usage of callbacks initiated by the interrupt, enable APOLLOIOM_USE_IRQS by adding following line in example\source\config\RTE_Device.h
<code>
#define APOLLOIOM_USE_IRQS 1
</code>

Enable IOM debug by adding following line in example\source\config\RTE_Device.h
<code>
#define IOM_DEBUG 1
</code>

## GPIO

Enable by adding following line in example\source\config\RTE_Device.h
<code>
#define APOLLOGPIO_ENABLED 1
</code>

Enable APOLLOGPIO_USE_ARDUINO to add pinMode, attachInterrupt, digitalWrite, digitalRead and detachInterrupt functions by adding following line in example\source\config\RTE_Device.h
<code>
#define APOLLOGPIO_USE_ARDUINO 1
</code>

To enable IRQ handling inside of the GPIO module (ApolloGpio) and possible usage of callbacks initiated by the interrupt, enable APOLLOGPIO_USE_IRQS by adding following line in example\source\config\RTE_Device.h
<code>
#define APOLLOGPIO_USE_IRQS 1
</code>

Enable GPIO debug by adding following line in example\source\config\RTE_Device.h
<code>
#define GPIO_DEBUG 1
</code>

## UART

Enable all UARTs by adding following line in example\source\config\RTE_Device.h
<code>
#define APOLLOUART_ENABLED 1
</code>

Enable just selected UART0..1 by adding following line(s) in example\source\config\RTE_Device.h
<code>
#define APOLLOUART0_ENABLED 1
#define APOLLOUART1_ENABLED 1
</code>

Enable Semihosting via UART by adding following line(s) in example\source\config\RTE_Device.h
<code>
#define UART_SEMIHOST_ENABLED  1
#define UART_SEMIHOST UART0
</code>

Enable CMSIS Driver API by adding following line(s) in example\source\config\RTE_Device.h
<code>
#define USE_CMSIS_DRIVER 1
</code>

Enable UART debug by adding following line in example\source\config\RTE_Device.h
<code>
#define UART_DEBUG 1
</code>

## SYSCTRL

Enable SYSCTRL debug by adding following line in example\source\config\RTE_Device.h
<code>
#define SYSCTRL_DEBUG 1
</code>

## ITMSEMIHOST

Enable ITMSEMIHOST by adding following line in example\source\config\RTE_Device.h
<code>
#define ITMSEMIHOST_ENABLED  1
</code>


A complete documentation will follow soon.
