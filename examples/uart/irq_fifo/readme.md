This is an example for how to input/output data via UART0 connected at pin 23 (RX) and pin 22 (TX) using interrupts

The example needs following defines enabled in RTE_Device.h:
```
#define APOLLOUART0_ENABLED   1
#define APOLLOGPIO_ENABLED    1
#define APOLLOUART0_IRQ_ENABLED 1
```
