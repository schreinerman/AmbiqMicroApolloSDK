Needs following modules enabled in RTE_Device.h (in the example\source\config folder)
```
#define APOLLOUART0_ENABLED   1
#define APOLLOGPIO_ENABLED    1
```

This is an example for output via UART0 connected at pin 23 (RX) and pin 22 (TX).

The example is using the MCU Template for Apollo2 from http://www.feeu.com/apollo2
and the low-level drivers of FEEU with latest sources available
at https://github.com/schreinerman/AmbiqMicroApolloSDK
