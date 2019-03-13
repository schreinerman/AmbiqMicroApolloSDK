This example is demonstrating SPI.

GPIO Connection:
- GPIO5 SCK pin
- GPIO6 MISO pin
- GPIO7 MOSI pin
- GPIO42 CS pin

Software Flow:
- Sending 2 bytes in au8DataOut, 
- receiving 20 bytes with pullup on MISO, 
- receiving 20 bytes without pullup on MISO
- and keeping CS low during all transfers

Following defines are enabled in RTE_Device.h:
```
#define APOLLOGPIO_ENABLED    1
#define IOMSTR0_ENABLED       1
```

Optionally enabled in RTE_Device.h for debugging:
```
#define DEBUG_OUTPUT 1
#define IOM_DEBUG    1
```
    
