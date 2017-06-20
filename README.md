# AmbiqMicroApolloSDK
Alternative SDK for Ambiq Micro Apollo 1 and 2 MCU

The alternative SDK for Ambiq Micro Apollo 1 and 2 MCUs is offering CMSIS compatible implementations. In the first step the API is not yet CMSIS driver API compatible, but this feature will be added as next step.

The SDK can be used with FEEU MCU Templates, already configured for the correct device.

If HW shall be enabled, this must be done in RTE_Device.h. Example:

#define IOMSTR0_ENABLED 1

A complete documentation will follow soon.
