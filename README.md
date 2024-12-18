# APDS9930 Ambient Light and Proximity Sensor 
This driver was developed for STM32 MCUs, and specifically for the STM32F302R8. For the sensor, I used a breakout board from Keyestudio 48 in 1 Sensor Kit. 

Header file is in */Core/Inc*, and Source file is in */Core/Src*. To use in non STM32F3s, the indcluded header file in line 13 of *APDS9930.h* should be changed to your version. For example, if using an STM32F4, change that file to "stm32f4xx_hal.h".

By defauly, sensor intilization function only enables proximity as interrupt, but functions and macros are available to customize the configuration of the sensor.

## Links to documentation
- [Product Page](https://www.broadcom.com/products/optical-sensors/integrated-ambient-light-proximity-sensors/apds-9930)
- [Datasheet](https://github.com/morena1001/APDS-9930-Driver/blob/main/APDS_9930_Datasheet.pdf)

## Getting Started
This section includes the pins connected to the sensor .ioc file configurations.

| STM32 Pin   | APDS9930 Pin | Function    |
| ----------- | -----------  | ----------- |
| 3.3VL       | 3.3V         | LED Power   |
| 3.3V        | 3.3V         | Power       |
| GND         | GND          | Ground      |
| PB7         | SDA          | I^2^C Data  |
| PA15        | SCL          | I^2^C Clock |
| PA0         | INT          | Interrupt   |

**PA0** :
- GPIO mode is *External Interrupt Mode with Falling edge trigger detection*
- Has an internal *Pull up*

**PA15** : 
- GPIO mode is *Alternate Function Open Drain*
- Has *No pull-up and no pull-down*
- Has a *High* Maximum output speed

**PB7** : 
- GPIO mode is *Alternate Function Open Drain*
- Has *No pull-up and no pull-down*
- Has a *High* Maximum output speed
- Fast mode is *Disabled*

**NVIC**
- EXTI line0 interrupt is *Enabled*

**SYS**
- Debug is set to *Serial Wire*
 
**I2C1**
- Speed mode is set to *Fast Mode*
- Speed Frequency is set to *400*KHz
- Primary address length selection is in its default setting, *7-bit*

**USART 2** **OPTIONAL**
- Mode is *Asynchronous*

## Version History
- Initial final product
- Removed High Speed Clock, not needed
- Better configuration capabilities
- 3.3VL for LED power was needed after all
- Fixed Prox threshold setting issues

## License Information
This project is **open source**.

Feel free to use hoever you see fit, and if you have any questions regarding the driver, feel free to e-mail me.
