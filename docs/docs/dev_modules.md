# Optional Modules
The previous sections largely describe a MODAQ BlackBox configuration for basic asset tracking and disturbance monitoring with data transmissions via Iridium SBD. There is sufficient processing processing power and I/O (Input/Output) support to extend this reference design to a wide range of data acquisition (and control) applications. This section is provided to highlight some examples of options users may consider in their own builds. Many of these have been developed into BB variants for projects with simple DAQ requirements or where a compact, rugged, and battery-operable solution was necessary. 

While the ESP32 is a very impressive and capable little device, it does have its limits. Users should conduct their own performance tests to determine highest stable and sustainable sampling rates and number of I/O channels, as well as the impacts of complex of onboard processing, file I/O, and communications with other systems. Fortunately, the ESP32 supports <a href="https://www.freertos.org/index.html" target="_blank">FreeRTOS</a> out of the box[^1] and users can leverage advanced features such as multitasking, queues, semaphores/mutexes, and the ESP32's dual CPU cores. 

## I/O Modules
Input Modules convert signals from sensors or instruments to a communications format supported by the MCU (in this case, the ESP32). These are usually specialized and purposeful devices that may perform considerable onboard processing of the input signals and can be found at many different performance, quality, and price points. The quality of these modules could have more influence on the overall measurement quality than any contributions from the MCU.  

><font color = blue><b>Tech Note:</b> The ESP32 and many other MCUs have built-in I/O. In most cases, these I/O are used to interface with the external I/O modules.
<font color = #404040>

These optional modules can be incorporated into the BB_Core code with a little C++ programming effort. Links to 3rd party libraries and examples are included in each section below. Please study and test the libraries before using in serious or production applications. There are usually multiple sources releasing libraries that may be more/less capable, robust, and/or easy to use. 

### ADC
ADC (Analog to Digital Converter), as the name implies, converts an analog signal, such as a voltage, to a digital representation that can be easily read by the MCU. The ESP32 has a built-in ADC, but it's not very good- even a cheap <a href="https://www.adafruit.com/product/1085" target="_blank">ADS1115</a> is worlds better. 

Before going any further, it's important to recognize that there are many technical aspects of ADC design, selection, and application that require a fundamental, if not advanced, understanding of several topics in electronics and mathematics. ADCs are one area of specialization that some dedicate their entire careers to master. Good results can still be had with the understanding of some <a href="https://ww1.microchip.com/downloads/en/Appnotes/atmel-8456-8-and-32-bit-avr-microcontrollers-avr127-understanding-adc-parameters_application-note.pdf" target="_blank">basic principles</a> of ADCs.

We have incorporated the ADS1115 in BB variants that are acquiring 0-5vdc voltage sources and the ADS1115 is used as the ADC in <a href="https://store.ncd.io/product/4-channel-4-20-ma-current-loop-receiver-16-bit-ads1115-i2c-mini-module/" target="_blank">this</a> 4-20 mA current loop receiver. 

><font color = blue><b>Tech Note:</b> ADCs typically can measure a voltage up to the supply voltage (VDD), meaning that if the ADC is powered by 5v, the measurement range will be 0 - 5v. However, this requires the ADC's PGA (Programmable Gain Amplifier) to be set accordingly. Further, the ESP32 is a 3.3v device and can be damaged if the GPIO pins are subject to voltages in excess of 3.3v. Look closely at the specifications to determine if the logic level connections (I2C, SPI) are regulated to 3.3v or if they track with VDD. There are several approaches to avoid over-volting the ESP32 including supply the ADC with 3.3v and limit the measurement range to 3.3v, sourcing a breakout board with separate supplies for VDD and logic (or boards that have onboard conversion for 3.3v logic level), or use <a href="https://www.sparkfun.com/products/12009" target="_blank">level shifters</a>.<p>If it's necessary to measure voltage ranges greater than VDD, look into breakout boards or ADCs with AFEs (Analog Front End) designed for the desired voltage range- or look into <a href="https://electronics.stackexchange.com/questions/464281/voltage-scaling-of-10-to-10-v-analog-signal-to-0-to-3-v-for-input-to-adc-using" target="_blank">DIY solutions</a>. 
<font color = #404040>

<a href="https://github.com/Wh1teRabbitHU/ADS1115-Driver/tree/master" target="_blank">ADS1X15</a> library and examples

### Load Cells
<img align="right" src="../img/bb_load_link.png" width="350" height="495">Since they share a lot in common with ADCs, load cells can be easily interfaced to BB. We have successfully integrated a <a href="https://sensing-systems.com/wp-content/uploads/2023/05/Submersible-Tension-Link-Load-Cell-Catalog.pdf" target="_blank">submersible anchor load cell</a> with a BB variant based largely on the reference design provided in this repo. This is a strain gage type load cell that effectively behaves as a half-bridge strain gage and is interfaced to the ESP32 through an <a href="https://www.sparkfun.com/products/13879" target="_blank">HX711</a> load cell amplifier using TWI (Two Wire Interface), which is similar to I2C but without the addressing. 

><font color = blue><b>Tech Note:</b>Because the HX711 lacks addressing (and its protocol does not adhere to I2C standards), it takes a bit of work to acquire multiple load cells using this chipset (use additional GPIO ADC pins or a multiplexer module). There are versions of the HX711 breakout boards that expose inputs for a second load cell or the board converts the TWI to compliant I2C and provides configurable addressing, but the one we linked above is not one of those. 
<font color = #404040>

<a href="https://github.com/olkal/HX711_ADC/tree/master" target="_blank">HX711</a> library and examples

### Encoder
Encoders can easily be supported by BB, however users need to consider the signaling or communications protocol of the encoder and select one that is compatible. Many encoders can speak serial (such as RS-232), quadrature, or output a proportional analog voltage or current signal. We have successfully used a <a href="https://ecatalog.dynapar.com/ecatalog/absolute-encoders/en/AR62_AR63" target="_blank">submersible absolute magnetic encoder</a> with 4-20 mA signaling on a BB variant.  

### RTD
PT100 and/or PT1000 RTDs can be acquired with an RTD conversion breakout board such as the <a href="https://www.adafruit.com/product/3328" target="_blank">MAX31865</a>. This module outputs via SPI protocol and can be read by the BB-Core code with minor modifications.   

<a href ="https://github.com/adafruit/Adafruit_MAX31865" target="_blank">MAX31865</a> library and example

## Communications
### RS232/485
### LoRa
### WiFi
### Web Server

[^1]: "Out of the box" meaning FreeRTOS has been ported to the underlying software development API and can be called from code written in <a href="https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/" target="_blank">Arduino IDE</a> or <a href="https://randomnerdtutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/" target="_blank">Visual Studio Code</a> once the ESP32 board support has been installed. Fortunately, all 'getting started with the ESP32' tutorials will install this support by default. 