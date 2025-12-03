# MODAQ BlackBox Hardware Architecture

## General

<img align="right" src="../img/bb_mainboard_half.png">

Aside from the functional requirements laid out in the General Overview section, we had several design goals for the BlackBox platform, namely:

1. Low cost hardware
2. Widely available components
3. Popular software development language
4. Open-source libraries for peripherals
5. Strong community support for all of the above

There were also several practical goals to the design:

6. Flexible power modes to minimize power consumption when needed
7. Reasonably performant and reliable components
8. Flexible I/O options
9. Compact size


## Controller
The development of MODAQ BlackBox began at a time when computer chip shortages were reaching its peak and many popular microcontroller (MCU) and microprocessor development platforms were in tight supply or had excessive markups. We looked at a number of MCU options that met our goals that managed to largely avoid the supply and pricing issues, such as Arduino, Teensy, and STM32, but ultimately selected the ESP32 platform. While we could have used any of the other platforms mentioned (and we did test them), the ESP32 has strong adoption in the Maker Community and there are commercial industrial controllers and PLCs based on the ESP32. 

What we like about the ESP32 is its very low cost, tiny form factor, fast 240 MHz dual-core processor, large system memory, <a href="https://www.freertos.org/index.html" target="_blank">FreeRTOS</a> support, ample I/O options, and selectable power and sleep modes. In addition, the ESP32 System on a Chip (SoC) is available by itself for inclusion in custom circuit board designs or as a development board with breadboard/protoboard friendly pinouts, USB controller, and a voltage regulator. There are numerous vendors, including <a href="https://www.adafruit.com/search?q=esp32" target="_blank">Adafruit</a> and <a href="https://www.sparkfun.com/search/results?term=esp32" target="_blank">Sparkfun</a>, that offer custom development board variants, but most follow a similar a similar design to the "<a href="https://www.digikey.com/en/products/detail/espressif-systems/ESP32-DEVKITC-32E/12091810" target="_blank">DevKit C</a>" layout, which is what we used. 

## Modules

At this stage in the BB development, we're using a development board for the MCU and breakout boards for the peripherals. This gives us broad flexibility for the functionality of any particular build. 

><font color = blue><b>Tech Note:</b> Breakout boards are modular circuit board constructions centered around a particular integrated circuit (IC) or SoC module that frequently include additional components as specified by the manufacturer's reference design commonly found in their datasheets. These additional components may include conditioning and/or filtering for signals and power, pull up/down resistors, indicator LEDs, and jumpers. These boards are convenient for breadboarding and rapid prototyping since they 'breakout' the tiny IC or SoC pins to common breadboard/perfboard header pins that are easier to work with. 
<font color = #404040>

We are considering creating a PCB design in EDA software that would use raw components that would theoretically reduce power consumption, main board size, and build times, while improving reliability and ruggedness. However, since the majority of our builds to date have been one-offs, we use a compact perfboard to mount the components and soldered wires for the interconnects ([see Mainboard Layout in the Technical Reference section for details](tech_ref.md#mainboard-layout)). 

While the ESP32 can interface to a wide variety of hardware and communication/signaling protocols, this BB design uses the very common SPI, I2C, UART, and GPIO interfaces. 

### RTC
The ESP32 has an onboard Real Time Clock, however it's not very good and the time is not maintained between power cycles. To address this, we include an external DS3231 RTC with accuracy performance around ±3.5 PPM or better (temperature dependent). This keeps the clock drift to <<1 second/day- which is very good. The DS3231 interfaces to the ESP32 through I2C

><font color = blue><b>Tech Note:</b> PPM is Parts Per Million and is similar to percent. Where percent is 'per' and 'cent' meaning 'per hundred' (100 cents to a dollar, <b>cent</b>ury is 100 years), PPM could be called permill (but never is!). The question is: parts per million of what? When it discussing clocks, it could refer to the drift expected of an oscillator (with its design frequency expressed in MHz for instance) or the drift expected with respect to actual time. Examples can be found <a href="https://www.best-microcontroller-projects.com/ppm.html" target="_blank">here</a>. 
<font color = #404040>

### IMU
The Inertial Measurement Unit selected for BB is the InvenSense MPU-9250, which is a 9-DOF sensor measuring 3 axis each of acceleration, rotation, and magnetic field. Onboard processing allows it to output useful parameters including orientation and heading. InvenSense makes a family of roughly similar and compatible IMUs, including the very common MPU-6050. However, the MPU-9250 includes the magnetometer and supports Wake-On-Motion (WOM)- both of which are utilized in this BB design. If these features are not needed, the MPU-6050 is pin compatible with the BB design (though it has fewer pins, the first 8 pins are the same), but the code will need to be modified, since it's using a 9250 library and expecting the chip to respond with the 9250's identifier byte (and the WOM function calls disabled). The MPU-9250 interfaces to the ESP32 using I2C. 

IMUs can be very fussy devices to work with, since they have inherent drift and can employ complex mathematics and sensor fusion to reduce the impacts of this drift. There are also numerous specifications that are impacted by sensing methods and design price-point. Precision, high-specification models can cost tens of thousands of dollars, so expectations should be tempered for a <$20 device like the MPU-9250. That said, value-wise (defined as performance divided by cost) it's hard to beat- especially for this type of application. 

### GPS
In this BB reference design, we used a GPS breakout board based on the <a href="https://www.u-blox.com/en/product/neo-6-series" target="_blank">u-blox NEO-6M</a> module. This is an older module that's compact, cheap, performs reasonable, and widely available though it's limited to just the US GPS constellation (no Galileo, Glonass, or BeiDou). 

Most <s>GPS</s> GNSS[^1] modules with a UART (serial) communications interface will work with this BB code, since the code parses the text-based <a href="https://aprs.gids.nl/nmea/" target="_blank">NMEA sentences</a> which are standardized on most receivers. Therefore, it's not critical to use the identical module as discussed here. 

><font color = blue><b>Tech Note:</b> GNSS modules capable of receiving constellations other than US GPS may output NMEA sentences with the '$GN' prefix instead of '$GP'. Therefore the popular $GPGGA sentence will be $GNGGA. It may be necessary to adjust the BB code or include library to parse the $GN tags. 
<font color = #404040>

### Iridium Modem
The BB could be built to communicate to the outside world through numerous methods including WiFi, ethernet, LoRa, VHF/UHF radio, cell modem, and serial, however a design requirement was that the BB must report data at least hourly in locations outside of WiFI or cellular range. In addition, since the objectives for the BB are to be self-contained and rapidly deployable, a tether/cable solution or nearby companion base station were out of consideration. This left as the only option satellite communications. 

For short, periodic data messages, our best and most accessible option was <a href="https://www.iridium.com/services/iridium-sbd/" target="_blank">Iridium SBD</a>. To use this service, we selected the low-cost <a href="https://www.sparkfun.com/products/14498" target="_blank">RockBlock 9603N</a> communications module. This has a sufficiently small footprint and an acceptable power use profile. In addition, it easily connects to the ESP32 and has a C++ library. The 9603N interfaces to the ESP32 using UART. 

><font color = blue><b>Tech Note:</b>  Users whose purpose is "related in some way or other to environmental protection, awareness or study, or to protect human life"[^2] might investigate if they qualify for using <a href="https://www.argos-system.org/" target="_blank">Argos</a> instead of Iridium. While BB is not written with Argos in mind, it should not take much effort to modify the C++ code to use Argos. Another option is <a href="https://www.kineis.com/en/homepage-english/" target="_blank">Kinéis</a> which appears to be related to Argos, but allows for less restrictive use cases. 

<font color = #404040>

### Storage
One of the weak points of most MCU-based SBCs, relatively speaking, is that they have few options for local storage. The most common and best supported method is using microSD cards- which is what BB uses. Fortunately,  microSD cards are pretty reliable and widely used in a broad range of electronics, however, not all microSD cards are <a href="https://www.makeuseof.com/tag/5-mistakes-avoid-buying-next-microsd-card/" target="_blank">created equal</a>. We use microSD cards with minimum Speed Class 10 and Application Class A1 or industrial markings from prominent manufacturers. While it's always recommended to perform a data budget analysis[^3] to determine storage size requirements and select a card with sufficient storage. It's best not to go crazy and just get a huge card, since high capacity cards might not be supported due to hardware and/or file system limitations. Cards up to 8GB capacities are sufficient due to the low volume of data generated by the BB tracking variant and will hardly fill a small fraction of that space. 

><font color = blue><b>Tech Note:</b>  MicroSD cards designed for industrial applications claim to be more rugged for for harsh environments and have greater rated write endurance that standard, everyday cards. 

<font color = #404040>

## Power
BB requires both 5v and 3.3v DC supplies to power the controller and breakout boards, while the whole system is powered from 3 primary Lithium Thionyl Chloride batteries wired in series producing 10.8 v nominally. We use step-down voltage regulators to reduce the battery voltage to the desired values. While there is an efficiency penalty to pay with about 10% lost to the conversion, we gain stability from the onboard regulator circuit.    

## Enclosure
We had a few requirements to meet that largely limited our enclosure selection for the electronics, such as: it had to be rugged, survive submersion to several hundred meters, and float. Further, it had to be as compact as possible, meet runtime expectations, contain GPS and Iridium antennas, and be adaptable for connections to external I/O. Of all of these, the depth rating requirement was the dominant factor and that influenced how we met the other requirements. 

BB uses a 3" diameter, 9.5" long cylindrical pressure housing, which was the smallest size we could optimize for that allowed reserve buoyancy (floats) and has a sufficiently sized aperture to mount the antennas[^4] behind a radio-transparent endcap. We were fortunate to find a <a href="https://bluerobotics.com/product-category/watertight-enclosures/" target="_blank">supplier</a> of good quality and low cost pressure housings with a large assortment of sizes and options- and with excellent documentation, complete with CAD files. The specifics of the pressure housing can be found in the [Technical Reference section](tech_ref.md#pressure-housing).


[^1]: Pedantically, GPS refers to the US GPS constellation, while GNSS broadly covers all constellations 
[^2]: https://www.clsamerica.com/science-with-argos
[^3]: In a data budget analysis, multiply together: number bytes written per write operation, number of write operations per day (or hour/minute depending on deployment time scale), number of deployment days, and safety factor (1.10 at minimum). The product will be the minimum size of the storage needed.
[^4]: "sufficiently sized aperture" is a bit of a stretch. As discussed in the [Technical Reference section](tech_ref.md#pressure-housing), we made significant compromises for the antennas, particularly the GPS antenna, to make things fit. 