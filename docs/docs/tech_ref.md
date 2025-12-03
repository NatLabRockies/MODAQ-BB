
#Technical Reference Manual

## System Design

Once all the components were selected, it was necessary to figure out how to mount and wire them- while being constrained by the available space in the enclosure. It was an iterative process with several variables to optimize, including the enclosure's diameter and length, battery size and quantity, and electronics placement. Since one of our requirements was that the package floats, volume, weight, and center of gravity were also selection factors. Ultimately, it was determined that a 75mm inner diameter by 240mm length cylindrical pressure housing would be the minimal size for this design. 

### Schematic

![MODAQ BlackBox Schematic](img/BBDAQ_Schematic_drawio.png)

### Mainboard Layout

With the 75mm diameter constraint of the pressure housing, we limited the board width to a maximum of 60mm to allow for component clearance with the wall curvature.  With a bit of finesse, we were able to arrange all components except the satellite modem on a single side of a 58 x 94mm <a href ="https://www.amazon.com/Treedix-Solderable-BreadBoard-Universal-Prototyping/dp/B0896YPD8F" target="_blank">perfboard</a>. The other side of the board was reserved primarily for wiring. 


![MODAQ BlackBox Board Layout](img/bb_board_layout.png)

Part placement was dense, but manageable. All components stayed within the footprint of the perfboard. The wiring was pre-mapped using the row and column markings on the board in a spreadsheet and this was used as a checklist during soldering so that no connections were missed. Even with careful planning, the wiring turned into a bit of a rat's nest, with congested zones that became difficult to solder. Our technique improved for the second unit we assembled (not pictured), but we are strongly considering designing a mainboard in <a href="https://www.kicad.org/" target="_blank">KiCad</a> (or other EDA software) if we intend to produce more of these. 

![MODAQ Blackbox Board Wiring](img/bb_board_back.jpeg)

### Power System
This implementation of MODAQ BlackBox is powered by 3 Tadiran Lithium Thionyl Chloride D-sized primary batteries wired in parallel to deliver, nominally, 3.6v with 57 Ahr or around 200 Whr of capacity. <img align="right" src="../img/bb_battery_bay.png">

><font color = red>
<AB: insert power budget discussion>
<font color = #404040>

The mainboard components operate on 3.3v while the satellite modem requires 5v. We used a <a href="https://www.pololu.com/product/2842" target="_blank">buck converter</a> to regulate the 3.3v supply and a <a href="https://www.pololu.com/product/2564" target="_blank">boost converter</a> for the 5v side. 

## Power Management
Power management was a top priority and it informed several decisions including component selection and sleep strategies. Unlike DAQ variants of this design, the BlackBox configuration does not require continuous sampling. Therefore we could introduce sleep cycles to conserve power. 

### MCU

The ESP32 MCU can be programmatically throttled to lower clock speeds to reduce power consumption. For instance this line will reduce the clock speed to 80 Mhz:
```
{
    setCpuFrequencyMhz(80);
}
```
This will result in approximately 50% power savings over the full (default) 240 MHz clock speed. This <a href="https://mischianti.org/2021/03/06/esp32-practical-power-saving-manage-wifi-and-cpu-1/" target="_blank">link</a> contains a good discussion and how-to on ESP power saving measures. 

### GPS Module

The GPS' maximum power consumption is on par with the MCU operating at full clock speed- and this power varies depending on the current operational mode the GPS. Simply, the GPS will startup in Acquisition mode until it acquires sufficient satellites and then it may toggle between Acquisition mode and Tracking mode as needed. Tracking mode consumes less power than Acquisition mode, however to be conservative, we based our power consumption projections on the device's maximum published drain. Some GPS modules may have power saving modes that can be preset or invoked, however do to the variability of these modes and likely performance reduction, we left the GPS to its default settings and elected to control the power going to the GPS module instead. 

Using a GPIO pin on the ESP32 MCU, we set the pin to digital HIGH when we want to power the GPS and LOW to turn it off. Since a GPIO pin cannot supply sufficient current to operate the GPS, we use a 2N222 transistor connected to that GPIO pin to switch the supply to the GPS. 

There are concerns that simply pulling the plug on the GPS is not an ideal way to manage the power consumption of the system. Continuing the discussion above on Acquisition vs Tracking mode, we usually think of GPS operation in terms of Cold Start, Warm Start, and Hot Start. Manufacturers will often publish expected times to first fix based on which of these start modes is currently applicable. The distinction between these modes is a function of completeness and freshness of the device's ephemeris data (record of tracked satellites' orbital position and information necessary to calculate a positional fix), which in itself is a function of how long the device spent in Acquisition mode and how long ago Acquisition mode ran. 

Cutting power to the GPS obviously drops the power consumption to zero, but also suspends the building of the ephemeris data. Most GPS units have a backup battery which will preserve the ephemeris between power cycles, but if enough time has elapsed - or the ephemeris was incomplete on the last power up interval - the GPS may have to spend more time at full power consumption to achieve its first fix. 

We've found that after a 30-60 minute sleep cycle, our time to first fix (TTFF), under best conditions, is <60 seconds. However, if there is heavy cloud cover or antenna obstruction, this time could easily exceed several minutes. In the BlackBox code, we have several measures to address long TTFF as well as low satellite count and fix age:

1. Reject fixes that are more than 1000 ms old.
2. If TTFF is greater than a configurable value, we cancel GPS sleep for the next sleep cycle.
3. If satellite count is less than a configurable value during any one of the first 5 wake cycles, we cancel GPS sleep on next sleep cycle.

These measures aim to prevent data starvation of the GPS and improve fix reliability. It may be worthwhile to explore alternative strategies rather than simply killing power to the GPS, however this approach has worked for us.


## Pressure housing

The MODAQ BB design presented here uses the BlueRobotics <a href="https://bluerobotics.com/store/watertight-enclosures/wte-vp/#tube" target="_blank">aluminum subsea housing</a> with an inner diameter of 75mm (3") and 240mm (9.5") in length. For radio transparency, we selected the acrylic blank endcap for one end of the housing. Acrylic parts affect the maximum depth rating of the system, however it was necessary for the GPS and satellite radios to function. 

Maximum possible depth rating for BlueRobotics housings is 950m. The following selections will impact the depth rating:

- Flat acrylic endcap derate depth to 400m
- Dome acrylic endcap derate depth to 750m
- Aluminum endcap full depth rating 950m
- Swap aluminum tube for transparent acrylic, derates from 125m to 300m depending on length of tube

For completely submerged applications, it's not necessary to use any acrylic parts unless transparency is necessary. 