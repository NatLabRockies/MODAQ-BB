# MODAQ BlackBox Documentation <img align="right" src="img/MODAQ_BB_Logo.png">

><font color = red>Until this message is removed, this guide is in draft and actively being edited. During this time, some sections may be blank or not fully developed. <font color = #404040>

## General Overview
The National Laboratory of the Rockies (NLR), under funding from the <a href="https://www.energy.gov/eere/water/water-power-technologies-office" target="_blank">Department of Energy's Water Power Technologies Office </a>, has developed a compact, rapid deployment data acquisition system that can withstand water depths up to 400m. Codenamed "BlackBox" (or simply BB), since its initial purpose was to track marine assets and record vital data streams that could later be recovered in the event of a mishap - much like a traditional black box <img style="float: right; padding: 10px 3px 10px 10px;" src="img/bb_complete.png">used in aviation and shipping, the name has stuck. 

MODAQ BlackBox is a battery-operable microcontroller platform with internal inertial sensing, GPS, satellite communications, and additional I/O (input/output) support in a depth-rated pressure enclosure.  

Since BB is self-contained and relatively compact, it can be quickly deployed with minimal effort. The enclosure can be clamped to a tube (such as part of a railing) or mast in a location with unobstructed view of the sky using the available clamp accessory or a common hose clamp. 

While BB is designed to operate unattended, users can configure what data are uploaded and the frequency of satellite transmissions. Once data are uploaded, they can be relayed to an email distribution list, <a href="https://modaq.nrel.gov/" target="_blank">MODAQ:Web</a> operational dashboard, or custom destination. 

BB has found utility in NREL <a href="https://www.nrel.gov/water/marine-energy.html" target = "_blank">Waterpower</a> projects beyond its original vision and has been configured and successfully deployed in more traditional data-gathering applications where a simple, battery-operated solution was indicated. As part of the <a href="https://nrel.github.io/MODAQ/" target="_blank">MODAQ</a> family, BB fills a space in the spectrum of missions can be supported that were previously impractical using the traditional MODAQ hardware architecture due to factors such as weight, size, and cost. 

><font color = blue>Links to 3rd party websites, suppliers, and references in this documentation are provided for convenience and should not to be interpreted as an endorsement by the authors.  <font color = #404040>