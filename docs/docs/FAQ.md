# FAQ

## Why build an asset tracker when there are commercially-available devices specifically for this purpose?

When the notion of having a tracker was first presented, our first thought was that we would source something 'off the shelf'. We conducted a search and considered a variety of options targeted to both the private and commercial marine market such as purposeful asset trackers, <a href="https://en.wikipedia.org/wiki/Voyage_data_recorder" target="_blank">voyage data recorders</a>, and even repurposing lagrangian-drifter beacons. Aside from simple location tracking, the requirements expanded to include acquisition of external data streams, auxiliary data storage (backup/mirror of primary DAQ), and flexible communications options. There were also a number of other requirements such as positive buoyancy, can operate on batteries for a specific period of time, survive at depths of at least a few 100m, strobe light, and acoustic pinger. These requirements were prioritized into 'must-haves' and 'added-as-needed' (meaning we needed to have a solution if a specific project required it). 

The commercial options reviewed all had strengths and weaknesses to differing degrees on most of the requirements- which included things like:

- Proprietary or closed systems
- Ecosystem lock-in or subscription charges
- Limited external interfacing (usually limited to common marine or aviation electronics)
- Limited programming or configuration options
- Too bulky or difficult form-factor
- Lack (sufficient) onboard storage
- Limited documentation or unknown specifications of key features
- Some were rugged and submersible, others not so much

It became clear that adapting the most promising candidates would be time consuming and the end result would likely be something that still falls short of requirements- a bit of a kludge. This would be difficult to assure to our sponsors that we're presenting the best solution for their ask and that it will work to expectations. This brought us to the conclusion to design our own system. 