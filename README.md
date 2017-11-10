# 3D-printing an Acoustic Spanner


## Credits
* Ermes Toninelli, Mitchell A. Cox, Stuart Brown, Graham Gibson, Matthew Edgar, Andrew Forbes, Miles Padgett 
(Andrew.Forbes@wits.ac.za; Miles.Padgett@glasgow.ac.uk)

## Acknowledgements
* University of Glasgow, Scotland, United Kingdom
* University of the Witwatersrand, Johannesburg, South Africa
* EPSRC
* Centre for doctoral training in Intelligent Sensing and Measurment

## Introduction

Waves are ubiquitous in nature: water waves, light waves, sound waves, etc. Everyone is familiar with the force that waves can impart on objects placed in their path. For example, a laser pointer shining on an object exerts a force on it, albeit a very small one. As it turns out, this force is directly proportional to the power of the wave and inversely proportional to its group velocity: that is for the same power, a slower wave can exert a bigger force than a faster wave. Therefore, in the case of light waves the force is so low, as the speed of light is a very big number. Conversely, for the much slower sound waves (sound is approximately one million times slower than light) it is possible to exert a much bigger force, for an equal power expenditure.

In addition to linear momentum, both light and sound can carry orbital angular momentum (OAM). Whereas linear momentum allows to ‘push’ objects along the direction of propagation, OAM can be used to rotate them! This form of momentum is different from the perhaps more familiar spin angular momentum, which gives rise to the phenomenon of polarisation, and it can be visualised, in the case of light, as each photon spinning on itself. When an OAM carrying field interacts with an object, it exerts a force both along the direction of propagation and around it: like the sort of force that is required to open a door, by both turning the door knob and by pushing the door.

OAM arises from a phase structure within the field. Accordingly, many waves are used to create a certain field and the phase differences between these waves can be engineered by introducing delays. For example, if the waves are in phase with each other (i.e. without delays) the field can be visualised as a series of plane waves, for which the intensity of the waves is the same all over. If however, the phases are regularly changed from 0 to 2pi (one full cycle) in an azimuthal manner (going around the direction of propagation), then the field takes the form of a spiral. This is the type of field that gives rise to OAM.

In this work we use OAM carrying sound waves to build an acoustic spanner. An acoustic spanner allows to exert orbital angular momentum (OAM) on objects, causing them to rotate. Seeing this phenomenon in action is an excellent opportunity to think about the exchange of energy between sound waves and matter, and it is hence an important pedagogical tool. However, building an acoustic spanner able to spin macroscopic objects can require relatively expensive equipment and advanced technical skills. We show how to build a free-space acoustic spanner based on a 3D-printed sound-guiding structure and common electrical components. We show a video of our free-space acoustic spanner in action, in which a styrofoam packing peanut spins in a circular motion and changes direction according to the handedness of the OAM sound (see attached video).

## How do I build one?

1. You will need to get a PCB manufactured. Gerbers are provided which you can send through to your favourite manufacturer. We use Elecrow because they are cheap, fast and do a good job.
2. Accumulate all the compoments. A Bill Of Materials (BOM) is provided as a spreadsheet. The total cost is less than $100.
3. Learn to solder if you don't already know how (YouTube?). Solder all the components, carefully, to the PCB. 
4. Check your soldering and that each component is in the right place. Also check for solder bridges that shouldn't be there.
5. Burn the provided firmware to an Arduino Due using the Arduino IDE or Visual Studio Code.
6. Plug everything together and turn it on! You need to connect a 12V, 1A (or more) power supply to the top PCB for this to work.
