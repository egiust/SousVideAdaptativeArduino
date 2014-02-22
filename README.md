SousVideAdaptativeArduino
=========================

Adaptative regulation sous-vide cooker algorithm (for arduino)

For more info, refer to :

http://www.instructables.com/id/Cheap-and-effective-Sous-Vide-cooker-Arduino-power/


Intro
=========================

First, this algorithm is a temperature regulation algorithm not based on PID. The constraints which presided to its design are the following : 

- Must adapt to the characteristics of the system (heating element + heated fluid + container) automatically
- Slow switching time acomodating the use of classical mechanical relays and of all types of heating systems
- Must be adequate for the normal workflow of sous-vide cooking (heat to target temperature / open container / put sous-vide food in it / close container / let it cook)
 

Additional constraints :
- Safety features : automatic shutdown under special conditions
- Automatic detection of sudden drop of temperature attributed to removal of the temperature probe from the fluid : in that case, wait for probe back in the fluid


Adaptative regulation algorithm (overview)
==================================================

After meddling with automatic PID calibration algorithms and getting absolutely no useable results from it, another approach was taken.

The algorithm would not use PID, but rather try to modelize how a human operator would act if he had to perform the regulation.

Here are the main steps of the algorithm :

- Wait for an initial stable temperature
- Heat full steam on till we reach 65% of target temperature, then cut the heat. Due to inertia, temperature will continue to rise, though
- Wait for stabilization (below target temperature)
- Calculate Gain in °C per seconds (of heating) based on this data
- Boost temperature to reach target temperature based on this gain
- Wait for stabilization
- As long as we are not stabilizing within accepted range (0.25°C below or above target temperature), perform other boosts. We will have to do 2 or 3 of these because temperature progression is not linear (while our gain-based dumb human approach assumes it is)
- After that perform regulation (details below)
 

Again, the regulation part tries to modelize how a human operator would do it :

- When temperature is set at target, wait for it to decrease slightly (0.12 °C = the DROP_DEGREES_FOR_CALC_REGULATION constant)
- Based on the time it took, calculate cycling loop of heating and waiting periods and perform it. Ex : heat for 5 seconds, stop and wait for 30 seconds, heat for 5 seconds, stop and wait for 30 seconds, etc ...
- Continue to check temperature periodically
  - if we end up more than 0.2 °C above target temp, wait for the temperature to drop and adjust heating and waiting periods to reduce heat time.
  - if we are under 0.1 °C below target, perform a slight boost.
- If at any time we go under 0.25 °C below target, we leave regulation and perform a classic boost


Not so complicated after all. Also, it works well. 


