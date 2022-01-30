# SurveyorAutopilotForOrbiterSpaceFlightSimulator

Copyright (C) 2022 Harish Saranathan
Released under the MIT License

This video demonstrates the project in action: https://youtu.be/5ZG41mQxzlI

This project implements an autopilot for the Surveyor spacecraft used during its lunar descent in Orbiter Space Flight Simulator.

This project requires Orbiter Space Flight Simulator. You can download the product from here:
http://orbit.medphys.ucl.ac.uk/
or here:
https://github.com/orbitersim/orbiter


# INSTALL INSTRUCTIONS

1) Download and place Surveyor.DLL under <OrbiterRoot>/Modules/
2) Download and place SurveyorLanding.scn under <OrbiterRoot>/Scenarios/Surveyor/
3) Download the mesh from here: https://www.orbithangar.com/showAddon.php?id=e69853be-2dd6-4b37-a5df-fe6827c01cae
  Follow the tutorial here to modify and place the meshes appropriately:
  https://www.orbiterwiki.org/wiki/Vessel_Tutorial_1#Meshes
  https://www.orbiterwiki.org/wiki/Vessel_Tutorial_2#Jettisoning_Stuff_.28Visual.29
  You only need Surveyor-AMR.msh, Surveyor-Lander.msh, and Surveyor-Retro.msh placed in <OrbiterRoot>/Meshes/. Use Shipedit.exe
  under <OrbiterRoot>/Orbitersdk/utils/ as directed by the tutorial links above, but do not modify C++ code.
  I have not included the meshes in this project because I am not the author of them and do not hold the copyright.
4) Launch Orbiter. Navigate to Surveyor directory and launch the scenario. Enjoy!


# SCENARIO OVERVIEW
  
The scenario represents the descent of the Surveyor spacecraft onto the lunar surface.
The Surveyor program consisted of a series of unmanned lander missions before the manned Apollo missions. It was launched directly to
the moon on a collision trajectory, with one mid-course correction to impact at the desired landing location. About 30 minutes before
impact, the spacecraft was re-oriented to point retrograde (opposite to the direction of travel). The Altitude Marker Radar (AMR)
located just inside the retro rocket engine was precisely measuring the height above terrain. Upon crossing an altitude of 110 km, 
the spacecraft was traveling at about 2.5 km/s. At this rate, it would impact in less than 45 seconds! After crossing this altitude
mark, a timer was started that counted to 7 seconds, after which the retro rocket engine was fired until it depleted its propellant.
When the engine ignited, the exhaust spit out the AMR, which eventually fell to the surface. While the retro engine was firing, the
three vernier thrusters were used to keep the spacecraft oriented retrograde. After the retro engine depleted its propellant, it was
also jettisoned. The spacecraft was at about 25 km altitude, and still falling at about 200 m/s. The vernier thrusters continued firing
to eat away the remaining velocity. These thrusters were throttled by a controller using altitude readings from a four-beam doppler radar.
The goal was to slow down to about 1.3 m/s by the time it was just 4 m above the surface. At this point, the vernier thrusters were shut
down, and the spacecraft was in free fall for these remaining 4 meters, and eventually settled on the surface.
Note that the spacecraft also consisted of six cold gas reaction control system (RCS) thrusters to change the spacecraft orientation.
However, they were very weak, and provided angular velocity magnitude of only about 0.5 deg/s after firing for several seconds.

The scenario starts with Surveyor at about 1000 km altitude, falling towards the lunar surface at about 2.1 km/s. It is yet to orient
itself retrograde (unlike the original mission).

My autopilot stays idle for the first 10 seconds, and then uses the vernier thrusters (instead of the RCS thrusters for better control
authority) to orient the spacecraft retrograde. The rest of the autopilot logic follows the description above. There is absolutely no
manual intervention at any point in the scenario.
  

# IF YOU WANT TO MODIFY SOURCE CODE

Follow the steps here to set up the dependencies:
https://www.orbiterwiki.org/wiki/Free_Compiler_Setup

The spacecraft definition is a derivative work of the tutorial here:
https://www.orbiterwiki.org/wiki/Vessel_Tutorial_1
The spacecraft definition that is modified from the tutorial can be found in Surveyor.cpp.

The autopilot logic can be found in AutoPilot.cpp.

This project also includes a Visual Studio project file SUrveyor.vcxproj. You will have to modify the project properties so that the 
Include Directories and Library Directories (under VC++ Directories) are based off of your install location of Orbiter SPace Flight Simulator.
Also, modify the Output File location listed in 'Linker -> General' to your requirement.




