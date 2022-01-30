# SurveyorForOrbiterSpaceFlightSimulator

Copyright (C) 2022 Harish Saranathan
Released under the MIT License

This project implements an autopilot for the Surveyor spacecraft used during its lunar descent in Orbiter Space Flight Simulator.

This project requires Orbiter Space Flight Simulator. You can download the product from here:
http://orbit.medphys.ucl.ac.uk/
or here:
https://github.com/orbitersim/orbiter

Install instructions:
1) Download and place Surveyor.DLL under <OrbiterROot>/Modules/
2) 

Follow the steps here to set up the dependencies:
https://www.orbiterwiki.org/wiki/Free_Compiler_Setup

The spacecraft definition is a derivative work of the tutorial here:
https://www.orbiterwiki.org/wiki/Vessel_Tutorial_1
Use the tutorial to obtain and modify the meshes.
The spacecraft definition that is modified from the tutorial can be found in Surveyor.cpp.

The autopilot logic can be found in AutoPilot.cpp.

This project also includes a Visual Studio project file SUrveyor.vcxproj. You will have to modify the project properties so that the 
Include Directories and Library Directories (under VC++ Directories) are based off of your install location of Orbiter SPace Flight Simulator.
Also, modify the Output File location listed in 'Linker -> General' to your requirement.




