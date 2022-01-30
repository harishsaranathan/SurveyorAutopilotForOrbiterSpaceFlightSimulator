// ==============================================================
//                 ORBITER MODULE: Surveyor
//             Copyright (C) 2022 Harish Saranathan
//                Released under the MIT License
//
// SurveyorConstants.h
// Header file defining constants for Surveyor
//
// ==============================================================

#include "orbitersdk.h"

const double  PB_SIZE = 1.0;                // mean radius [m]
const VECTOR3 PB_CS = { 10.5,15.0,5.8 };    // x,y,z cross sections [m^2]
const VECTOR3 PB_PMI = { 0.50,0.50,0.50 };  // principal moments of inertia (mass-normalised) [m^2]
const VECTOR3 PB_RD = { 0.025,0.025,0.02 }; // rotation drag coefficients
const double  PB_ISP = 5e4;                 // fuel-specific impulse [m/s]

// Vessel mass
const double LANDER_EMPTY_MASS = 289.10; // Basic bus plus payload minus AMR minus retro case
const double RETRO_EMPTY_MASS = 64.88;   // Retro thruster empty mass
const double AMR_MASS = 3.82;            // AMR mass

// Leg position information to define touchdown points
const double LEG_RAD = 1.5;  // Radial distance from roll axis
const double LEG_STA = -0.6; // Position along roll axis

// Retro engine parameters
const double RETRO_PROP_MASS = 560.64;
const double RETRO_THRUST = 39140;
const double RETRO_BURNTIME = 40.5;
const double RETRO_ITOT = RETRO_THRUST * RETRO_BURNTIME;
const double RETRO_ISP = RETRO_ITOT / RETRO_PROP_MASS;
const double RETRO_STA = -0.75;

// RCS thruster parameters
const double RCS_PROP_MASS = 2;
const double RCS_ISP = 630.0;
const double RCS_THRUST = 0.25;
const double RCS_RAD = 1;
const double RCS_STA = -0.5;
const double RCS_SPACE = 0.1;

// Vernier thruster parameters
const double VERNIER_PROP_MASS = 70.98;
const double VERNIER_ISP = 3200;
const double VERNIER_THRUST = 463;
const double VERNIER_RAD = 0.86 - 0.28;
const double VERNIER_STA = -0.5;

// Define impact convex hull
static const DWORD ntdvtx = 12;
static TOUCHDOWNVTX tdvtx[ntdvtx] = {
	{_V(0,  -1.5, 2), 2e4, 1e3, 1.6, 1},
	{_V(-1,  -1.5,-1.5), 2e4, 1e3, 3.0, 1},
	{_V(1,  -1.5,-1.5), 2e4, 1e3, 3.0, 1},
	{_V(-0.5,-0.75,3), 2e4, 1e3, 3.0},
	{_V(0.5,-0.75,3), 2e4, 1e3, 3.0},
	{_V(-2.6,-1.1,-1.9), 2e4, 1e3, 3.0},
	{_V(2.6,-1.1,-1.9), 2e4, 1e3, 3.0},
	{_V(-1,   1.3, 0), 2e4, 1e3, 3.0},
	{_V(1,   1.3, 0), 2e4, 1e3, 3.0},
	{_V(-1,   1.3,-2), 2e4, 1e3, 3.0},
	{_V(1,   1.3,-2), 2e4, 1e3, 3.0},
	{_V(0,   0.3,-3.8), 2e4, 1e3, 3.0}
};

// Surface gravitational acceleration of moon
const double g = 1.62;