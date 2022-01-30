// ==============================================================
//                  ORBITER MODULE: Surveyor
//             Copyright (C) 2022 Harish Saranathan
//                Released under the MIT License
//
// AutoPilot.cpp
// Autopilot implementation for Surveyor
//
// ==============================================================

#include "Surveyor.h"
#include <cstdlib>
#include <algorithm>

AutoPilot::AutoPilot(void)
// Autopilot constructor
{
	// Initialize proportional gains for angular velocity controller (inner loop of attitude control system)
	Kp_w.x = 400;
	Kp_w.y = 400;
	Kp_w.z = 400;

	// Initialize gains for angle error controller (outer loop of attitude control system)
	Kp_ang = 0.5;

	// Initialize autopilot mode
	Mode = IDLE;

	// Initialize timer
	Timer = 0;
}

void AutoPilot::idleVernierThrusters(Surveyor* sc)
// Set the vernier thrust levels to 0, and set vernier thruster 1 thrust vector angle to 0
{
	// Set all vernier thrust levels to 0
	sc->SetThrusterLevel(sc->th_vernier[0], 0);
	sc->SetThrusterLevel(sc->th_vernier[1], 0);
	sc->SetThrusterLevel(sc->th_vernier[2], 0);

	// Set thrust vector angle of vernier thruster 1 to 0
	sc->SetThrusterDir(sc->th_vernier[0], _V(0, 0, 1));
}

double AutoPilot::radarAltitude(Surveyor* sc)
// Radar altimeter
{
	return sc->GetAltitude() - sc->GetSurfaceElevation();
}

void AutoPilot::updateTimer(double const dt)
// Advance timer by the specified time dt in seconds
{
	Timer += dt;
}

void AutoPilot::autopilotUpdate(Surveyor* sc, double const& SimT, double const& dt)
// Autopilot loop called in each orbiter time step
{
	// Declare character vector representing autopilot mode that will be printed in the debug string
	char ModeString[24];

	// Call autopilot routine based on mode, and fill out ModeString appropriately
	switch (Mode)
	{
	case IDLE:
		strncpy(ModeString, "Idle", 24);
		idleControl(sc, dt);
		break;
	case HOLD_FOR_RETRO:
		strncpy(ModeString, "Hold for retro ignition", 24);
		holdForRetroDescent(sc);
		break;
	case RETRO_DESCENT:
		strncpy(ModeString, "Initial descent", 24);
		retroDescent(sc, dt);
		break;
	case FINAL_DESCENT:
		strncpy(ModeString, "Final descent", 24);
		finalDescent(sc);
		break;
	case SHUTDOWN:
		strncpy(ModeString, "Shutdown", 24);
		shutdown(sc);
		break;
	default:
		strncpy(ModeString, "Unknown", 24);
		vernierControl(sc, 0);
	}

	// Print debug string
	VECTOR3 v;
	bool status = sc->GetAirspeedVector(FRAME_LOCAL, v);
	sprintf(oapiDebugString(), "Autopilot mode: %s   Altitude: %f m   Velocity: %f m/s   Vernier thrust levels: %f, %f, %f   Retro thrust level: %f",
		ModeString,
		radarAltitude(sc),
		sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2)),
		sc->GetThrusterLevel(sc->th_vernier[0]), sc->GetThrusterLevel(sc->th_vernier[1]), sc->GetThrusterLevel(sc->th_vernier[2]),
		sc->GetThrusterLevel(sc->th_retro));
}

void AutoPilot::idleControl(Surveyor* sc, double const & dt)
// Autopilot routine for IDLE mode. This is the initial mode, and lasts for 10 seconds. All thrusters are left at idle.
{
	// Once the timer ticks to 10, advance autopilot mode to HOLD_FOR_RETRO, reset timer, and return
	if (Timer >= 10)
	{
		Mode = HOLD_FOR_RETRO;
		Timer = 0;
		return;
	}

	// Idle vernier thrusters
	idleVernierThrusters(sc);

	// Advance timer by the orbiter-specified time step dt
	updateTimer(dt);
}

void AutoPilot::holdForRetroDescent(Surveyor* sc)
/* Autopilot routine for HOLD_FOR_RETRO mode.The vernier thrusters are used to
   orient the spacecraft opposite to surface relative velocity vector.*/
{
	// Height above terrain
	double altitude = radarAltitude(sc);

	// Set vernier thrust levels. The steady state thrust level is 0.
	vernierControl(sc, 0);

	// If altitude goes below 110 km, advance autopilot mode to RETRO_DESCENT
	if (altitude <= 110000) Mode = RETRO_DESCENT;
}

void AutoPilot::retroDescent(Surveyor* sc, double const& dt)
/* Autopilot routine for RETRO_DESCENT mode. The timer starts to run at the beginning of this mode.
   After 7 seconds have elapsed, the retro rocket is ignited, which burns at maximum thrust until
   the propellant is exhausted. All this time, the vernier thrusters are used for keepimg the spacecraft
   oriented opposite to surface relative velocity.*/
{
	// After 7 seconds have elapsed since the beginning of this mode, fire the retro rocket.
	if (Timer >= 7)
	{
		// Keep the retro thrust level at maximum.
		sc->SetThrusterLevel(sc->th_retro, 1);

		// Use the vernier thrusters only to keep the spacecraft oriented opposite to surface relative velocity.
		// THe steady state thrust level is 0.
		vernierControl(sc, 0);
	}

	// The retro rocket propellant will be exhausted after 40 seconds from ignition. Wait for one
	// more second, and then advance the autopilot mode to FINAL_DESCENT. Reset the timer.
	if (Timer >= 48)
	{
		sc->SetThrusterLevel(sc->th_retro, 0);
		Timer = 0;
		Mode = FINAL_DESCENT;
	}

	// Advance the timer by the orbiter-specified time step dt.
	updateTimer(dt);
}

void AutoPilot::finalDescent(Surveyor* sc)
/* Autopilot routine for FINAL_DESCENT mode. Keep the vernier thrusters at idle until descending below 20 km,
   except for keepimg the spacecraft oriented opposite to surface relative velocity. After this, they are used
   for both slowing down and keeping the spacecraft oriented opposite to the surface relative velocity. The
   desired thrust level is updated at each time step as a constant value that will provide the desired velocity
   at 0 m altitude. The approach is naiive, as it assumes the instantaneous mass to be constant at each uptate.
   TODO: Derive analytical expression for this desired thrust by accounting decreasing mass.
   This mode ends when the altitude is 4 meters. */
{
	// Height above terrain
	double altitude = radarAltitude(sc);

	if (altitude <= 4.0)
	// If altitude is less than or equal to 4 m, advance the autopilot mode to SHUTDOWN, and return.
	{
		Mode = SHUTDOWN;
	}
	else if (altitude > 20000)
		// If the altitude is greater than 20 km, set the steady state thrust level of the vernier thrusters to 0, but
		// continue to use them to keep the spacecraft oriented opposite to surface relative velocity vector.
	{
		vernierControl(sc, 0);
	}
	else
		// Calculate the vernier thrust level for a desired final velocity, while simultaneously using it to keep the
		// spacecraft oriented opposite to surface relative velocity vector.
	{
		// Obtain current mass
		double m = sc->GetMass();

		// Obtain current surface relative velocity vector
		VECTOR3 u;
		bool status = sc->GetAirspeedVector(FRAME_LOCAL, u);

		// Current surface relative velocity magnitude squared
		double uSq = pow(u.x, 2) + pow(u.y, 2) + pow(u.z, 2);

		// Set the final desired velocity magnitude to 1 m/s if the altitude is less than 500 ft, and 50 m/s otherwise.
		double vSq;
		if (altitude <= 500)
		{
			vSq = 1.0;
		}
		else
		{
			vSq = 50 * 50;
		}
		// Required thrust assuming constant gravity, mass, and a flight path angle of -90 degrees
		double F = (m * g) - (m * (vSq - uSq) / altitude);

		// Set the vernier thrust levels
		vernierControl(sc, min(max(F / (3 * VERNIER_THRUST), 0), 1));
	}
}

void AutoPilot::shutdown(Surveyor* sc)
// Autopilot routine for SHUTDOWN mode. The vernier thrusters and vernier thruster 1 thrust vector angle are set to 0.
{
	// Idle vernier thrusters
	idleVernierThrusters(sc);
}

void AutoPilot::vernierControl(Surveyor* sc, double const & thrustLevel)
// Controller for vernier thrusters to maintain the specified steady state thrust level, while also keeping the spacecraft
// oriented retrograde with respect to the surface relative velocity vector
{
	// Get the current surface relative velocity vector
	REFFRAME frame = FRAME_LOCAL;
	VECTOR3 v;
	bool status = sc->GetAirspeedVector(frame, v);

	// Get the current angular velocity vector
	VECTOR3 w;
	sc->GetAngularVel(w);

	// Calculate the unit vector in the body frame of the spacecraft about which the spacecraft must be rotated to get to the
	// desired orientation. This calculates as the unit vector of the cross product between the roll axis (z axis) and the 
	// negative surface relative velocity vector
	VECTOR3 lambda;
	lambda.x = -(-v.y / sqrt(pow(v.x, 2) + pow(v.y, 2)));
	lambda.y = (-v.x / sqrt(pow(v.x, 2) + pow(v.y, 2)));
	lambda.z = 0;

	// Calculate the angle by which the spacecraft must be rotated about lambda to get to the desired orientation
	double ang = acos((-v.z) / sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2)));

	// Calculate the desired angular velocity vector for the inner control loop that drives the angular velocity vector to the desired value
	VECTOR3 omega_d;
	if (ang < 0.01)
	// If ang is less than 0.01 radians, and hence, inside the deadband, the desired angular velocity is 0
	{
		omega_d = { 0,0,0 };
	}
	else
	// If ang is outside the deadband, ang is driven towards by a proportional controller
	{
		// Outer loop for driving ang towards 0, which commands the desired angular velocity magnitude, and limit the magnitude
		// to 0.02 rad/s
		double omega_d_mag = -min(Kp_ang * ang, 0.02);

		// Construct the desired angular velocity vector
		omega_d = { lambda.x * omega_d_mag, lambda.y * omega_d_mag, lambda.z * omega_d_mag };
	}

	// Call angular velocity controller to calculate the desired thrust level for each vernier thruster, and the desired thrust vector
	// for vernier thruster 1
	angularVelocityController(sc, omega_d, w, thrustLevel);

	// Set the vernier thrust levels to the value specified by the controller
	sc->SetThrusterLevel(sc->th_vernier[0], VernierThrustLevel.x);
	sc->SetThrusterLevel(sc->th_vernier[1], VernierThrustLevel.y);
	sc->SetThrusterLevel(sc->th_vernier[2], VernierThrustLevel.z);

	// Set the vernier thruster 1 thrust vector angle to the desired angle specified by the controller
	sc->SetThrusterDir(sc->th_vernier[0], _V(sin(Alpha), 0, cos(Alpha)));
}

void AutoPilot::angularVelocityController(Surveyor* sc, VECTOR3 const omega_d, VECTOR3 const omega, double const & thrustLevel)
// Angular velocity control loop to calculate the vernier thrust levels and vernier thruster 1 thrust vector angle to drive the
// spacecraft angular velocity to the desired value, while simultaneously providing the specified steady state thrust level
{
	// Angular velocity vector error
	VECTOR3 OmegaError = { omega.x - omega_d.x , omega.y - omega_d.y , omega.z - omega_d.z };

	if (sqrt(pow(OmegaError.x, 2) + pow(OmegaError.y, 2) + pow(OmegaError.z, 2)) < 0.0001)
	// If the angular velocity vector error magnitude is less than 0.0001 rad/s, do not attempt to modify the angular velocity
	// further as it is inside the angular velocity deadband. Accordingly, just set the thrusters to the specified steady
	// state thrust level, and vernier thruster 1 thrust vector angle to 0.
	{
		VernierThrustLevel.x = thrustLevel;
		VernierThrustLevel.y = thrustLevel;
		VernierThrustLevel.z = thrustLevel;
		Alpha = 0;
	}
	else
	// If the angular velocity error is outside the deadband, calculate the thrust levels and thrust vector angle to drive the
	// error towards 0
	{
		VECTOR3 M; // Desired moments
		double F1; // Desired thruster 1 thrust
		double F2; // Desired thruster 2 thrust
		double F3; // Desired thruster 3 thrust

		// Set the thruster 1 level directly to the specified steady state value, and clip the value to between 0.05 and 0.95.
		// The clipping ensured control saturation will be avoided on other thrusters.
		F1 = VERNIER_THRUST*min(max(thrustLevel, 0.05), 0.95);

		// Calculate the desired moments based on a proportional controller to drive the angular velocity vector to 0.
		M.x = Kp_w.x * OmegaError.x;
		M.y = Kp_w.y * OmegaError.y;
		M.z = Kp_w.z * OmegaError.z;

		// Based on the desired roll moment and vernier thruster 1 thrust level, calculate the thrust vector angle
		Alpha = min(max(asin(-(M.z / (VERNIER_RAD * F1))), -5 * PI / 180), 5 * PI / 180);

		// Based on the desired pitch and yaw moments, vernier 1 thrust level, and vernier 1 thrust vector angle, calculate the desired
		// vernier 2 and 3 thrust levels
		F2 = ((F1 * cos(Alpha)) - (M.x / VERNIER_RAD)) - ((1 / sqrt(3)) * ((VERNIER_STA * F1 * sin(Alpha)) - M.y) / VERNIER_RAD);
		F3 = ((F1 * cos(Alpha)) - (M.x / VERNIER_RAD)) + ((1 / sqrt(3)) * ((VERNIER_STA * F1 * sin(Alpha)) - M.y) / VERNIER_RAD);

		// Clip the thrust levels to between 0 and 1
		VernierThrustLevel.x = min(max(F1 / VERNIER_THRUST, 0), 1);
		VernierThrustLevel.z = min(max(F2 / VERNIER_THRUST, 0), 1);
		VernierThrustLevel.y = min(max(F3 / VERNIER_THRUST, 0), 1);
	}
}