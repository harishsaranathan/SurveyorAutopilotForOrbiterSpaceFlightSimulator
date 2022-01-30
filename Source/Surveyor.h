// ==============================================================
//                   ORBITER MODULE: Surveyor
//             Copyright (C) 2022 Harish Saranathan
//                Released under the MIT License
//
// Surveyor.h
// Header file with declarations for Surveyor and AutoPilot classes
//
// ==============================================================

#include "SurveyorConstants.h"

class Surveyor;

// Autopilot modes
enum AutoPilotStatus { IDLE, HOLD_FOR_RETRO, RETRO_DESCENT, FINAL_DESCENT, SHUTDOWN };

// Autopilot class declaration
class AutoPilot {
public:
	AutoPilot(void);
	void vernierControl(Surveyor* sc, double const & thrustControl);
	void angularVelocityController(Surveyor* sc, VECTOR3 const omega_d, VECTOR3 const omega, double const & thrustLevel);
	void updateTimer(double const dt);
	void autopilotUpdate(Surveyor* sc, double const& SimT, double const & dt);
	void idleControl(Surveyor* sc, double const & dt);
	void holdForRetroDescent(Surveyor* sc);
	void retroDescent(Surveyor* sc, double const & dt);
	void finalDescent(Surveyor* sc);
	void shutdown(Surveyor* sc);
	double radarAltitude(Surveyor* sc);
	void idleVernierThrusters(Surveyor* sc);
private:
	VECTOR3 VernierThrustLevel; // Throttle level for vernier engines
	VECTOR3 Kp_w; // Proportional gain for angular velocity loop
	double Kp_ang; // Proportional gain for angle error loop
	double Alpha; // Thrust vector angle for vernier thruster 1 for roll control
	AutoPilotStatus Mode; // Autopilot mode
	double Timer; // Timer used in switching autopilot modes
};

// Surveyor class declaration
class Surveyor : public VESSEL3 {
public:
	Surveyor(OBJHANDLE hVessel, int flightmodel);
	~Surveyor();
	void clbkSetClassCaps(FILEHANDLE cfg);
	void clbkPreStep(double SimT, double SimDT, double MJD);
	double CalcEmptyMass();
	int clbkConsumeBufferedKey(DWORD key, bool down, char* kstate);
	void SpawnObject(char* classname, char* ext, VECTOR3 ofs);
	void Jettison();
	void SetupMeshes();
	void AddLanderMesh();
	void AddRetroMesh();
	void AddAMRMesh();

	THRUSTER_HANDLE th_vernier[3], th_retro, th_rcs[6], th_group[2];
private:
	AutoPilot AutoFlight; // Autopilot
	int status; // Vessel status to represent staging
};
