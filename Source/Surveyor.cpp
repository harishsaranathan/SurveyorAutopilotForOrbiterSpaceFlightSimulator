// ==============================================================
//                 ORBITER MODULE: Surveyor
//             Copyright (C) 2022 Harish Saranathan
//       Released under the Gnu Free Documentation License
//
// Evolved from:
//                 ORBITER MODULE: Surveyor
//                Copyright (C) 2005 Kwan3217
//      Released under the Gnu Free Documentation License
// 
//                 ORBITER MODULE: ShuttlePB
//                  Part of the ORBITER SDK
//          Copyright (C) 2002-2004 Martin Schweiger
//                   All rights reserved
//
// Surveyor.cpp
// Implementation of Surveyor vessel class, with calls to the autopilot
//
// ==============================================================

#define STRICT
#define ORBITER_MODULE

#include "Surveyor.h"
#include <cstdlib>

// ==============================================================
// Shuttle-PB class interface
// ==============================================================

// Propelant resource handles
PROPELLANT_HANDLE ph_vernier, ph_rcs, ph_retro;

Surveyor::Surveyor(OBJHANDLE hVessel, int flightmodel)
	: VESSEL3(hVessel, flightmodel)
{
}

Surveyor::~Surveyor()
{
}

// ==============================================================
// Overloaded callback functions
// ==============================================================

// --------------------------------------------------------------
// Set the capabilities of the vessel class
// --------------------------------------------------------------
void Surveyor::clbkSetClassCaps(FILEHANDLE cfg)
{
	// Initialize status
	status = 0;

	// Initialize autopilot
	AutoFlight = AutoPilot();

	// physical vessel parameters
	SetSize(PB_SIZE);
	SetPMI(PB_PMI);
	SetCrossSections(PB_CS);
	SetRotDrag(PB_RD);
	SetTouchdownPoints(_V(0, LEG_RAD, LEG_STA), _V(sqrt(3.0) / 2 * LEG_RAD, -0.5 * LEG_RAD, LEG_STA), _V(-sqrt(3.0) / 2 * LEG_RAD, -0.5 * LEG_RAD, LEG_STA));

	// propellant resources
	ph_vernier = CreatePropellantResource(VERNIER_PROP_MASS);
	ph_rcs = CreatePropellantResource(RCS_PROP_MASS);
	ph_retro = CreatePropellantResource(RETRO_PROP_MASS);

	// Retro engines
	th_retro = CreateThruster(_V(0.0, 0.0, RETRO_STA), _V(0, 0, 1), RETRO_THRUST, ph_retro, RETRO_ISP);
	AddExhaust(th_retro, 2, 0.3);

	// Vernier engines
	th_vernier[0] = CreateThruster(_V(0.0 * VERNIER_RAD, 1.0 * VERNIER_RAD, VERNIER_STA), _V(0, 0, 1), VERNIER_THRUST, ph_vernier, VERNIER_ISP);
	th_vernier[1] = CreateThruster(_V(sqrt(3.0) / 2 * VERNIER_RAD, -0.5 * VERNIER_RAD, VERNIER_STA), _V(0, 0, 1), VERNIER_THRUST, ph_vernier, VERNIER_ISP);
	th_vernier[2] = CreateThruster(_V(-sqrt(3.0) / 2 * VERNIER_RAD, -0.5 * VERNIER_RAD, VERNIER_STA), _V(0, 0, 1), VERNIER_THRUST, ph_vernier, VERNIER_ISP);
	CreateThrusterGroup(th_vernier, 3, THGROUP_MAIN);
	for (int i = 0; i < 3; i++) {
		AddExhaust(th_vernier[i], 1, 0.1);
	}

	// Set surface friction coefficients
	SetSurfaceFrictionCoeff(5, 5);

	//Roll (Leg1) jets
	th_rcs[0] = CreateThruster(_V(-RCS_SPACE, RCS_RAD, RCS_STA), _V(1, 0, 0), RCS_THRUST, ph_rcs, RCS_ISP);
	th_rcs[1] = CreateThruster(_V(RCS_SPACE, RCS_RAD, RCS_STA), _V(-1, 0, 0), RCS_THRUST, ph_rcs, RCS_ISP);

	//Leg2 jets
	th_rcs[2] = CreateThruster(_V(sqrt(3.0) / 2 * RCS_RAD, -0.5 * RCS_RAD, RCS_STA - RCS_SPACE), _V(0, 0, 1), RCS_THRUST, ph_rcs, RCS_ISP);
	th_rcs[3] = CreateThruster(_V(sqrt(3.0) / 2 * RCS_RAD, -0.5 * RCS_RAD, RCS_STA + RCS_SPACE), _V(0, 0, -1), RCS_THRUST, ph_rcs, RCS_ISP);

	//Leg3 jets
	th_rcs[4] = CreateThruster(_V(-sqrt(3.0) / 2 * RCS_RAD, -0.5 * RCS_RAD, RCS_STA - RCS_SPACE), _V(0, 0, 1), RCS_THRUST, ph_rcs, RCS_ISP);
	th_rcs[5] = CreateThruster(_V(-sqrt(3.0) / 2 * RCS_RAD, -0.5 * RCS_RAD, RCS_STA + RCS_SPACE), _V(0, 0, -1), RCS_THRUST, ph_rcs, RCS_ISP);

	// Thruster group for attitude pitch down
	th_group[0] = th_rcs[3];
	th_group[1] = th_rcs[5];
	CreateThrusterGroup(th_group, 2, THGROUP_ATT_PITCHDOWN);

	// Thruster group for attitude pitch up
	th_group[0] = th_rcs[2];
	th_group[1] = th_rcs[4];
	CreateThrusterGroup(th_group, 2, THGROUP_ATT_PITCHUP);

	// Thruster group for attitude roll right
	th_group[0] = th_rcs[0];
	CreateThrusterGroup(th_group, 1, THGROUP_ATT_BANKRIGHT);

	// Thruster group for attitude roll left
	th_group[0] = th_rcs[1];
	CreateThrusterGroup(th_group, 1, THGROUP_ATT_BANKLEFT);

	// Thruster group for attitude yaw right
	th_group[0] = th_rcs[3];
	th_group[1] = th_rcs[4];
	CreateThrusterGroup(th_group, 2, THGROUP_ATT_YAWRIGHT);

	// Thruster group for attitude yaw left
	th_group[0] = th_rcs[2];
	th_group[1] = th_rcs[5];
	CreateThrusterGroup(th_group, 2, THGROUP_ATT_YAWLEFT);

	// Add exhaust parameters for each RCS thruster
	for (int i = 0; i < 6; i++) {
		AddExhaust(th_rcs[i], 0.1, 0.05);
	}

	// camera parameters
	SetCameraOffset(_V(0, 0.8, 0));

	// associate a mesh for the visual
	SetupMeshes();
}

void Surveyor::clbkPreStep(double SimT, double SimDT, double MJD) {

	// Get commanded roll, pitch, and yaw
	double P, Y, R;
	P = GetThrusterGroupLevel(THGROUP_ATT_PITCHUP) - GetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN);
	Y = GetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT) - GetThrusterGroupLevel(THGROUP_ATT_YAWLEFT);
	R = GetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT) - GetThrusterGroupLevel(THGROUP_ATT_BANKLEFT);

	// Print the commanded roll, pitch, and yaw for debugging purposes
	//sprintf(oapiDebugString(), "Pitch %f Yaw %f Roll %f", P, Y, R);

	// Define the thrust vector based on the commanded roll, pitch, and yaw
	SetThrusterDir(th_vernier[0], _V(0.087 * R, 0, 1));
	SetThrusterDir(th_vernier[1], _V(0, 0, 1.0 + 0.05 * (P - Y)));
	SetThrusterDir(th_vernier[2], _V(0, 0, 1.0 + 0.05 * (P + Y)));

	// Set empty mass
	SetEmptyMass(CalcEmptyMass());

	if (status == 1 && GetPropellantMass(ph_retro) < 0.0001) {
		//Jettison the spent main retro
		Jettison();
	}
	if (status == 0 && GetPropellantMass(ph_retro) < 0.999 * RETRO_PROP_MASS) {
		//Jettison the AMR if the retro has started burning
		Jettison();
		//Relight the retro if needed
		SetThrusterLevel(th_retro, 1);
	}

	// Call autopilot update loop
	AutoFlight.autopilotUpdate(this, SimT, SimDT);
}

double Surveyor::CalcEmptyMass() {
	// Calculate vessel empty mass

	double EmptyMass = 0;
	if (GetPropellantMass(ph_retro) > 0.999 * RETRO_PROP_MASS) {
		// If the retro tnruster has not been fired yet, the AMR is still attached - add it to the empty mass
		EmptyMass += AMR_MASS;
	}
	if (GetPropellantMass(ph_retro) > 0.0001) {
		// If the retro thruster has propellant left, it means the retro thruster is still attached - add it to the empty mass
		EmptyMass += RETRO_EMPTY_MASS;
	}

	// Add lander empty mass to the total empty mass
	EmptyMass += LANDER_EMPTY_MASS;

	// Return the calculated empty mass
	return EmptyMass;
}

int Surveyor::clbkConsumeBufferedKey(DWORD key, bool down, char* kstate) {
	// The retro thruster can be activated by pressing down the L key

	if (!down) return 0; // only process keydown events

	if (KEYMOD_SHIFT(kstate)) {

	}
	else { // unmodified keys
		switch (key) {
		case OAPI_KEY_L:  // Fire Retro
			SetThrusterLevel(th_retro, 1);
			return 1;
		}
	}
	return 0;
}

void Surveyor::SpawnObject(char* classname, char* ext, VECTOR3 ofs) {
	// Create a new vessel to represent the jettisoned part

	VESSELSTATUS vs;
	char name[256];
	GetStatus(vs);
	Local2Rel(ofs, vs.rpos);
	vs.eng_main = 0;
	vs.eng_hovr = 0.0;
	vs.status = 0;
	strcpy(name, GetName());
	strcat(name, ext);
	oapiCreateVessel(name, classname, vs);
}

void Surveyor::Jettison() {
	// Jettison logic
	// status = 0 - Retro thruster and AMR are attached
	// status = 1 - AMR is jettisoned
	// status = 2 - Retro thruster is jettisoned

	switch (status) {
	case 0:
		// Jettison AMR
		status = 1;
		SpawnObject("Surveyor_AMR", "-AMR", _V(0, 0, -0.6));
		SetupMeshes();
		break;
	case 1:
		// Jettison retro thruster
		status = 2;
		SpawnObject("Surveyor_Retro", "-Retro", _V(0, 0, -0.5));
		SetupMeshes();
		break;
	}
}

void Surveyor::AddLanderMesh() {
	// Lander mesh

	VECTOR3 ofs = _V(0, 0.3, 0);
	AddMesh("Surveyor-Lander", &ofs);
}
void Surveyor::AddRetroMesh() {
	// Retro thruster mesh

	VECTOR3 ofs = _V(0, 0, -0.5);
	AddMesh("Surveyor-Retro", &ofs);
}
void Surveyor::AddAMRMesh() {
	// AMR mesh

	VECTOR3 ofs = _V(0, 0, -0.6);
	AddMesh("Surveyor-AMR", &ofs);
}

void Surveyor::SetupMeshes() {
	// Set up the meshes for the spacecraft stack

	ClearMeshes();
	switch (status) {
	case 0:
		AddAMRMesh();
	case 1:
		AddRetroMesh();
	case 2:
		AddLanderMesh();
	}
}

// --------------------------------------------------------------
// Vessel initialisation
// --------------------------------------------------------------
DLLCLBK VESSEL* ovcInit(OBJHANDLE hvessel, int flightmodel)
{
	return new Surveyor(hvessel, flightmodel);
}

// --------------------------------------------------------------
// Vessel cleanup
// --------------------------------------------------------------
DLLCLBK void ovcExit(VESSEL* vessel)
{
	if (vessel) delete (Surveyor*)vessel;
}
