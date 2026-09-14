#line 1 "C:\\Users\\Klaus\\Documents\\ONYX-Pro\\RP2040-Zero\\core.cpp"
// =======================================================================
//                          🔹 K A Y R O N 🔹
// =======================================================================
//  Archivo    : core.cpp
//  Autor      : Klaus Michalsky
//  Fecha      : Mar-2026
// -----------------------------------------------------------------------
//  ▫️ DESCRIPCIÓN
//      - Implementación de funciones centrales de sincronización
// =======================================================================

#include <Arduino.h>

#include "command.h"
#include "core.h"
#include "homing.h"
#include "motors.h"
#include "sensors.h"
#include "utils.h"
#include "xy_plane.h"
#include "z_axis.h"

float currentShoulderAngle = 0.0f;
float currentElbowAngle = 0.0f;

// INICIALIZACIÓN DEL CORE
// -----------------------------------------------------------------------
void coreInit() {
    homeAllState = HomeAllState::IDLE;
    homeSingleState = HomeSingleState::IDLE;
}

// CONTROL DE TAREAS EN EL FLUJO
// -----------------------------------------------------------------------

void coreUpdate() {
    if (homeAllState != HomeAllState::IDLE) {
        homeAll();
        return;
    }

    if (homeSingleState != HomeSingleState::IDLE) {
        homeSingleMotor();
        return;
    }

    updateXY();
    updateZ();

    // 🔥 SOLO UNA STATE MACHINE ACTIVA
    if (captureSeqState != CaptureSequenceState::IDLE) {
        updateCaptureSequence();
    } else if (moveSeqState != MoveSequenceState::IDLE) {
        updateMoveSequence();
    }
    // ...otras tareas
}

void resetAll() {
    resetXYStates();
    resetZStates();
    COMM.println("REINICIO COMPLETO");
}