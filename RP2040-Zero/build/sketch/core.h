#line 1 "/Users/klausmichalsky/Proyectos Mac/KAYRON/RP2040-Zero/core.h"
// =======================================================================
//                          🔹 K A Y R O N 🔹
// =======================================================================
//  Archivo    : core.h
//  Autor      : Klaus Michalsky
//  Fecha      : Mar-2026
// -----------------------------------------------------------------------
//  ▫️ DESCRIPCIÓN
//      - Declaración de funciones centrales del robot.
// =======================================================================

#pragma once

#include <Arduino.h>

#include "config.h"
#include "homing.h"

void coreInit();
void coreUpdate();
