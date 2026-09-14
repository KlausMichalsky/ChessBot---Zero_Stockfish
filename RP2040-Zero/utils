// =======================================================================
//                         🔹 O N Y X   P R O 🔹
// =======================================================================
//  Archivo    : utils.cpp
//  Autor      : Klaus Michalsky
//  Fecha      : Feb-2026
// -----------------------------------------------------------------------
//  ▫️ DESCRIPCIÓN
//      - Implementación de funciones de cálculo y conversiónes
//      - Ángulos a pasos de motores
//      - Casilla a coordenadas
//      - Casilla a ángulos
//      - Cinemática inversa
// =======================================================================

#include <Arduino.h>
#include <math.h>

#include "communication.h"
#include "config.h"
#include "sensors.h"
#include "utils.h"

// CONVERCION DE ANGULO A PASOS DE MOTOR
// -----------------------------------------------------------------------
long angleToStep(float angle, MotorID id) {
    switch (id) {
        case MotorID::J1:
            return (angle / 360.0) *
                   motor1Config.microstepping *
                   motor1Config.reduction *
                   motor1Config.stepsPerRevolution;
        case MotorID::J2:
            return (angle / 360.0) *
                   motor2Config.microstepping *
                   motor2Config.reduction *
                   motor2Config.stepsPerRevolution;
        default:
            // Por si llega un MotorID inválido
            COMM.println("ERROR: MotorID inválido en angleToStep");
            return 0;
    }
}

// CONVERCION DE VALOR BRUTO DE 12 BITS DEL AS5600 (0-4095) A GRADOS
// -----------------------------------------------------------------------
float rawToDegrees(uint16_t rawAngle) {
    return (rawAngle * 360.0) / 4096.0;
}

// REDONDEO DE FLOAT A 1 DECIMAL
// -----------------------------------------------------------------------
float round1Decimal(float value) {
    return ((int)(value * 10 + 0.5)) / 10.0;
}

// CONVERSIÓN DE CASILLA → XY
// -----------------------------------------------------------------------
bool chessSquareToXY(
    const String &square,
    float &x,
    float &y) {
    // Validar longitud
    if (square.length() != 2) { // Verifica que el texto tenga exactamente 2 caracteres.
        return false;
    }

    // Obtener letra y numero
    char file = toupper(square[0]); // Optener la primera letra y convertir a mayuscula
    char rank = square[1];          // obtener el numero

    // Validar fila y ángulo
    if (file < 'A' || file > 'H') {
        return false;
    }
    if (rank < '1' || rank > '8') {
        return false;
    }

    // Convertir a índices
    int fileIndex = file - 'A'; // 'A' -> 0, 'B' -> 1, ..., 'H' -> 7
    int rankIndex = rank - '1'; // '1' -> 0, '2' -> 1, ..., '8' -> 7

    // Convertir a coordenadas en centro de casillas
    x = A1_OFFSET_X + (fileIndex * SQUARE_SIZE);
    y = A1_OFFSET_Y + (rankIndex * SQUARE_SIZE);

    return true;
}


// -----------------------------------------------------------------------
// CINEMÁTICA INVERSA SCARA
// -----------------------------------------------------------------------
bool inverseKinematics(float x, float y, float l1, float l2, float &theta1Deg, float &theta2Deg) {
  // 1. Distancia desde el origen (0,0) al objetivo
  float r2 = x * x + y * y;
  float r = sqrtf(r2);

  // 2. Validación de alcance del brazo
  if (r > (l1 + l2) || r < fabsf(l1 - l2)) {
    return false; // Punto fuera de alcance
  }

  // 3. Ángulo del Codo (Theta 2)
  // Ley de cosenos para theta2 -> c2=a2+b2−2abcos(C)
  float cos_theta2 = (l1 * l1 + l2 * l2 - r2) / (2.0f * l1 * l2);
  if (cos_theta2 > 1.0f)  cos_theta2 = 1.0f;
  if (cos_theta2 < -1.0f) cos_theta2 = -1.0f;

  float theta2Rad = acosf(cos_theta2); // ~39.67°

  // 4. Ángulo del Hombro (Theta 1)
  float alpha = atan2f(y, x); // ~42.41°

  float cos_psi = (l1 * l1 + r2 - l2 * l2) / (2.0f * l1 * r);
  if (cos_psi > 1.0f)  cos_psi = 1.0f;
  if (cos_psi < -1.0f) cos_psi = -1.0f;

  float psi = acosf(cos_psi); // ~30.87°

  float theta1Rad = alpha - psi; // ~11.54°

  // 5. Conversión a Grados
  theta1Deg = theta1Rad * 180.0f / M_PI;
  theta2Deg = theta2Rad * 180.0f / M_PI;

  return true;
}


// CONVERSION DE CASSILA A ANGULOS
// ➡️ Casilla -> XY -> IK (radianes) -> Converción en ángulos
bool chessSquareToAngles(
    const String &square,
    float &theta1Deg,
    float &theta2Deg) {
    float x, y;

    // Casilla -> XY
    if (!chessSquareToXY(square, x, y)) {
        return false;
    }

    float theta1Rad, theta2Rad;

    // XY → IK
    if (!inverseKinematics(
            x,
            y,
            LINK1,
            LINK2,
            theta1Rad,
            theta2Rad)) {
        return false;
    }

    // Rad -> deg
    theta1Deg = theta1Rad * 180.0f / M_PI;
    theta2Deg = theta2Rad * 180.0f / M_PI;

    return true;
}

// DEBUG
// TEST: IMPRIMIR TODAS LAS CASILLAS COMO COORDENADAS XY
// -----------------------------------------------------------------------
void printBoardXY() {
    COMM.println();
    COMM.println("-------- BOARD XY DEBUG --------");

    for (char file = 'A'; file <= 'H'; file++) {
        for (char rank = '1'; rank <= '8'; rank++) {
            String square = String(file) + String(rank);

            float x, y;

            delay(10);

            if (chessSquareToXY(square, x, y)) {
                COMM.print(square);
                COMM.print(" -> X: ");
                COMM.print(x);
                COMM.print(" Y: ");
                COMM.println(y);
            }
        }
    }

    COMM.println("-------- END BOARD --------");
    COMM.println();
}

float shortestAngle(float target, float current) {
    float diff = target - current;

    while (diff > 180)
        diff -= 360;
    while (diff < -180)
        diff += 360;

    return current + diff;
}
