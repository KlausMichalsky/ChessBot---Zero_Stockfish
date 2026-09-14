// ============================================================================
// @file scara_chess.h
// @brief Cinemática inversa para Robot SCARA.
//
// Convierte coordenadas cartesianas (X, Y)
// en milímetros y calcula los ángulos de articulación (Theta 1 y Theta 2)
// mediante cinemática inversa.
// ============================================================================

#include <Arduino.h>
#include <math.h>

// -----------------------------------------------------------------------
// CONFIGURACIÓN FÍSICA Y GEOMETRÍA DEL ROBOT SCARA
// -----------------------------------------------------------------------
const float LINK1 = 175.0f; // Longitud del brazo L1 en mm
const float LINK2 = 95.0f;  // Longitud del brazo L2 en mm

bool inverseKinematicsSCARA(float x, float y, float l1, float l2, float &theta1Deg, float &theta2Deg);
void processSerialInput();

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println(F("================================================"));
  Serial.println(F("    ROBOT SCARA - CINEMÁTICA INVERSA"));
  Serial.println(F("================================================"));
  Serial.print(F("L1: ")); Serial.print(LINK1); Serial.println(F(" mm"));
  Serial.print(F("L2: ")); Serial.print(LINK2); Serial.println(F(" mm"));
  Serial.println(F("------------------------------------------------"));
  Serial.println(F("Introduce coordenadas X,Y (ejemplo: 87.5,80):"));
}

void loop() {
  processSerialInput();
}

// -----------------------------------------------------------------------
// CINEMÁTICA INVERSA SCARA
// -----------------------------------------------------------------------
bool inverseKinematicsSCARA(float x, float y, float l1, float l2, float &theta1Deg, float &theta2Deg) {
  // 1. Distancia desde el origen (0,0) al objetivo
  float r2 = x * x + y * y;
  float r = sqrtf(r2);

  // 2. Validación de alcance del brazo
  if (r > (l1 + l2) || r < fabsf(l1 - l2)) {
    return false; // Punto fuera de alcance
  }

  // 3. Ángulo del Codo (Theta 2)
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

// -----------------------------------------------------------------------
// PROCESAMIENTO DE ENTRADA SERIE
// -----------------------------------------------------------------------
void processSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    int commaIndex = input.indexOf(',');
    if (commaIndex == -1) {
      Serial.println(F("[ERROR] Usa el formato X,Y (ejemplo: 87.5,80)"));
      return;
    }

    float targetX = input.substring(0, commaIndex).toFloat();
    float targetY = input.substring(commaIndex + 1).toFloat();

    Serial.print(F("Entrada -> X: ")); Serial.print(targetX);
    Serial.print(F(" mm, Y: ")); Serial.print(targetY); Serial.println(F(" mm"));

    float theta1 = 0.0f, theta2 = 0.0f;

    if (inverseKinematicsSCARA(targetX, targetY, LINK1, LINK2, theta1, theta2)) {
      Serial.println(F("-> ÁNGULOS CALCULADOS:"));
      Serial.print(F("   Theta 1 (Hombro L1): ")); Serial.print(theta1, 2); Serial.println(F(" °"));
      Serial.print(F("   Theta 2 (Codo L2)  : ")); Serial.print(theta2, 2); Serial.println(F(" °"));
    } else {
      Serial.println(F("[ERROR] Coordenada FUERA DEL ALCANCE del robot."));
    }
    Serial.println(F("------------------------------------------------"));
  }
}
