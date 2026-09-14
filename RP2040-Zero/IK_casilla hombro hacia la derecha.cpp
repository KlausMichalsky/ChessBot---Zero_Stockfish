#include <Arduino.h>
#include <math.h>

// -----------------------------------------------------------------------
// CONFIGURACIÓN FÍSICA Y GEOMETRÍA DEL ROBOT SCARA
// -----------------------------------------------------------------------
const float LINK1 = 175.0f; // Longitud del brazo L1 en mm
const float LINK2 = 95.0f;  // Longitud del brazo L2 en mm

// -----------------------------------------------------------------------
// CONFIGURACIÓN DEL TABLERO DE AJEDREZ (mm)
// -----------------------------------------------------------------------
constexpr float SQUARE_SIZE = 25.0f; 
constexpr float A1_OFFSET_X = -87.5f; // Centro de A1 en X
constexpr float A1_OFFSET_Y = 80.0f;  // Centro de A1 en Y

// Declaración de funciones
bool inverseKinematicsSCARA(float x, float y, float l1, float l2, float &theta1Deg, float &theta2Deg);
bool chessSquareToXY(const String &square, float &x, float &y);
bool chessSquareToAngles(const String &square, float &theta1Deg, float &theta2Deg, float &x, float &y);
void processSerialInput();

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println(F("================================================"));
  Serial.println(F("    ROBOT SCARA - CONTROL POR CASILLAS (A1-H8)  "));
  Serial.println(F("================================================"));
  Serial.print(F("L1: ")); Serial.print(LINK1); Serial.println(F(" mm"));
  Serial.print(F("L2: ")); Serial.print(LINK2); Serial.println(F(" mm"));
  Serial.println(F("------------------------------------------------"));
  Serial.println(F("Introduce una casilla (ejemplo: A1, e4, H8):"));
}

void loop() {
  processSerialInput();
}

// -----------------------------------------------------------------------
// CONVERSIÓN DE CASILLA A COORDENADAS (X, Y)
// -----------------------------------------------------------------------
bool chessSquareToXY(const String &square, float &x, float &y) {
  if (square.length() != 2) return false;

  char file = toupper(square[0]); // Columna A-H
  char rank = square[1];          // Fila 1-8

  if (file < 'A' || file > 'H') return false;
  if (rank < '1' || rank > '8') return false;

  int fileIndex = file - 'A'; // 'A'->0 ... 'H'->7
  int rankIndex = rank - '1'; // '1'->0 ... '8'->7

  x = A1_OFFSET_X + (fileIndex * SQUARE_SIZE);
  y = A1_OFFSET_Y + (rankIndex * SQUARE_SIZE);

  return true;
}

// -----------------------------------------------------------------------
// CINEMÁTICA INVERSA SCARA
// -----------------------------------------------------------------------
bool inverseKinematicsSCARA(float x, float y, float l1, float l2, float &theta1Deg, float &theta2Deg) {
  float r2 = x * x + y * y;
  float r = sqrtf(r2);

  // Validación de alcance físico
  if (r > (l1 + l2) || r < fabsf(l1 - l2)) {
    return false;
  }

  // Ángulo del Codo (Theta 2)
  float cos_theta2 = (l1 * l1 + l2 * l2 - r2) / (2.0f * l1 * l2);
  if (cos_theta2 > 1.0f)  cos_theta2 = 1.0f;
  if (cos_theta2 < -1.0f) cos_theta2 = -1.0f;

  float theta2Rad = acosf(cos_theta2);

  // Ángulo del Hombro (Theta 1)
  float alpha = atan2f(y, x);

  float cos_psi = (l1 * l1 + r2 - l2 * l2) / (2.0f * l1 * r);
  if (cos_psi > 1.0f)  cos_psi = 1.0f;
  if (cos_psi < -1.0f) cos_psi = -1.0f;

  float psi = acosf(cos_psi);

  float theta1Rad = alpha - psi;

  // Conversión a grados
  theta1Deg = theta1Rad * 180.0f / M_PI;
  theta2Deg = theta2Rad * 180.0f / M_PI;

  return true;
}

// -----------------------------------------------------------------------
// CASILLA -> COORDENADAS -> ÁNGULOS
// -----------------------------------------------------------------------
bool chessSquareToAngles(const String &square, float &theta1Deg, float &theta2Deg, float &x, float &y) {
  if (!chessSquareToXY(square, x, y)) {
    return false;
  }

  return inverseKinematicsSCARA(x, y, LINK1, LINK2, theta1Deg, theta2Deg);
}

// -----------------------------------------------------------------------
// PROCESAMIENTO DE ENTRADA SERIE
// -----------------------------------------------------------------------
void processSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    float x = 0.0f, y = 0.0f;
    float theta1 = 0.0f, theta2 = 0.0f;

    if (chessSquareToAngles(input, theta1, theta2, x, y)) {
      Serial.print(F("Casilla: ")); Serial.println(input);
      Serial.print(F(" -> Coordenadas calculadas: X = ")); Serial.print(x, 2);
      Serial.print(F(" mm, Y = ")); Serial.print(y, 2); Serial.println(F(" mm"));
      Serial.println(F(" -> ÁNGULOS CALCULADOS:"));
      Serial.print(F("    Theta 1 (Hombro L1): ")); Serial.print(theta1, 2); Serial.println(F(" °"));
      Serial.print(F("    Theta 2 (Codo L2)  : ")); Serial.print(theta2, 2); Serial.println(F(" °"));
    } else {
      Serial.print(F("[ERROR] Entrada inválida o casilla '"));
      Serial.print(input);
      Serial.println(F("' fuera del alcance del robot."));
    }
    Serial.println(F("------------------------------------------------"));
  }
}