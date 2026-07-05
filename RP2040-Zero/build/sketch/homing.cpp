#line 1 "/Users/klausmichalsky/Proyectos Mac/KAYRON/RP2040-Zero/homing.cpp"
// =======================================================================
//                          🔹 K A Y R O N 🔹
// =======================================================================
//  Archivo    : homing.cpp
//  Autor      : Klaus Michalsky
//  Fecha      : Feb-2026
// -----------------------------------------------------------------------
//  ▫️ DESCRIPCIÓN:
//      - Ejecutar la máquina de estados de homing
//        para motor1-2 (XY) y motor3 (Z).
//      - Detectar flancos del imán mediante sensor Hall.
//      - Calcular el centro/referencia y posicionar motores.
//      - Gestionar errores y timeouts de homing.
// =======================================================================

#include <Arduino.h>

#include <AccelStepper.h>

#include "command.h"
#include "config.h"
#include "homing.h"
#include "motors.h"
#include "sensors.h"
#include "utils.h"

// CONSTANTES INTERNAS DEL MÓDULO
// -----------------------------------------------------------------------
static const int8_t CW = 1;   // ClockWise plano XY
static const int8_t CCW = -1; // Counter-ClockWise plano XY
static const int8_t dir = 1;  // Dirección inicial eje Z

// VARIABLES INTERNAS DEL MODULO
// -----------------------------------------------------------------------
HomingXY motor1Homing;
HomingXY motor2Homing;
HomingZ motor3Homing;

HomeAllState homeAllState = HomeAllState::IDLE;
HomeSingleState homeSingleState = HomeSingleState::IDLE;
MotorID motorToHome = MotorID::NONE;

// INICIALIZACIÓN DEL ESTADO DE HOMING
// -----------------------------------------------------------------------
void homingInitXY(HomingXY &st) {
    st.state = HomingStateXY::INACTIVE; // estado inicial: homing apagado
    st.startTime = 0;                   // timestamp de inicio de homing
    st.firstEdge = 0;                   // primer flanco detectado (entrada/salida)
    st.secondEdge = 0;                  // segundo flanco detectado
    st.centerPosition = 0;              // centro calculado entre flancos
    st.fault = false;                   // no hay error latcheado
}

void homingInitZ(HomingZ &st) {
    st.state = HomingStateZ::INACTIVE;
    st.startTime = 0;
    st.initialPosition = 0;
    st.edge = 0;      // flanco de salida
    st.reference = 0; // referencia calculada
    st.fault = false;
}

// INICIO DEL PROCESO DE HOMING
// -----------------------------------------------------------------------
void homingStartXY(AccelStepper &motor,
                   const MotorConfig &cfg,
                   HomingXY &st,
                   int hallPin) {
    // Evita reentradas: si el homing ya está activo, no hace nada
    if (st.state != HomingStateXY::INACTIVE)
        return;

    pinMode(cfg.enablePin, OUTPUT);
    digitalWrite(cfg.enablePin, ENABLE_ACTIVE);
    digitalWrite(LED, LOW);

    // Configuración dinámica del motor para homing y referencia temporal al iniciar homing
    motor.setMaxSpeed(cfg.fastSpeed);
    motor.setAcceleration(cfg.acceleration);
    motor.setCurrentPosition(0);

    // Inicializa variables internas del estado
    st.startTime = millis();                  // marca de tiempo de inicio
    st.fault = false;                         // limpia error previo
    st.state = HomingStateXY::SEARCH_FAST_CW; // primer estado del homing
}

void homingStartZ(AccelStepper &motor,
                  const MotorConfig &cfg,
                  HomingZ &st,
                  int hallPin) {
    if (st.state != HomingStateZ::INACTIVE)
        return;

    pinMode(cfg.enablePin, OUTPUT);
    digitalWrite(cfg.enablePin, ENABLE_ACTIVE);
    digitalWrite(LED, LOW);

    motor.setMaxSpeed(cfg.fastSpeed);
    motor.setAcceleration(cfg.acceleration);
    motor.setCurrentPosition(0);

    st.startTime = millis();
    st.fault = false;
    st.state = HomingStateZ::FIND_EDGE_DOWNWARD;
}

// HOMING ACTIVO?
// -----------------------------------------------------------------------
bool homingXYisActive(const HomingXY &st) {
    return st.state != HomingStateXY::INACTIVE;
}
bool homingZisActive(const HomingZ &st) {
    return st.state != HomingStateZ::INACTIVE;
}

// PREGUNTAR ESTADO ACTUAL DEL HOMING
// -----------------------------------------------------------------------
HomingStateXY homingGetStateXY(const HomingXY &st) {
    return st.state; // devuelve el estado actual
}
HomingStateZ homingGetStateZ(const HomingZ &st) {
    return st.state;
}

// COMPROBAR SI HUBO ERROR EN HOMING
// -----------------------------------------------------------------------
bool homingXYhasError(const HomingXY &st) {
    return st.fault; // devuelve true si hubo error
}
bool homingZhasError(const HomingZ &st) {
    return st.fault;
}

// MÁQUINA DE ESTADOS DE HOMING DEL PLANO XY
// -----------------------------------------------------------------------
void homingStepXY(AccelStepper &motor,
                  const MotorConfig &cfg,
                  HomingXY &st,
                  int hallPin) {
    // Invierte la logica del HAll (imán presente = LOW)
    bool imanPresente = (digitalRead(hallPin) == LOW); // activo con pull-up

    // ⏱️ Timeout de homing XY
    if (st.state != HomingStateXY::OK && st.state != HomingStateXY::ERROR) {
        if (millis() - st.startTime > cfg.timeout) {
            st.state = HomingStateXY::ERROR;
        }
    }

    switch (st.state) {
        case HomingStateXY::SEARCH_FAST_CW:
            // Mover rápido en sentido horario hasta detectar imán
            // o hasta alcanzar límite de 90° (cfg.steps90Deg)
            motor.setSpeed(CW * cfg.fastSpeed);
            motor.runSpeed();
            if (imanPresente)
                st.state = HomingStateXY::FIND_FIRST_EDGE_CW; // imán detectado → buscar primer flanco
            else if (motor.currentPosition() > cfg.steps90Deg)
                st.state = HomingStateXY::SEARCH_FAST_CCW; // no detectó imán → revertir dirección
            break;

        case HomingStateXY::SEARCH_FAST_CCW:
            // Mover rápido en sentido antihorario hasta detectar imán
            // o hasta límite de -90° (cfg.steps90Deg)
            motor.setSpeed(CCW * cfg.fastSpeed);
            motor.runSpeed();
            if (imanPresente)
                st.state = HomingStateXY::FIND_FIRST_EDGE_CCW; // imán detectado → buscar primer flanco
            else if (motor.currentPosition() < -cfg.steps90Deg)
                st.state = HomingStateXY::ERROR; // límite alcanzado en ambas direcciones → error
            break;

        case HomingStateXY::FIND_FIRST_EDGE_CW:
            // Detectar el primer flanco de salida horario
            motor.setSpeed(CW * cfg.slowSpeed);
            motor.runSpeed();
            if (!imanPresente) {
                st.firstEdge = motor.currentPosition(); // guardar primer flanco horario
                st.state = HomingStateXY::REVERSE_EDGE_CW;
            }
            break;

        case HomingStateXY::FIND_FIRST_EDGE_CCW:
            // Detectar primer flanco de salida antihorario
            motor.setSpeed(CCW * cfg.slowSpeed);
            motor.runSpeed();
            if (!imanPresente) {
                st.firstEdge = motor.currentPosition(); // guardar primer flanco antihorario
                st.state = HomingStateXY::REVERSE_EDGE_CCW;
            }
            break;

        case HomingStateXY::REVERSE_EDGE_CW:
            // Invertir dirección para encontrar flanco de entrada horario
            motor.setSpeed(CCW * cfg.slowSpeed);
            motor.runSpeed();
            if (imanPresente)
                st.state = HomingStateXY::FIND_SECOND_EDGE_CW;
            break;

        case HomingStateXY::REVERSE_EDGE_CCW:
            // Invertir dirección para encontrar flanco de entrada antihorario
            motor.setSpeed(CW * cfg.slowSpeed);
            motor.runSpeed();
            if (imanPresente)
                st.state = HomingStateXY::FIND_SECOND_EDGE_CCW;
            break;

        case HomingStateXY::FIND_SECOND_EDGE_CW:
            motor.setSpeed(CCW * cfg.slowSpeed);
            motor.runSpeed();
            if (!imanPresente) {
                st.secondEdge = motor.currentPosition(); // guardar segundo flanco
                st.state = HomingStateXY::CALC_CENTER;
            }
            break;

        case HomingStateXY::FIND_SECOND_EDGE_CCW:
            motor.setSpeed(CW * cfg.slowSpeed);
            motor.runSpeed();
            if (!imanPresente) {
                st.secondEdge = motor.currentPosition();
                st.state = HomingStateXY::CALC_CENTER;
            }
            break;

        case HomingStateXY::CALC_CENTER:
            st.centerPosition = (st.firstEdge + st.secondEdge) / 2;
            motor.moveTo(st.centerPosition);
            st.state = HomingStateXY::MOVE_TO_CENTER;
            break;

        case HomingStateXY::MOVE_TO_CENTER:
            motor.run();
            if (motor.distanceToGo() == 0) {
                motor.setCurrentPosition(0);
                digitalWrite(cfg.enablePin, ENABLE_INACTIVE);
                st.state = HomingStateXY::OK;
            }
            break;

        case HomingStateXY::OK:
            break;

        case HomingStateXY::ERROR:
            digitalWrite(cfg.enablePin, ENABLE_INACTIVE);
            st.fault = true;
            break;

        default:
            break;
    }
}

// MÁQUINA DE ESTADOS DE HOMING DEL EJE Z
// -----------------------------------------------------------------------
void homingStepZ(AccelStepper &motor,
                 const MotorConfig &cfg,
                 HomingZ &st,
                 int hallPin) {
    // Invierte la logica del HAll (imán presente = LOW)
    bool imanPresente = (digitalRead(hallPin) == LOW); // activo con pull-up

    // ⏱️ Timeout de homing Z
    if (st.state != HomingStateZ::OK && st.state != HomingStateZ::ERROR) {
        if (millis() - st.startTime > cfg.timeout) {
            st.state = HomingStateZ::ERROR;
        }
    }

    switch (st.state) {
        case HomingStateZ::FIND_EDGE_DOWNWARD:
            motor.setSpeed(dir * cfg.fastSpeed);
            motor.runSpeed();
            if (!imanPresente) {
                st.edge = motor.currentPosition();
                motor.moveTo(st.edge + 500); // avanza 500 pasos para alejarse un poquito del imán
                st.state = HomingStateZ::FIND_EDGE_UPWARD;
            }
            break;

        case HomingStateZ::FIND_EDGE_UPWARD:
            motor.setSpeed(-cfg.slowSpeed);
            motor.runSpeed();
            if (imanPresente) {
                st.edge = motor.currentPosition();

                Serial1.print("EDGE=");
                Serial1.println(st.edge);

                st.state = HomingStateZ::MOVE_TO_REFERENCE;
            } else if (motor.currentPosition() <= cfg.stepsLimit) {
                st.state = HomingStateZ::ERROR;
            }
            break;

        case HomingStateZ::MOVE_TO_REFERENCE:
            motor.setSpeed(cfg.slowSpeed); // bajar
            motor.runSpeed();

            if (motor.currentPosition() >= st.edge + Z_HOME_OFFSET) {
                motor.stop();
                motor.setCurrentPosition(0);
                digitalWrite(cfg.enablePin, ENABLE_INACTIVE);
                st.state = HomingStateZ::OK;
            }
            break;

        case HomingStateZ::OK:
            digitalWrite(LED, HIGH);
            digitalWrite(cfg.enablePin, ENABLE_INACTIVE);
            break;

        case HomingStateZ::ERROR:
            digitalWrite(cfg.enablePin, ENABLE_INACTIVE);
            st.fault = true;
            break;

        default:
            break;
    }
}

// HOMING PARA MOTORES INDEPENDIENTES
// -----------------------------------------------------------------------
void homeSingleMotor() {
    // Motor 1
    if (homingXYisActive(motor1Homing)) {
        homingStepXY(motor1, motor1Config, motor1Homing, HALL_1);
    }
    if (motor1Homing.state == HomingStateXY::OK) {
        COMM.print(motorStatus(MotorID::J1));
        homeSingleState = HomeSingleState::DONE;
        homingInitXY(motor1Homing);
        return;
    }

    // Motor 2
    if (homingXYisActive(motor2Homing)) {
        homingStepXY(motor2, motor2Config, motor2Homing, HALL_2);
    }
    if (motor2Homing.state == HomingStateXY::OK) {
        COMM.print(motorStatus(MotorID::J2));
        homeSingleState = HomeSingleState::DONE;
        homingInitXY(motor2Homing);
        return;
    }

    // Motor 3
    if (homingZisActive(motor3Homing)) {
        homingStepZ(motor3, motor3Config, motor3Homing, HALL_3);
    }
    if (motor3Homing.state == HomingStateZ::OK) {
        COMM.print(motorStatus(MotorID::Z));
        homeSingleState = HomeSingleState::DONE;
        homingInitZ(motor3Homing);
        return;
    }
}

// HOMING PARA TODOS LOS MOTORES
// -----------------------------------------------------------------------
void homeAll() {
    // Ejecutar homings normalmente
    switch (homeAllState) {
        case HomeAllState::MOTOR1:
            if (motor1Homing.state == HomingStateXY::INACTIVE)
                homingStartXY(motor1, motor1Config, motor1Homing, HALL_1);

            homingStepXY(motor1, motor1Config, motor1Homing, HALL_1);

            if (motor1Homing.state == HomingStateXY::OK)
                homeAllState = HomeAllState::MOTOR2;
            break;

        case HomeAllState::MOTOR2:
            if (motor2Homing.state == HomingStateXY::INACTIVE)
                homingStartXY(motor2, motor2Config, motor2Homing, HALL_2);

            homingStepXY(motor2, motor2Config, motor2Homing, HALL_2);

            if (motor2Homing.state == HomingStateXY::OK)
                homeAllState = HomeAllState::MOTOR3;
            break;

        case HomeAllState::MOTOR3:
            if (motor3Homing.state == HomingStateZ::INACTIVE)
                homingStartZ(motor3, motor3Config, motor3Homing, HALL_3);

            homingStepZ(motor3, motor3Config, motor3Homing, HALL_3);

            if (motor3Homing.state == HomingStateZ::OK) {
                homeAllState = HomeAllState::DONE;
                commandSendStatusReport();

                COMM.println("DONE"); // 👈 ESTO ES LO IMPORTANTE
                                      // Manda un mensaje de "DONE" al finalizar el homing de todos los motores,
                                      // para que el Raspi sepa que puede continuar con el primer movimiento.
                delay(100);
                homingInitXY(motor1Homing);
                homingInitXY(motor2Homing);
                homingInitZ(motor3Homing);

                digitalWrite(LED, HIGH);

                // 🔥 GUARDAR OFFSET SOLO UNA VEZ
                delay(200);
                sensor1Offset = sensorHomingOffset(Wire);
                delay(200);
                sensor2Offset = sensorHomingOffset(Wire1);
            }
            break;

        case HomeAllState::DONE:
            homeAllState = HomeAllState::IDLE;

            break;

        default:
            break;
    }
}
