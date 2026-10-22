/*
 * ============================================================
 *  ROBOT FUTBOL - Mando DualShock 4 (PS4)
 *  ESP32-WROOM-32  +  2 modulos BTS7960  +  2 motores DC
 *
 *  Curva de velocidad configurable mediante:
 *      USE_SPEED_CURVE
 *
 *  true  -> usa curva de aceleracion/frenado
 *  false -> respuesta directa del joystick

 *
 *  Controles:
 *      Stick izq (Y) ........... adelante / atras
 *      Stick der (X) ........... giro
 *      R1 ...................... turbo (boton)
 *      L1 ...................... modo lento
 *      R2 ...................... sin funcion
 *
 *  Core requerido: "esp32-bluepad32"
 *  Placa: ESP32 Dev Module -> esp32-bluepad32:esp32:esp32
 * ============================================================
 */
#include "sdkconfig.h"

#include <Arduino.h>
#include <Bluepad32.h>

// ======================================================
// PINES BTS7960
// ======================================================

// Motor izquierdo
const int LEFT_R_PWM  = 18;
const int LEFT_L_PWM  = 19;

// Motor derecho
const int RIGHT_R_PWM = 16;
const int RIGHT_L_PWM = 17;

// ======================================================
// CONFIGURACIÓN
// ======================================================

// Techo normal de PWM.
const int MAX_PWM = 180;

// Techo con R1 (turbo).
const int TURBO_PWM = 250;

// Techo con L1 (modo lento). Menor valor = menor velocidad maxima.
const int SLOW_PWM = 100;

// Zona muerta del joystick.
const int DEADZONE = 70;

// Bluepad32 normaliza los ejes a -512..512.
const int AXIS_MAX = 512;

// Polaridad de cada motor.
// El codigo trabaja con "positivo = adelante". Si al empujar el
// stick hacia arriba una rueda va hacia atras, cambia SOLO el
// valor de esa rueda.
const bool INVERT_LEFT  = false;
const bool INVERT_RIGHT = true;

// ------------------------------------------------------
// PISO DE PWM DEL MOTOR
// ------------------------------------------------------

// PWM minimo con el que el motor realmente arranca.
const int MIN_PWM = 60;

// Valor (0.0 a 1.0) por debajo del cual una rueda se considera
// detenida. Evita que la rueda "quieta" de una diagonal reciba
// el piso de PWM y gire sola.
const float MIN_INPUT = 0.10f;

// Tiempo máximo sin recibir correctamente al mando
// antes de detener los motores.
const unsigned long FAILSAFE_TIME = 500;

// PWM del ESP32: 8 bits (0..255) a 1000 Hz.
const int PWM_BITS = 8;
const int PWM_FREQ = 1000;

// ======================================================
// CURVA DE ACELERACIÓN
// ======================================================

// ------------------------------------------------------
// ACTIVAR / DESACTIVAR CURVA
//
// true  -> aceleracion y frenado progresivos
// false -> respuesta directa del joystick
// ------------------------------------------------------

const bool USE_SPEED_CURVE = false;

// Cambio máximo de PWM por segundo.
const int ACCEL_RATE = 700;

// Frenado: más rápido que la aceleración.
const int DECEL_RATE = 800;

// ======================================================
// MAC DEL MANDO
// ======================================================

const bool WANTED_MAC_ENABLED = true;

const uint8_t WANTED_MAC[6] = {
  0xAC, 0xFD, 0x93, 0x2F, 0xCE, 0xA2
};

// ======================================================
// BARRA DE LUZ
// ======================================================

const bool USE_LIGHTBAR_FEEDBACK = true;

// ======================================================
// VARIABLES
// ======================================================

GamepadPtr gamepad = nullptr;

// Estado interno de la rampa.
int appliedLeftPWM  = 0;
int appliedRightPWM = 0;

unsigned long lastRampMs = 0;

unsigned long lastGamepadUpdate = 0;
unsigned long lastSerialPrint   = 0;

// Último modo de luz: 0 = standby/normal, 1 = lento, 2 = turbo.
int lastLightMode = -1;


// ======================================================
// UTILIDADES
// ======================================================

// Compara la dirección Bluetooth del mando con la buscada.
bool macMatches(const ControllerProperties &props) {

  for (int i = 0; i < 6; i++) {
    if (props.btaddr[i] != WANTED_MAC[i]) {
      return false;
    }
  }

  return true;
}


void printMac(const uint8_t *btaddr) {

  for (int i = 0; i < 6; i++) {

    if (i > 0) {
      Serial.print(":");
    }

    if (btaddr[i] < 0x10) {
      Serial.print("0");
    }

    Serial.print(btaddr[i], HEX);
  }
}


// ------------------------------------------------------
// PARAR MOTORES
// ------------------------------------------------------

void stopMotors() {

  analogWrite(LEFT_R_PWM, 0);
  analogWrite(LEFT_L_PWM, 0);

  analogWrite(RIGHT_R_PWM, 0);
  analogWrite(RIGHT_L_PWM, 0);

  // La parada de emergencia es instantánea.
  appliedLeftPWM  = 0;
  appliedRightPWM = 0;

  lastRampMs = millis();
}


// ------------------------------------------------------
// Convierte el joystick de -512...512 a -1.0...1.0
// aplicando zona muerta.
// ------------------------------------------------------

float processAxis(int value) {

  if (abs(value) <= DEADZONE) {
    return 0.0;
  }

  float sign = (value > 0) ? 1.0 : -1.0;

  float magnitude =
      (float)(abs(value) - DEADZONE) /
      (float)(AXIS_MAX - DEADZONE);

  magnitude = constrain(magnitude, 0.0, 1.0);

  return sign * magnitude;
}


// ------------------------------------------------------
// Convierte un valor -1.0...1.0 a PWM con signo.
//
// Cerca de cero = parado. Cualquier otro valor se reparte
// entre MIN_PWM (donde el motor arranca) y el techo, asi
// todo el recorrido del stick es util.
// ------------------------------------------------------

int toPwm(float value, int ceiling) {

  float magnitude = fabsf(value);

  if (magnitude < MIN_INPUT) {
    return 0;
  }

  int pwm = MIN_PWM + (int)roundf(magnitude * (ceiling - MIN_PWM));

  pwm = min(pwm, ceiling);

  return (value > 0.0f) ? pwm : -pwm;
}


// ------------------------------------------------------
// Control de un motor BTS7960
// ------------------------------------------------------

void setMotor(int speed, bool invert, int rPin, int lPin) {

  speed = constrain(speed, -255, 255);

  if (invert) {
    speed = -speed;
  }

  if (speed > 0) {

    analogWrite(rPin, speed);
    analogWrite(lPin, 0);

  } else if (speed < 0) {

    analogWrite(rPin, 0);
    analogWrite(lPin, -speed);

  } else {

    analogWrite(rPin, 0);
    analogWrite(lPin, 0);
  }
}


// ======================================================
// CURVA DE ACELERACIÓN
// ======================================================

int rampStep(int rate, unsigned long dt) {

  long steps = ((long)rate * (long)dt) / 1000;

  if (steps <= 0) {
    return 0;
  }

  // Si el loop se frenó, no damos un salto gigante.
  if (steps > rate) {
    steps = rate;
  }

  return (int)steps;
}


int approach(int current, int target, int maxStep) {

  if (maxStep <= 0) {
    return current;
  }

  if (current < target) {
    return min(current + maxStep, target);
  }

  if (current > target) {
    return max(current - maxStep, target);
  }

  return target;
}


// ------------------------------------------------------
// Rampa de un motor
//
// Un cambio de sentido pasa primero por cero.
// ------------------------------------------------------

int rampMotor(int current, int target, unsigned long dt) {

  bool acelerando =
      (target != 0) &&
      (abs(target) > abs(current)) &&
      ((current >= 0) == (target >= 0));

  int step = rampStep(
      acelerando ? ACCEL_RATE : DECEL_RATE,
      dt
  );

  return approach(current, target, step);
}


// ======================================================
// CALLBACKS
// ======================================================

void onConnectedGamepad(GamepadPtr gp) {

  ControllerProperties props = gp->getProperties();

  Serial.println();
  Serial.println("==============================");
  Serial.println("      MANDO CONECTADO :) ");
  Serial.println("==============================");

  Serial.print("Modelo  : ");
  Serial.println(gp->getModelName());

  Serial.print("MAC     : ");
  printMac(props.btaddr);

  Serial.print("Bateria : ");
  Serial.println(gp->battery());

  // --------------------------------------------------
  // Filtro por MAC
  // --------------------------------------------------

  if (WANTED_MAC_ENABLED && !macMatches(props)) {

    Serial.println("MAC distinta a la buscada -> IGNORADO.");

    if (gamepad != nullptr) {
      Serial.println("Hay otro mando conectado, no se reemplaza.");
    }

    Serial.println("==============================");
    Serial.println();

    return;
  }

  gamepad = gp;

  lastGamepadUpdate = millis();
  lastLightMode = -1;

  if (USE_LIGHTBAR_FEEDBACK) {
    // Azul = standby/normal al conectar el mando.
    gamepad->setColorLED(0, 0, 180);
    lastLightMode = 0;
  }

  Serial.println("Aceptado.");
  Serial.println("==============================");
  Serial.println();
}


void onDisconnectedGamepad(GamepadPtr gp) {

  Serial.println();
  Serial.println("==============================");
  Serial.println("    MANDO DESCONECTADO :( ");
  Serial.println("==============================");

  stopMotors();

  if (gamepad == gp) {
    gamepad = nullptr;
  }
}


// ======================================================
// CONTROL DEL ROBOT
// ======================================================

void controlRobot() {

  if (gamepad == nullptr) {
    stopMotors();
    return;
  }

  if (!gamepad->isConnected()) {
    stopMotors();
    return;
  }

  // ----------------------------------------------------
  // LEER JOYSTICKS
  // ----------------------------------------------------

  // Bluepad32 entrega Y negativo hacia arriba: se invierte para
  // que "arriba" sea adelante (positivo).
  // Stick izquierdo: solo avance / retroceso.
  // Stick derecho horizontal: giro.
  // L1 activa el modo lento; R1 activa el turbo.
  float forward = -processAxis(gamepad->axisY());
  float turn = processAxis(gamepad->axisRX());

  // ----------------------------------------------------
  // TURBO (R1) Y MODO LENTO (L1)
  // ----------------------------------------------------
  // R1 activa el turbo como boton digital.
  bool turbo = gamepad->r1();
  bool slowMode = gamepad->l1();

  // L1 tiene prioridad si se mantienen ambos controles.
  int ceiling = slowMode ? SLOW_PWM : (turbo ? TURBO_PWM : MAX_PWM);

  // Barra de luz: azul = standby/normal, verde = lento, rojo = turbo.
  // Si L1 y R1 están presionados a la vez, el modo lento tiene prioridad.
  int lightMode = slowMode ? 1 : (turbo ? 2 : 0);

  if (USE_LIGHTBAR_FEEDBACK && lightMode != lastLightMode) {
    lastLightMode = lightMode;

    if (lightMode == 1) {
      gamepad->setColorLED(0, 180, 0);       // Verde: modo lento
    } else if (lightMode == 2) {
      gamepad->setColorLED(180, 0, 0);       // Rojo: turbo
    } else {
      gamepad->setColorLED(0, 0, 180);       // Azul: standby/normal
    }
  }

  // ----------------------------------------------------
  //   // MEZCLA DIFERENCIAL
  //
  // El stick izquierdo controla solamente avance/retroceso.
  // El stick derecho horizontal controla el giro.
  // L1 limita la velocidad maxima; R1 activa el turbo.
  // ----------------------------------------------------

  float left  = forward - turn;
  float right = forward + turn;

  // Normalizacion para no superar el rango -1.0...1.0.
  float maxValue = max(fabsf(left), fabsf(right));

  if (maxValue > 1.0f) {
    left  /= maxValue;
    right /= maxValue;
  }

  // ----------------------------------------------------
  // CONVERTIR A PWM
  // ----------------------------------------------------

  int leftPWM  = toPwm(left,  ceiling);
  int rightPWM = toPwm(right, ceiling);

  // ----------------------------------------------------
  // CURVA DE ACELERACIÓN
  // ----------------------------------------------------

  unsigned long now = millis();

  unsigned long dt = now - lastRampMs;

  lastRampMs = now;

  if (USE_SPEED_CURVE) {

    // ----------------------------------------------
    // CURVA ACTIVADA
    // ----------------------------------------------

    appliedLeftPWM =
        rampMotor(
            appliedLeftPWM,
            leftPWM,
            dt
        );

    appliedRightPWM =
        rampMotor(
            appliedRightPWM,
            rightPWM,
            dt
        );

  } else {

    // ----------------------------------------------
    // CURVA DESACTIVADA
    //
    // El PWM aplicado es directamente el pedido.
    // ----------------------------------------------

    appliedLeftPWM  = leftPWM;
    appliedRightPWM = rightPWM;
  }

  // ----------------------------------------------------
  // ENVIAR A MOTORES
  // ----------------------------------------------------

  setMotor(
    appliedLeftPWM,
    INVERT_LEFT,
    LEFT_R_PWM,
    LEFT_L_PWM
  );

  setMotor(
    appliedRightPWM,
    INVERT_RIGHT,
    RIGHT_R_PWM,
    RIGHT_L_PWM
  );

  lastGamepadUpdate = millis();

  // ----------------------------------------------------
  // MONITOR SERIAL
  // ----------------------------------------------------

  if (millis() - lastSerialPrint >= 200) {

    lastSerialPrint = millis();

    Serial.print("raw X=");
    Serial.print(gamepad->axisX());

    Serial.print(" Y=");
    Serial.print(gamepad->axisY());

    Serial.print(" RX=");
    Serial.print(gamepad->axisRX());

    Serial.print(" RY=");
    Serial.print(gamepad->axisRY());

    Serial.print(" | ");

    Serial.print("F: ");
    Serial.print(forward, 2);

    Serial.print(" | T: ");
    Serial.print(turn, 2);

    Serial.print(" | ");

    if (slowMode) {
      Serial.print("LENTO");
    } else {
      Serial.print(turbo ? "TURBO" : "normal");
    }

    Serial.print(" (max ");
    Serial.print(ceiling);
    Serial.print(")");

    Serial.print(" | Curva: ");
    Serial.print(USE_SPEED_CURVE ? "ON" : "OFF");

    Serial.print(" | L req: ");
    Serial.print(leftPWM);

    Serial.print(" real: ");
    Serial.print(appliedLeftPWM);

    Serial.print(" | R req: ");
    Serial.print(rightPWM);

    Serial.print(" real: ");
    Serial.println(appliedRightPWM);
  }
}


// ======================================================
// SETUP
// ======================================================

void setup() {

  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("==============================");
  Serial.println("ROBOT FUTBOL - DUALSHOCK 4");
  Serial.println("==============================");

  Serial.print("Pines  : izq R=");
  Serial.print(LEFT_R_PWM);

  Serial.print(" L=");
  Serial.print(LEFT_L_PWM);

  Serial.print(" | der R=");
  Serial.print(RIGHT_R_PWM);

  Serial.print(" L=");
  Serial.println(RIGHT_L_PWM);

  Serial.print("PWM    : normal ");
  Serial.print(MAX_PWM);

  Serial.print(" | turbo (R1) ");
  Serial.print(TURBO_PWM);
  Serial.print(" | lento (L1) ");
  Serial.println(SLOW_PWM);

  Serial.print("Piso   : ");
  Serial.println(MIN_PWM);

  Serial.print("Curva  : ");
  Serial.println(
      USE_SPEED_CURVE ? "ACTIVADA" : "DESACTIVADA"
  );

  Serial.print("Invert : izq=");
  Serial.print(
      INVERT_LEFT ? "true" : "false"
  );

  Serial.print("  der=");
  Serial.println(
      INVERT_RIGHT ? "true" : "false"
  );

  Serial.print("MAC    : ");

  printMac(WANTED_MAC);

  Serial.print(
      WANTED_MAC_ENABLED
          ? " (filtro activo)"
          : " (sin filtro)"
  );

  Serial.println();

  Serial.println("==============================");

  // Configuración PWM (API Arduino-ESP32 3.x: resolución y frecuencia por pin).
  pinMode(LEFT_R_PWM, OUTPUT);
  pinMode(LEFT_L_PWM, OUTPUT);

  pinMode(RIGHT_R_PWM, OUTPUT);
  pinMode(RIGHT_L_PWM, OUTPUT);

  analogWriteResolution(LEFT_R_PWM, PWM_BITS);
  analogWriteFrequency(LEFT_R_PWM, PWM_FREQ);

  analogWriteResolution(LEFT_L_PWM, PWM_BITS);
  analogWriteFrequency(LEFT_L_PWM, PWM_FREQ);

  analogWriteResolution(RIGHT_R_PWM, PWM_BITS);
  analogWriteFrequency(RIGHT_R_PWM, PWM_FREQ);

  analogWriteResolution(RIGHT_L_PWM, PWM_BITS);
  analogWriteFrequency(RIGHT_L_PWM, PWM_FREQ);

  stopMotors();

  BP32.setup(
      &onConnectedGamepad,
      &onDisconnectedGamepad
  );

  // Desactivar dispositivo virtual.
  BP32.enableVirtualDevice(false);

  Serial.println("Esperando DualShock...");

  Serial.println(
      "Emparejalo con: boton PS + Share"
  );
}


// ======================================================
// LOOP
// ======================================================

void loop() {

  BP32.update();

  // ----------------------------------------------------
  // FAILSAFE
  // ----------------------------------------------------

  if (gamepad == nullptr) {

    stopMotors();

  } else if (!gamepad->isConnected()) {

    stopMotors();

  } else if (
      millis() - lastGamepadUpdate > FAILSAFE_TIME
  ) {

    stopMotors();

    // Reiniciar el reloj del failsafe.
    lastGamepadUpdate = millis();

  } else {

    controlRobot();
  }

  delay(5);
}
