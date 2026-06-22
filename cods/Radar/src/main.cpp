#include <Arduino.h>
#include <AFMotor.h>

// Este sketch asume:
// - Motor 28BYJ-48 en el shield tipo Adafruit/SainSmart V1
// - Motor conectado al puerto M1 + M2
// - Sensor ultrasonico PING))) de 3 pines con SIG en A0
// - Salida serial en formato: distancia,angulo

namespace {
// Pin del PING))) usado como trigger y echo.
constexpr uint8_t PIN_SENSOR = A0;
// LED de estado conectado en A1.
constexpr uint8_t PIN_LED_ALERTA = A1;
// Velocidad del puerto serial hacia Python.
constexpr long BAUDIOS = 9600;
// Tiempo maximo de espera del eco.
constexpr long TIMEOUT_ECO_US = 8000;
// Ritmo del parpadeo del LED.
constexpr unsigned long INTERVALO_LED_MS = 300;
// Tiempo que tarda la salida real en completar 360 grados.
constexpr unsigned long PERIODO_GIRO_SALIDA_MS = 22000;
// Rango util de deteccion que se enviara al radar.
constexpr int DISTANCIA_MIN_CM = 2;
constexpr int DISTANCIA_MAX_CM = 100;
// Pasos internos del 28BYJ-48 por vuelta.
constexpr int PASOS_BASE_POR_REVOLUCION = 2048;
// DOUBLE da mas torque que INTERLEAVE y ayuda a que el eje real no se atrase
// respecto al angulo que reporta el programa.
constexpr uint8_t ESTILO_PASO = DOUBLE;
// Referencia de pasos segun el modo seleccionado.
constexpr int PASOS_POR_REVOLUCION =
    (ESTILO_PASO == INTERLEAVE) ? PASOS_BASE_POR_REVOLUCION * 2 : PASOS_BASE_POR_REVOLUCION;
// Cuantos pasos avanza el motor por lectura.
constexpr int PASOS_POR_MUESTRA = 8;
// RPM configuradas en el shield.
constexpr int VELOCIDAD_RPM = 8;

AF_Stepper motor(PASOS_BASE_POR_REVOLUCION, 1);  // Cambiar a 2 si el motor esta en M3 + M4

// Estado actual del LED intermitente.
bool estado_led = false;
// Ultimo instante en que cambio el LED.
unsigned long ultimo_cambio_led_ms = 0;
// Marca de tiempo usada para calcular el angulo reportado.
unsigned long inicio_giro_salida_ms = 0;
}

// Dispara el sensor y convierte el eco a centimetros.
long medir_distancia_cm() {
  pinMode(PIN_SENSOR, OUTPUT);
  digitalWrite(PIN_SENSOR, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_SENSOR, HIGH);
  delayMicroseconds(5);
  digitalWrite(PIN_SENSOR, LOW);

  pinMode(PIN_SENSOR, INPUT);
  long duracion = pulseIn(PIN_SENSOR, HIGH, TIMEOUT_ECO_US);

  if (duracion == 0) {
    return DISTANCIA_MAX_CM;
  }

  long distancia = duracion / 29 / 2;

  // Limita la lectura al rango esperado del proyecto.
  if (distancia < DISTANCIA_MIN_CM) {
    distancia = DISTANCIA_MIN_CM;
  }

  if (distancia > DISTANCIA_MAX_CM) {
    distancia = DISTANCIA_MAX_CM;
  }

  return distancia;
}

// Calcula el angulo segun el tiempo real de una vuelta completa.
float calcular_angulo_actual() {
  unsigned long tiempo_transcurrido_ms = millis() - inicio_giro_salida_ms;
  unsigned long avance_en_ciclo_ms = tiempo_transcurrido_ms % PERIODO_GIRO_SALIDA_MS;
  return (360.0f * avance_en_ciclo_ms) / PERIODO_GIRO_SALIDA_MS;
}

// Envia una muestra en formato compatible con el programa Python.
void enviar_lectura_serial(long distancia, float angulo) {
  Serial.print(distancia);
  Serial.print(',');
  Serial.println(angulo, 1);
}

// Mueve el barrido unos pasos en cada iteracion.
void avanzar_barrido() {
  motor.step(PASOS_POR_MUESTRA, FORWARD, ESTILO_PASO);
}

// Hace parpadear el LED sin bloquear el resto del ciclo.
void actualizar_led_alerta() {
  unsigned long ahora_ms = millis();
  if (ahora_ms - ultimo_cambio_led_ms < INTERVALO_LED_MS) {
    return;
  }

  ultimo_cambio_led_ms = ahora_ms;
  estado_led = !estado_led;
  digitalWrite(PIN_LED_ALERTA, estado_led ? HIGH : LOW);
}

// Inicializa serial, LED, motor y referencia de tiempo.
void setup() {
  Serial.begin(BAUDIOS);
  pinMode(PIN_LED_ALERTA, OUTPUT);
  digitalWrite(PIN_LED_ALERTA, LOW);
  motor.setSpeed(VELOCIDAD_RPM);
  inicio_giro_salida_ms = millis();
  delay(500);
}

// Lee distancia, calcula angulo, reporta y avanza el radar.
void loop() {
  long distancia = medir_distancia_cm();
  float angulo = calcular_angulo_actual();

  enviar_lectura_serial(distancia, angulo);
  avanzar_barrido();
  actualizar_led_alerta();
}
