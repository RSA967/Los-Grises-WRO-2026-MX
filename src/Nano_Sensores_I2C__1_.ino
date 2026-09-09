#include <Wire.h>
#include <Adafruit_NeoPixel.h>

// ====================================================================
// CAMBIA SOLAMENTE ESTE VALOR SEGUN LA TIRA INSTALADA
// Tira actual: 16 LEDs. Para la tira nueva, cambia 16 por 8.
constexpr uint16_t NUMERO_DE_LEDS = 16;
// ====================================================================

// --------------------------- Configuracion ---------------------------
constexpr uint8_t I2C_ADDRESS = 0x08;
constexpr uint8_t SENSOR_COUNT = 5;

constexpr uint8_t TRIGGER_PINS[SENSOR_COUNT] = {2, 4, 6, 8, 10};
constexpr uint8_t ECHO_PINS[SENSOR_COUNT] = {3, 5, 7, 9, 11};

// Distribucion fisica:
// S1 = izquierda 90 grados, S2 = izquierda 25 grados, S3 = frontal,
// S4 = derecha 25 grados y S5 = derecha 90 grados.
// Se lee primero S3 frontal y despues se alternan ambos lados.
constexpr uint8_t SENSOR_SCAN_ORDER[SENSOR_COUNT] = {2, 0, 4, 1, 3};

constexpr uint8_t NEOPIXEL_PIN = 12;

// Color de la tira cuando esta encendida: blanco.
// Puedes cambiar estos tres valores para usar otro color.
constexpr uint8_t LED_RED = 255;
constexpr uint8_t LED_GREEN = 255;
constexpr uint8_t LED_BLUE = 255;

// Para el control no necesitamos esperar ecos de hasta 4 m.
// 12 ms permite medir aproximadamente hasta 2 m y reduce mucho el tiempo
// perdido cuando un sensor no recibe eco.
constexpr unsigned long ECHO_TIMEOUT_US = 12000UL;
constexpr uint16_t MAX_DISTANCE_MM = 2000;

// Con cinco sensores, 10 ms deja aproximadamente 50 ms como minimo antes de
// volver a disparar el mismo sensor. Si aparece interferencia, sube a 15 o 20.
constexpr uint8_t INTER_SENSOR_DELAY_MS = 10;

Adafruit_NeoPixel strip(
    NUMERO_DE_LEDS,
    NEOPIXEL_PIN,
    NEO_GRB + NEO_KHZ800
);

// Estas variables se comparten entre loop() y las interrupciones de I2C.
volatile uint16_t distancesMm[SENSOR_COUNT] = {
    MAX_DISTANCE_MM,
    MAX_DISTANCE_MM,
    MAX_DISTANCE_MM,
    MAX_DISTANCE_MM,
    MAX_DISTANCE_MM
};

volatile uint8_t requestedBrightnessPercent = 0;

// -------------------------- Sensores HC-SR04 -------------------------

uint16_t readDistanceMm(uint8_t triggerPin, uint8_t echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(3);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  const unsigned long durationUs =
      pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);

  if (durationUs == 0) {
    // Sin eco significa que no se detecto un objeto dentro del alcance.
    // Publica la distancia maxima en vez del marcador 0xFFFF.
    return MAX_DISTANCE_MM;
  }

  // Distancia aproximada:
  //   centimetros = duracion_us / 58
  //   milimetros  = duracion_us * 10 / 58
  const unsigned long distanceMm =
      (durationUs * 10UL + 29UL) / 58UL;

  if (distanceMm > MAX_DISTANCE_MM) {
    return MAX_DISTANCE_MM;
  }

  return static_cast<uint16_t>(distanceMm);
}

void scanAllSensors() {
  for (uint8_t position = 0; position < SENSOR_COUNT; ++position) {
    const uint8_t sensorIndex = SENSOR_SCAN_ORDER[position];
    const uint16_t newDistance = readDistanceMm(
        TRIGGER_PINS[sensorIndex],
        ECHO_PINS[sensorIndex]
    );

    // Publica cada lectura inmediatamente; ya no espera a terminar los cinco.
    // En un ATmega328P, copiar uint16_t no es atomico.
    noInterrupts();
    distancesMm[sensorIndex] = newDistance;
    interrupts();

    // Permite aplicar una nueva intensidad sin esperar una ronda completa.
    updateNeoPixelsIfNeeded();
    delay(INTER_SENSOR_DELAY_MS);
  }
}

// ----------------------------- NeoPixel ------------------------------

void updateNeoPixelsIfNeeded() {
  static uint8_t appliedPercent = 0xFF;

  const uint8_t percent = requestedBrightnessPercent;
  if (percent == appliedPercent) {
    return;
  }

  appliedPercent = percent;

  // El valor recibido por I2C siempre esta limitado a 0..100.
  const uint8_t brightness255 =
      static_cast<uint8_t>(map(percent, 0, 100, 0, 255));

  // Se escala el color directamente para evitar perdidas acumuladas al
  // cambiar repetidamente el brillo global de la libreria.
  const uint8_t red =
      (static_cast<uint16_t>(LED_RED) * brightness255) / 255;
  const uint8_t green =
      (static_cast<uint16_t>(LED_GREEN) * brightness255) / 255;
  const uint8_t blue =
      (static_cast<uint16_t>(LED_BLUE) * brightness255) / 255;

  strip.fill(strip.Color(red, green, blue));
  strip.show();
}

// ------------------------------- I2C ---------------------------------

// El ESP32 escribe un byte con la intensidad solicitada, de 0 a 100.
void onI2CReceive(int byteCount) {
  if (byteCount <= 0 || !Wire.available()) {
    return;
  }

  uint8_t percent = Wire.read();

  // Descarta cualquier byte adicional para dejar limpio el buffer.
  while (Wire.available()) {
    Wire.read();
  }

  if (percent > 100) {
    percent = 100;
  }

  requestedBrightnessPercent = percent;
}

// El ESP32 solicita 10 bytes:
// sensor 1 LOW, sensor 1 HIGH, ... sensor 5 LOW, sensor 5 HIGH.
void onI2CRequest() {
  uint8_t packet[SENSOR_COUNT * 2];

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    const uint16_t distance = distancesMm[i];
    packet[i * 2] = static_cast<uint8_t>(distance & 0xFF);
    packet[i * 2 + 1] = static_cast<uint8_t>(distance >> 8);
  }

  Wire.write(packet, sizeof(packet));
}

// ----------------------------- Arduino -------------------------------

void setup() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    pinMode(TRIGGER_PINS[i], OUTPUT);
    digitalWrite(TRIGGER_PINS[i], LOW);
    pinMode(ECHO_PINS[i], INPUT);
  }

  strip.begin();
  strip.clear();
  strip.show();

  // En el Nano clasico: SDA = A4 y SCL = A5.
  Wire.begin(I2C_ADDRESS);

  // La libreria Wire del AVR puede activar pull-ups internos hacia 5 V.
  // Se desactivan porque este bus comparte lineas con un ESP32 de 3.3 V.
  // La DFR0478 ya incluye pull-ups de 10 kohm hacia 3.3 V en SDA y SCL.
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);

  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
}

void loop() {
  updateNeoPixelsIfNeeded();
  scanAllSensors();
  updateNeoPixelsIfNeeded();
}
