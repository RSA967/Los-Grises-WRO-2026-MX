/*
  ESP32_Obstaculos_OpenMV
  Obstaculos para el robot de India con camara OpenMV.

  Es gemelo de los programas HuskyLens: conserva la misma ley de control,
  los mismos pines del robot y una sola orden de servo y motor por ciclo.
  La unica diferencia importante es la camara: OpenMV envia por UART el ID,
  la posicion y el area del pilar detectado.

  Formato compatible con OPENMV/color_corner.py:
    ID,X,Y,AREA,ROI,COLISION,PARED_NEGRA[,PARKING,X_PARKING,AREA_PARKING]\n
    ID 3 = verde, ID 5 = rojo, ID 0 = sin pilar.
    X llega normalizada de -100 (izquierda) a 100 (derecha).

  Cableado UART2 a 3.3 V:
    OpenMV TX -> FireBeetle D2 / GPIO25 (RX)
    OpenMV RX <- FireBeetle D3 / GPIO26 (TX, opcional)
    OpenMV GND -- FireBeetle GND

  El Nano de ultrasonicos y el BNO085 siguen en I2C IO21/IO22 a 50 kHz.
  La camara usa UART a 19200 baudios y no comparte el bus I2C.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Preferences.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

// ====================================================================
//                  VARIABLES PARA CALIBRAR EL ROBOT
// ====================================================================

// Los Kp del pasillo y de la camara se ajustan aqui. Despues hay que volver
// a cargar el programa al ESP32. Ningun valor guardado en la memoria puede
// reemplazarlos. Los demas parametros conservan sus ajustes opcionales por
// monitor serie.

// ------------------------- Velocidad --------------------------------

// Arath rueda al 40 %. La V2 de India rueda al 70 %, pero esa velocidad
// esta calibrada contra sus maniobras de rescate, que aqui no existen.
// Confirma la trayectoria a 40 y sube de cinco en cinco.
constexpr uint8_t VELOCIDAD_DEFECTO = 70;

// El sentido fisico del motor de este robot sale invertido.
constexpr bool INVERTIR_DIRECCION_MOTOR = true;

// ---------------------- Direccion por pasillo -----------------------

// diferencia = (S1 + S2) - (S4 + S5), en centimetros.
// S1 y S2 miran a la izquierda, S4 y S5 a la derecha.
//
// Control P del pasillo:
//   error = izquierda - derecha
//   correccion = Kp * error
// Arath: angulo = 90 - 0.18 * error, con tope de 18 grados.
constexpr float KP_PASILLO = 4.0f;  // <--- AJUSTAR KP DE PASILLO AQUI
constexpr float TOPE_PASILLO_DEFECTO = 30.0f;

// Cambiar a 1 si el robot se pega a la pared en vez de separarse.
constexpr int8_t DIRECCION_SERVO_PASILLO = -1;

// Recorte de los laterales antes de restar. Ver la nota del encabezado.
constexpr float MAXIMA_DISTANCIA_LATERAL_CM = 150.0f;

// ----------------------- Direccion por camara -----------------------

// OpenMV envia X normalizada entre -100 y 100. El objetivo 62.5 y Kp 0.20
// producen el mismo angulo que objetivo 200 y Kp 0.0625 en HuskyLens 2.
// Rojo a la izquierda de la imagen -> el robot pasa por la derecha.
constexpr float OBJETIVO_X_DEFECTO = 35.0f;
constexpr float KP_CAMARA = 1.0f;  // <--- AJUSTAR KP DE CAMARA AQUI
constexpr float TOPE_CAMARA_DEFECTO = 30.0f;

// Cambiar a -1 si el robot rebasa por el lado equivocado.
constexpr int8_t DIRECCION_SERVO_CAMARA = 1;

// IDs enviados por OPENMV/color_corner.py.
constexpr uint8_t ID_ROJO = 5;
constexpr uint8_t ID_VERDE = 3;

// El firmware OpenMV ya filtra blobs pequenos. Este segundo filtro evita
// usar como objetivo una deteccion con area casi nula.
constexpr uint16_t AREA_MINIMA_PILAR_PX = 30;

// --------------------------- Mision ---------------------------------

constexpr uint8_t ESQUINAS_PARA_TERMINAR = 12;  // 3 vueltas
constexpr float GRADOS_POR_ESQUINA = 90.0f;

// Arath cruza la linea de meta y sigue 250 ms antes de frenar.
constexpr unsigned long MILISEGUNDOS_TRAMO_FINAL = 250;

// ------------------------ Ritmo del ciclo ---------------------------

// 40 Hz. OpenMV se recibe sin bloquear por UART y el paquete del Nano tarda
// unos 2 ms, asi que el ciclo cierra con margen de sobra.
constexpr unsigned long PERIODO_CICLO_MS = 25;

// ====================================================================
//                             PINES
// ====================================================================

constexpr uint8_t PIN_SERVO = 27;  // D4 / IO27
constexpr uint8_t PIN_MOTOR_PWM = D6;
constexpr uint8_t PIN_MOTOR_AIN2 = D7;
constexpr uint8_t PIN_MOTOR_AIN1 = D8;
constexpr uint8_t PIN_ENCODER_A = D5;

constexpr int8_t PIN_OPENMV_RX = 25;  // D2, recibe desde TX de OpenMV
constexpr int8_t PIN_OPENMV_TX = 26;  // D3, envia hacia RX de OpenMV

constexpr uint8_t PIN_SDA = 21;
constexpr uint8_t PIN_SCL = 22;

constexpr uint8_t DIRECCION_NANO = 0x08;
constexpr uint8_t DIRECCION_BNO085_PRIMARIA = 0x4B;
constexpr uint8_t DIRECCION_BNO085_SECUNDARIA = 0x4A;

// El bus va a 50 kHz por el Nano de 5 V: sus pull-ups son de 3.3 V y el
// margen de HIGH es de apenas 0.3 V. Ver extras/README_I2C_SENSORES.md.
constexpr uint32_t FRECUENCIA_I2C_HZ = 50000;
constexpr uint16_t TIMEOUT_I2C_MS = 10;

constexpr uint32_t OPENMV_BAUD = 19200;
constexpr unsigned long OPENMV_TIMEOUT_MS = 500;
constexpr int OPENMV_X_MIN = -100;
constexpr int OPENMV_X_MAX = 100;
constexpr int OPENMV_X_TOLERANCIA = 5;
constexpr int OPENMV_ALTO_IMAGEN_PX = 240;
constexpr int OPENMV_CAMPOS_MINIMOS = 6;
constexpr int OPENMV_CAMPOS_MAXIMOS = 10;
constexpr size_t OPENMV_LINEA_BYTES = 128;
constexpr size_t OPENMV_BUFFER_RX_BYTES = 512;
constexpr int OPENMV_MAXIMO_ATRASO_BYTES = 128;
constexpr uint8_t OPENMV_LINEAS_POR_CICLO = 8;
constexpr unsigned long OPENMV_AVISO_INVALIDO_MS = 1000;

// ====================================================================
//                      CONSTANTES DE HARDWARE
// ====================================================================

constexpr uint8_t CANTIDAD_SENSORES = 5;
constexpr uint8_t BYTES_PAQUETE = CANTIDAD_SENSORES * 2;
constexpr uint16_t DISTANCIA_INVALIDA_MM = 0xFFFF;
constexpr uint16_t DISTANCIA_SIN_ECO_MM = 2000;
constexpr unsigned long TIMEOUT_SENSORES_MS = 250;

constexpr uint8_t INDICE_IZQ_90 = 0;   // S1
constexpr uint8_t INDICE_IZQ_25 = 1;   // S2
constexpr uint8_t INDICE_FRONTAL = 2;  // S3
constexpr uint8_t INDICE_DER_25 = 3;   // S4
constexpr uint8_t INDICE_DER_90 = 4;   // S5

constexpr int SERVO_CENTRO_GRADOS = 90;
constexpr int SERVO_CORRECCION_MAXIMA_GRADOS = 40;
constexpr uint16_t SERVO_PULSO_MINIMO_US = 500;
constexpr uint16_t SERVO_PULSO_MAXIMO_US = 2500;
constexpr uint32_t SERVO_PWM_HZ = 50;
constexpr uint8_t SERVO_PWM_BITS = 16;
constexpr uint32_t MOTOR_PWM_HZ = 20000;
constexpr uint8_t MOTOR_PWM_BITS = 10;

#if ESP_ARDUINO_VERSION_MAJOR < 3
constexpr uint8_t CANAL_PWM_SERVO = 0;
constexpr uint8_t CANAL_PWM_MOTOR = 1;
#endif

// El interruptor del juez corta la potencia del motor, no la del ESP32. El
// PWM se prepara desde el encendido y la ronda arranca cuando el encoder
// confirma que el robot de verdad se movio.
constexpr uint32_t PULSOS_PARA_ARRANCAR = 4;

constexpr uint32_t BNO085_INTERVALO_REPORTE_US = 10000;
constexpr unsigned long BNO085_REINTENTO_MS = 1000;
constexpr unsigned long BNO085_SIN_DATOS_MS = 2000;
constexpr float MAXIMO_SALTO_RUMBO_GRADOS = 90.0f;

constexpr uint8_t BRILLO_LEDS_PORCENTAJE = 20;
constexpr unsigned long INTERVALO_BRILLO_MS = 2000;
constexpr unsigned long INTERVALO_TELEMETRIA_MS = 500;

constexpr uint8_t SERIAL_LINEA_BYTES = 48;

// Tope de bytes atendidos por ciclo: pegar un texto largo no debe retrasar
// los sensores ni el servo.
constexpr uint8_t SERIAL_BYTES_POR_CICLO = 32;

// Espacio propio para que los ajustes de OpenMV no se mezclen con los de
// HuskyLens 1 o 2.
constexpr char NVS_ESPACIO[] = "openmv";

// ====================================================================
//                       COMPROBACIONES EN FRIO
// ====================================================================

static_assert(VELOCIDAD_DEFECTO > 0 && VELOCIDAD_DEFECTO <= 100,
              "La velocidad va de 1 a 100");
static_assert(DIRECCION_SERVO_PASILLO == 1 || DIRECCION_SERVO_PASILLO == -1,
              "La direccion del pasillo solo puede ser 1 o -1");
static_assert(DIRECCION_SERVO_CAMARA == 1 || DIRECCION_SERVO_CAMARA == -1,
              "La direccion de la camara solo puede ser 1 o -1");
static_assert(TOPE_PASILLO_DEFECTO <= SERVO_CORRECCION_MAXIMA_GRADOS,
              "El tope del pasillo no cabe en el recorrido del servo");
static_assert(TOPE_CAMARA_DEFECTO <= SERVO_CORRECCION_MAXIMA_GRADOS,
              "El tope de la camara no cabe en el recorrido del servo");
static_assert(ID_ROJO != ID_VERDE, "Los IDs de color deben ser distintos");
static_assert(KP_PASILLO >= 0.0f, "Kp de pasillo no puede ser negativo");
static_assert(KP_CAMARA >= 0.0f, "Kp de camara no puede ser negativo");
static_assert(ESQUINAS_PARA_TERMINAR > 0, "Hacen falta esquinas que contar");
static_assert(PERIODO_CICLO_MS > 0, "El periodo del ciclo no puede ser 0");

// ====================================================================
//                             ESTADO
// ====================================================================

enum class Color : uint8_t { NINGUNO, ROJO, VERDE };

enum class Fase : uint8_t { ESPERANDO, RODANDO, TRAMO_FINAL, TERMINADO };

struct Pilar {
  Color color = Color::NINGUNO;
  float xCamara = 0.0f;  // normalizada: -100 izquierda, +100 derecha
  uint16_t areaPx = 0;
  int16_t yPx = -1;
  uint8_t roi = 0;
  uint8_t id = 0;
};

// Lo que se calibra en la mesa. Arranca en las constantes de arriba, se
// cambia por el monitor serie y, si se guarda, sobrevive al apagon. Va
// declarado aqui, antes de cualquier funcion, porque el IDE de Arduino
// genera los prototipos al principio del archivo.
struct Ajustes {
  float topePasillo = TOPE_PASILLO_DEFECTO;
  float topeCamara = TOPE_CAMARA_DEFECTO;
  float objetivoX = OBJETIVO_X_DEFECTO;
  uint8_t velocidad = VELOCIDAD_DEFECTO;
};

Adafruit_BNO08x bno085;
sh2_SensorValue_t valorBno085;
HardwareSerial openMVSerial(2);

Fase fase = Fase::ESPERANDO;
Pilar pilar;
Ajustes ajustes;
Preferences memoria;
bool telemetriaActiva = true;

uint16_t distanciasMm[CANTIDAD_SENSORES] = {
    DISTANCIA_SIN_ECO_MM, DISTANCIA_SIN_ECO_MM, DISTANCIA_SIN_ECO_MM,
    DISTANCIA_SIN_ECO_MM, DISTANCIA_SIN_ECO_MM};
unsigned long ultimoPaqueteMs = 0;
bool hayDistancias = false;

bool servoListo = false;
bool motorListo = false;
int ultimoAnguloEscrito = -1;
uint8_t ultimaVelocidadEscrita = 255;

volatile uint32_t pulsosEncoder = 0;

bool bno085Listo = false;
bool bno085ConRumbo = false;
float rumboAnteriorGrados = 0.0f;
float giroAcumuladoGrados = 0.0f;
uint8_t esquinas = 0;
unsigned long ultimoReintentoBnoMs = 0;
unsigned long ultimoCuadroBnoMs = 0;

bool camaraLista = false;
unsigned long ultimoPaqueteCamaraMs = 0;
unsigned long ultimoAvisoOpenMvMs = 0;
uint32_t tramasOpenMvValidas = 0;
uint32_t tramasOpenMvInvalidas = 0;
char lineaOpenMV[OPENMV_LINEA_BYTES];
size_t largoLineaOpenMV = 0;
bool descartandoLineaOpenMV = false;

unsigned long inicioRondaMs = 0;
unsigned long finTramoFinalMs = 0;
unsigned long ultimoCicloMs = 0;
unsigned long ultimoBrilloMs = 0;
unsigned long ultimaTelemetriaMs = 0;

char lineaSerial[SERIAL_LINEA_BYTES];
uint8_t largoLineaSerial = 0;

// ====================================================================
//                            UTILIDADES
// ====================================================================

float normalizarDelta(float gradosDelta) {
  while (gradosDelta > 180.0f) gradosDelta -= 360.0f;
  while (gradosDelta < -180.0f) gradosDelta += 360.0f;
  return gradosDelta;
}

float distanciaLateralCm(uint16_t distanciaMm) {
  const float cm = static_cast<float>(distanciaMm) / 10.0f;
  return cm > MAXIMA_DISTANCIA_LATERAL_CM ? MAXIMA_DISTANCIA_LATERAL_CM : cm;
}

// ====================================================================
//                          SERVO Y MOTOR
// ====================================================================

bool iniciarServo() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcAttach(PIN_SERVO, SERVO_PWM_HZ, SERVO_PWM_BITS);
#else
  ledcSetup(CANAL_PWM_SERVO, SERVO_PWM_HZ, SERVO_PWM_BITS);
  ledcAttachPin(PIN_SERVO, CANAL_PWM_SERVO);
  return true;
#endif
}

void escribirServo(int grados) {
  if (!servoListo) {
    return;
  }

  grados = constrain(grados, 0, 180);

  const uint32_t pulsoUs =
      SERVO_PULSO_MINIMO_US +
      (static_cast<uint32_t>(grados) *
       (SERVO_PULSO_MAXIMO_US - SERVO_PULSO_MINIMO_US)) /
          180UL;

  constexpr uint32_t PERIODO_US = 1000000UL / SERVO_PWM_HZ;
  constexpr uint32_t DUTY_MAXIMO = (1UL << SERVO_PWM_BITS) - 1UL;

  const uint32_t duty =
      (pulsoUs * DUTY_MAXIMO + PERIODO_US / 2UL) / PERIODO_US;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_SERVO, duty);
#else
  ledcWrite(CANAL_PWM_SERVO, duty);
#endif
}

// Unico punto del programa que mueve el servo.
void aplicarDireccion(float grados) {
  grados = constrain(
      grados,
      static_cast<float>(SERVO_CENTRO_GRADOS -
                         SERVO_CORRECCION_MAXIMA_GRADOS),
      static_cast<float>(SERVO_CENTRO_GRADOS +
                         SERVO_CORRECCION_MAXIMA_GRADOS));

  const int redondeado = static_cast<int>(lroundf(grados));
  if (redondeado != ultimoAnguloEscrito) {
    escribirServo(redondeado);
    ultimoAnguloEscrito = redondeado;
  }
}

bool iniciarMotor() {
  pinMode(PIN_MOTOR_AIN1, OUTPUT);
  pinMode(PIN_MOTOR_AIN2, OUTPUT);
  digitalWrite(PIN_MOTOR_AIN1, LOW);
  digitalWrite(PIN_MOTOR_AIN2, LOW);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcAttach(PIN_MOTOR_PWM, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
#else
  ledcSetup(CANAL_PWM_MOTOR, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  ledcAttachPin(PIN_MOTOR_PWM, CANAL_PWM_MOTOR);
  return true;
#endif
}

// Unico punto del programa que mueve el motor. Siempre hacia adelante:
// este programa no tiene reversas ni maniobras de rescate.
void aplicarMotor(uint8_t porcentaje) {
  if (!motorListo) {
    return;
  }

  porcentaje = constrain(porcentaje, 0, 100);

  if (porcentaje == 0) {
    digitalWrite(PIN_MOTOR_AIN1, LOW);
    digitalWrite(PIN_MOTOR_AIN2, LOW);
  } else {
    digitalWrite(PIN_MOTOR_AIN1, INVERTIR_DIRECCION_MOTOR ? LOW : HIGH);
    digitalWrite(PIN_MOTOR_AIN2, INVERTIR_DIRECCION_MOTOR ? HIGH : LOW);
  }

  if (porcentaje == ultimaVelocidadEscrita) {
    return;
  }

  constexpr uint32_t DUTY_MAXIMO = (1UL << MOTOR_PWM_BITS) - 1UL;
  const uint32_t duty =
      (static_cast<uint32_t>(porcentaje) * DUTY_MAXIMO + 50UL) / 100UL;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_MOTOR_PWM, duty);
#else
  ledcWrite(CANAL_PWM_MOTOR, duty);
#endif
  ultimaVelocidadEscrita = porcentaje;
}

void IRAM_ATTR pulsoEncoder() { pulsosEncoder = pulsosEncoder + 1; }

void iniciarEncoder() {
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), pulsoEncoder, RISING);
}

uint32_t leerPulsosEncoder() {
  noInterrupts();
  const uint32_t pulsos = pulsosEncoder;
  interrupts();
  return pulsos;
}

// ====================================================================
//                       ULTRASONICOS POR I2C
// ====================================================================

bool leerDistancias() {
  const uint8_t recibidos =
      Wire.requestFrom(static_cast<uint8_t>(DIRECCION_NANO),
                       static_cast<uint8_t>(BYTES_PAQUETE));

  if (recibidos != BYTES_PAQUETE) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (uint8_t i = 0; i < CANTIDAD_SENSORES; ++i) {
    const uint8_t bajo = Wire.read();
    const uint8_t alto = Wire.read();
    const uint16_t valor =
        static_cast<uint16_t>(bajo) | (static_cast<uint16_t>(alto) << 8);
    distanciasMm[i] =
        valor == DISTANCIA_INVALIDA_MM ? DISTANCIA_SIN_ECO_MM : valor;
  }

  return true;
}

void enviarBrilloLeds(uint8_t porcentaje) {
  Wire.beginTransmission(DIRECCION_NANO);
  Wire.write(static_cast<uint8_t>(constrain(porcentaje, 0, 100)));
  Wire.endTransmission();
}

// ====================================================================
//                         OPENMV POR UART
// ====================================================================

Color colorDeId(long id) {
  if (id == ID_ROJO) return Color::ROJO;
  if (id == ID_VERDE) return Color::VERDE;
  return Color::NINGUNO;
}

long limitarLong(long valor, long minimo, long maximo) {
  if (valor < minimo) return minimo;
  if (valor > maximo) return maximo;
  return valor;
}

// Separa una linea CSV numerica. Los campos posteriores al decimo se ignoran
// para no romper el enlace si el firmware agrega telemetria en el futuro.
int separarCamposOpenMV(const char *linea, long *campos, int capacidad) {
  int cantidad = 0;
  const char *cursor = linea;

  while (*cursor != '\0') {
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    if (*cursor == '\0') break;

    bool negativo = false;
    if (*cursor == '+' || *cursor == '-') {
      negativo = *cursor == '-';
      ++cursor;
    }
    if (*cursor < '0' || *cursor > '9') return -1;

    long valor = 0;
    uint8_t digitos = 0;
    while (*cursor >= '0' && *cursor <= '9') {
      if (digitos >= 9) return -1;
      valor = valor * 10L + static_cast<long>(*cursor - '0');
      ++digitos;
      ++cursor;
    }
    if (negativo) valor = -valor;
    if (cantidad < capacidad) campos[cantidad] = valor;
    ++cantidad;

    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    if (*cursor == ',') {
      ++cursor;
    } else if (*cursor != '\0') {
      return -1;
    }
  }

  return cantidad;
}

bool procesarLineaOpenMV(const char *linea, unsigned long ahoraMs) {
  long campos[OPENMV_CAMPOS_MAXIMOS] = {0};
  const int recibidos =
      separarCamposOpenMV(linea, campos, OPENMV_CAMPOS_MAXIMOS);

  if (recibidos < OPENMV_CAMPOS_MINIMOS) return false;

  const long id = campos[0];
  const long x = campos[1];
  const long y = campos[2];
  const long area = campos[3];
  const long roi = campos[4];
  const long colision = campos[5];

  if ((id != 0 && id != ID_ROJO && id != ID_VERDE) ||
      roi < 0 || roi > 2 ||
      (colision != 0 && colision != ID_ROJO && colision != ID_VERDE)) {
    return false;
  }

  Pilar nuevo;
  if (id != 0) {
    if (x < OPENMV_X_MIN - OPENMV_X_TOLERANCIA ||
        x > OPENMV_X_MAX + OPENMV_X_TOLERANCIA ||
        y < 0 || y >= OPENMV_ALTO_IMAGEN_PX ||
        area < AREA_MINIMA_PILAR_PX || area > UINT16_MAX ||
        (roi != 1 && roi != 2)) {
      return false;
    }

    nuevo.color = colorDeId(id);
    nuevo.id = static_cast<uint8_t>(id);
    nuevo.xCamara = static_cast<float>(
        limitarLong(x, OPENMV_X_MIN, OPENMV_X_MAX));
    nuevo.yPx = static_cast<int16_t>(y);
    nuevo.areaPx = static_cast<uint16_t>(area);
    nuevo.roi = static_cast<uint8_t>(roi);
  }

  pilar = nuevo;
  ultimoPaqueteCamaraMs = ahoraMs;
  ++tramasOpenMvValidas;

  if (!camaraLista) {
    camaraLista = true;
    Serial.println("OpenMV: enlace UART establecido");
  }
  return true;
}

void avisarTramaOpenMvInvalida(unsigned long ahoraMs) {
  ++tramasOpenMvInvalidas;
  if (ahoraMs - ultimoAvisoOpenMvMs >= OPENMV_AVISO_INVALIDO_MS) {
    ultimoAvisoOpenMvMs = ahoraMs;
    Serial.println("AVISO: trama OpenMV invalida");
  }
}

void iniciarCamara() {
  openMVSerial.setRxBufferSize(OPENMV_BUFFER_RX_BYTES);
  openMVSerial.begin(OPENMV_BAUD, SERIAL_8N1,
                     PIN_OPENMV_RX, PIN_OPENMV_TX);
  Serial.print("OpenMV UART2 lista: RX GPIO");
  Serial.print(PIN_OPENMV_RX);
  Serial.print(", TX GPIO");
  Serial.print(PIN_OPENMV_TX);
  Serial.print(", ");
  Serial.print(OPENMV_BAUD);
  Serial.println(" baudios");
}

void actualizarCamara(unsigned long ahoraMs) {
  // Si se acumularon demasiados bytes, son imagenes viejas. Se descarta la
  // parte atrasada y se recupera el encuadre con la siguiente linea completa.
  if (openMVSerial.available() > OPENMV_MAXIMO_ATRASO_BYTES) {
    while (openMVSerial.available() > OPENMV_MAXIMO_ATRASO_BYTES) {
      openMVSerial.read();
    }
    largoLineaOpenMV = 0;
    descartandoLineaOpenMV = true;
  }

  uint8_t lineasAtendidas = 0;
  while (openMVSerial.available() > 0 &&
         lineasAtendidas < OPENMV_LINEAS_POR_CICLO) {
    const char entrada = static_cast<char>(openMVSerial.read());

    if (entrada == '\r') continue;

    if (entrada == '\n') {
      ++lineasAtendidas;
      if (!descartandoLineaOpenMV && largoLineaOpenMV > 0) {
        lineaOpenMV[largoLineaOpenMV] = '\0';
        if (!procesarLineaOpenMV(lineaOpenMV, ahoraMs)) {
          avisarTramaOpenMvInvalida(ahoraMs);
        }
      }
      largoLineaOpenMV = 0;
      descartandoLineaOpenMV = false;
      continue;
    }

    if (descartandoLineaOpenMV) continue;

    if (largoLineaOpenMV < OPENMV_LINEA_BYTES - 1) {
      lineaOpenMV[largoLineaOpenMV++] = entrada;
    } else {
      largoLineaOpenMV = 0;
      descartandoLineaOpenMV = true;
      avisarTramaOpenMvInvalida(ahoraMs);
    }
  }

  if (camaraLista &&
      ahoraMs - ultimoPaqueteCamaraMs > OPENMV_TIMEOUT_MS) {
    camaraLista = false;
    pilar = Pilar();
    Serial.println("AVISO: enlace OpenMV perdido; sigue control por pasillo");
  }
}

// ====================================================================
//                        BNO085 Y CONTEO
// ====================================================================

bool iniciarBnoEn(uint8_t direccion) {
  if (!bno085.begin_I2C(direccion, &Wire)) {
    return false;
  }
  if (!bno085.enableReport(SH2_GAME_ROTATION_VECTOR,
                           BNO085_INTERVALO_REPORTE_US)) {
    return false;
  }

  bno085Listo = true;
  ultimoCuadroBnoMs = millis();
  Serial.print("BNO085 listo en 0x");
  Serial.println(direccion, HEX);
  return true;
}

void iniciarBno() {
  if (iniciarBnoEn(DIRECCION_BNO085_PRIMARIA) ||
      iniciarBnoEn(DIRECCION_BNO085_SECUNDARIA)) {
    return;
  }
  Serial.println("AVISO: BNO085 no detectado en 0x4B ni 0x4A");
}

void procesarRumbo(float rumboGrados, unsigned long ahoraMs) {
  ultimoCuadroBnoMs = ahoraMs;

  if (!bno085ConRumbo) {
    bno085ConRumbo = true;
    rumboAnteriorGrados = rumboGrados;
    return;
  }

  const float delta = normalizarDelta(rumboGrados - rumboAnteriorGrados);
  rumboAnteriorGrados = rumboGrados;

  // La ronda todavia no empieza: el giro acumulado arranca en cero cuando
  // el robot se mueve de verdad.
  if (fase != Fase::RODANDO) {
    return;
  }

  // Un solo cuadro con NaN dejaria el acumulado en NaN para siempre y con
  // el toda la mision.
  if (!isfinite(delta) || fabsf(delta) > MAXIMO_SALTO_RUMBO_GRADOS) {
    return;
  }

  giroAcumuladoGrados += delta;

  const uint8_t esquinasNuevas = static_cast<uint8_t>(
      floorf(fabsf(giroAcumuladoGrados) / GRADOS_POR_ESQUINA));

  if (esquinasNuevas > esquinas) {
    esquinas = min(esquinasNuevas, ESQUINAS_PARA_TERMINAR);
    Serial.print("Esquina ");
    Serial.print(esquinas);
    Serial.print("/");
    Serial.println(ESQUINAS_PARA_TERMINAR);
  }

  if (esquinas >= ESQUINAS_PARA_TERMINAR) {
    fase = Fase::TRAMO_FINAL;
    finTramoFinalMs = ahoraMs + MILISEGUNDOS_TRAMO_FINAL;
    Serial.println("3 vueltas completas: tramo final");
  }
}

void actualizarBno(unsigned long ahoraMs) {
  if (!bno085Listo) {
    if (ahoraMs - ultimoReintentoBnoMs >= BNO085_REINTENTO_MS) {
      ultimoReintentoBnoMs = ahoraMs;
      iniciarBno();
    }
    return;
  }

  // Contesta en I2C pero ya no manda cuaterniones. Reiniciarlo es la unica
  // salida: el conteo de vueltas depende por completo de el.
  if (ultimoCuadroBnoMs != 0 &&
      ahoraMs - ultimoCuadroBnoMs >= BNO085_SIN_DATOS_MS) {
    Serial.println("BNO085 sin datos: se reinicia");
    bno085Listo = false;
    bno085ConRumbo = false;
    ultimoReintentoBnoMs = ahoraMs;
    return;
  }

  if (bno085.wasReset()) {
    bno085ConRumbo = false;
    if (!bno085.enableReport(SH2_GAME_ROTATION_VECTOR,
                             BNO085_INTERVALO_REPORTE_US)) {
      bno085Listo = false;
      return;
    }
  }

  while (bno085.getSensorEvent(&valorBno085)) {
    if (valorBno085.sensorId != SH2_GAME_ROTATION_VECTOR) {
      continue;
    }

    const float qr = valorBno085.un.gameRotationVector.real;
    const float qi = valorBno085.un.gameRotationVector.i;
    const float qj = valorBno085.un.gameRotationVector.j;
    const float qk = valorBno085.un.gameRotationVector.k;
    const float rumbo = atan2f(2.0f * (qi * qj + qk * qr),
                               qi * qi - qj * qj - qk * qk + qr * qr) *
                        RAD_TO_DEG;
    procesarRumbo(rumbo, ahoraMs);
  }
}

// ====================================================================
//                       LEY DE DIRECCION
// ====================================================================

// Centrarse entre las dos paredes. Es la ley de Arath con el centro en 90.
float anguloPorPasillo() {
  const float izquierda = distanciaLateralCm(distanciasMm[INDICE_IZQ_90]) +
                          distanciaLateralCm(distanciasMm[INDICE_IZQ_25]);
  const float derecha = distanciaLateralCm(distanciasMm[INDICE_DER_25]) +
                        distanciaLateralCm(distanciasMm[INDICE_DER_90]);

  const float correccion =
      constrain(KP_PASILLO * (izquierda - derecha),
                -ajustes.topePasillo, ajustes.topePasillo);

  return static_cast<float>(SERVO_CENTRO_GRADOS) +
         static_cast<float>(DIRECCION_SERVO_PASILLO) * correccion;
}

// Empujar el pilar al costado que corresponde y sostenerlo ahi.
// Rojo: queda a la izquierda de la imagen, el robot pasa por la derecha.
// Verde: queda a la derecha de la imagen, el robot pasa por la izquierda.
float anguloPorCamara() {
  const float objetivo =
      pilar.color == Color::ROJO ? -ajustes.objetivoX : ajustes.objetivoX;

  const float correccion =
      constrain(KP_CAMARA * (pilar.xCamara - objetivo),
                -ajustes.topeCamara, ajustes.topeCamara);

  return static_cast<float>(SERVO_CENTRO_GRADOS) +
         static_cast<float>(DIRECCION_SERVO_CAMARA) * correccion;
}

// ====================================================================
//                    AJUSTES POR EL MONITOR SERIE
// ====================================================================

// Los demas parametros se pueden cambiar en caliente a 115200 baudios.
// Los dos Kp no aparecen como comandos: se cambian unicamente en la lista
// superior.
//
//   ver            muestra los valores actuales
//   tope 25        tope de correccion del pasillo, en grados
//   topecam 20     tope de correccion de la camara, en grados
//   objx 200       donde debe quedar el pilar, en pixeles desde el centro
//   vel 45         velocidad del motor, en por ciento
//   tel 0          calla la telemetria para poder escribir; tel 1 la vuelve
//   guardar        graba los valores actuales en la memoria del ESP32
//   reset          vuelve a los valores de compilacion, SIN guardar
//
// Los cambios son inmediatos pero VOLATILES: sin "guardar", el siguiente
// encendido vuelve a lo ultimo guardado. Eso es a proposito, para que un
// experimento malo no se quede pegado.
//
// Leer el serie no toca el camino de control: cambia parametros, no manda
// al servo. La regla de una escritura por ciclo sigue intacta.

void mostrarAjustes() {
  Serial.println("--- ajustes ---");
  Serial.print("  kp      ");
  Serial.print(KP_PASILLO, 3);
  Serial.println(" (se cambia en el codigo)");
  Serial.print("  kp cam  ");
  Serial.print(KP_CAMARA, 4);
  Serial.println(" (se cambia en el codigo)");
  Serial.print("  tope    ");
  Serial.println(ajustes.topePasillo, 1);
  Serial.print("  topecam ");
  Serial.println(ajustes.topeCamara, 1);
  Serial.print("  objx    ");
  Serial.println(ajustes.objetivoX, 0);
  Serial.print("  vel     ");
  Serial.println(ajustes.velocidad);
}

void cargarAjustes() {
  // En modo lectura, begin() falla si todavia no se ha guardado nada.
  if (!memoria.begin(NVS_ESPACIO, true)) {
    Serial.println("Sin ajustes guardados: se usan los de compilacion");
    return;
  }

  ajustes.topePasillo = memoria.getFloat("tope", TOPE_PASILLO_DEFECTO);
  ajustes.topeCamara = memoria.getFloat("topecam", TOPE_CAMARA_DEFECTO);
  ajustes.objetivoX = memoria.getFloat("objx", OBJETIVO_X_DEFECTO);
  ajustes.velocidad = memoria.getUChar("vel", VELOCIDAD_DEFECTO);
  memoria.end();

  Serial.println("Ajustes cargados de la memoria del ESP32");
}

void guardarAjustes() {
  if (!memoria.begin(NVS_ESPACIO, false)) {
    Serial.println("ERROR: no se pudo abrir la memoria");
    return;
  }

  memoria.putFloat("tope", ajustes.topePasillo);
  memoria.putFloat("topecam", ajustes.topeCamara);
  memoria.putFloat("objx", ajustes.objetivoX);
  memoria.putUChar("vel", ajustes.velocidad);
  memoria.end();

  Serial.println("Ajustes guardados");
}

// True si la linea empieza con el nombre y un espacio. El espacio importa:
// sin el, "tope" se comeria "topecam".
bool leerComando(const char *linea, const char *nombre, float &valor) {
  const size_t largo = strlen(nombre);

  if (strncmp(linea, nombre, largo) != 0 || linea[largo] != ' ') {
    return false;
  }

  valor = atof(linea + largo + 1);
  return true;
}

void aplicarComando(const char *linea) {
  float valor = 0.0f;

  if (strcmp(linea, "ver") == 0) {
    mostrarAjustes();
    return;
  }

  if (strcmp(linea, "guardar") == 0) {
    guardarAjustes();
    return;
  }

  if (strcmp(linea, "reset") == 0) {
    ajustes = Ajustes();
    Serial.println("Valores de compilacion restaurados, sin guardar");
    mostrarAjustes();
    return;
  }

  if (leerComando(linea, "tel", valor)) {
    telemetriaActiva = valor != 0.0f;
    Serial.println(telemetriaActiva ? "Telemetria encendida"
                                    : "Telemetria apagada");
    return;
  }

  if (leerComando(linea, "tope", valor)) {
    ajustes.topePasillo = constrain(
        valor, 0.0f, static_cast<float>(SERVO_CORRECCION_MAXIMA_GRADOS));
  } else if (leerComando(linea, "topecam", valor)) {
    ajustes.topeCamara = constrain(
        valor, 0.0f, static_cast<float>(SERVO_CORRECCION_MAXIMA_GRADOS));
  } else if (leerComando(linea, "objx", valor)) {
    ajustes.objetivoX =
        constrain(valor, 0.0f, static_cast<float>(OPENMV_X_MAX));
  } else if (leerComando(linea, "vel", valor)) {
    ajustes.velocidad = static_cast<uint8_t>(constrain(valor, 0.0f, 100.0f));
  } else {
    Serial.print("No entendi: ");
    Serial.println(linea);
    Serial.println("ver | tope | topecam | objx | vel | tel | "
                   "guardar | reset");
    return;
  }

  mostrarAjustes();
}

void atenderSerial() {
  uint8_t atendidos = 0;

  while (Serial.available() > 0 && atendidos < SERIAL_BYTES_POR_CICLO) {
    ++atendidos;
    const char entrada = static_cast<char>(Serial.read());

    if (entrada == '\r') {
      continue;
    }

    if (entrada == '\n') {
      lineaSerial[largoLineaSerial] = '\0';
      if (largoLineaSerial > 0) {
        aplicarComando(lineaSerial);
      }
      largoLineaSerial = 0;
      continue;
    }

    if (largoLineaSerial < SERIAL_LINEA_BYTES - 1) {
      lineaSerial[largoLineaSerial++] =
          static_cast<char>(tolower(static_cast<unsigned char>(entrada)));
    }
  }
}

// ====================================================================
//                          TELEMETRIA
// ====================================================================

void imprimirTelemetria(float angulo, uint8_t velocidad) {
  Serial.print("fase ");
  switch (fase) {
    case Fase::ESPERANDO:
      Serial.print("espera");
      break;
    case Fase::RODANDO:
      Serial.print("rodando");
      break;
    case Fase::TRAMO_FINAL:
      Serial.print("final");
      break;
    case Fase::TERMINADO:
      Serial.print("fin");
      break;
  }

  Serial.print(" | pilar ");
  switch (pilar.color) {
    case Color::ROJO:
      Serial.print("rojo");
      break;
    case Color::VERDE:
      Serial.print("verde");
      break;
    default:
      Serial.print("-");
      break;
  }

  Serial.print(" x ");
  Serial.print(pilar.xCamara, 0);
  Serial.print(" area ");
  Serial.print(pilar.areaPx);
  Serial.print(" cam ");
  Serial.print(camaraLista ? "ok" : "-");

  Serial.print(" | cm 90i ");
  Serial.print(distanciasMm[INDICE_IZQ_90] / 10);
  Serial.print(" 25i ");
  Serial.print(distanciasMm[INDICE_IZQ_25] / 10);
  Serial.print(" fr ");
  Serial.print(distanciasMm[INDICE_FRONTAL] / 10);
  Serial.print(" 25d ");
  Serial.print(distanciasMm[INDICE_DER_25] / 10);
  Serial.print(" 90d ");
  Serial.print(distanciasMm[INDICE_DER_90] / 10);

  Serial.print(" | servo ");
  Serial.print(angulo, 1);
  Serial.print(" motor ");
  Serial.print(velocidad);
  Serial.print(" | giro ");
  Serial.print(giroAcumuladoGrados, 0);
  Serial.print(" esquinas ");
  Serial.println(esquinas);
}

// ====================================================================
//                          SETUP Y LOOP
// ====================================================================

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("ESP32_Obstaculos_OpenMV");

  cargarAjustes();
  mostrarAjustes();

  servoListo = iniciarServo();
  aplicarDireccion(SERVO_CENTRO_GRADOS);

  motorListo = iniciarMotor();
  aplicarMotor(0);

  iniciarEncoder();

  Wire.begin(PIN_SDA, PIN_SCL, FRECUENCIA_I2C_HZ);
  Wire.setTimeOut(TIMEOUT_I2C_MS);

  iniciarBno();
  iniciarCamara();

  enviarBrilloLeds(BRILLO_LEDS_PORCENTAJE);
  ultimoBrilloMs = millis();
  ultimoCicloMs = millis();

  Serial.println("Listo. El motor arranca cuando el juez da potencia.");
}

void loop() {
  const unsigned long ahoraMs = millis();

  if (ahoraMs - ultimoCicloMs < PERIODO_CICLO_MS) {
    return;
  }
  ultimoCicloMs = ahoraMs;

  // ---------------- 1. percepcion ----------------

  actualizarCamara(ahoraMs);

  if (leerDistancias()) {
    ultimoPaqueteMs = ahoraMs;
    hayDistancias = true;
  } else if (hayDistancias &&
             ahoraMs - ultimoPaqueteMs > TIMEOUT_SENSORES_MS) {
    hayDistancias = false;
    Serial.println("AVISO: sin paquete del Nano");
  }

  actualizarBno(ahoraMs);

  // ---------------- 2. arranque de la ronda ----------------

  if (fase == Fase::ESPERANDO &&
      leerPulsosEncoder() >= PULSOS_PARA_ARRANCAR) {
    fase = Fase::RODANDO;
    inicioRondaMs = ahoraMs;
    giroAcumuladoGrados = 0.0f;
    esquinas = 0;
    Serial.println("Ronda iniciada");
  }

  if (fase == Fase::TRAMO_FINAL && ahoraMs >= finTramoFinalMs) {
    fase = Fase::TERMINADO;
    Serial.print("*** MISION TERMINADA en ");
    Serial.print((ahoraMs - inicioRondaMs) / 1000.0f, 1);
    Serial.println(" s ***");
  }

  // ---------------- 3. un angulo y una velocidad ----------------

  float angulo = SERVO_CENTRO_GRADOS;
  uint8_t velocidad = 0;

  if (fase == Fase::TERMINADO) {
    angulo = SERVO_CENTRO_GRADOS;
    velocidad = 0;
  } else if (!hayDistancias) {
    // Sin sensores el robot no sabe donde esta el pasillo. Frenar es la
    // unica respuesta honesta.
    angulo = SERVO_CENTRO_GRADOS;
    velocidad = 0;
  } else if (fase == Fase::TRAMO_FINAL) {
    // Cruzando la meta manda el pasillo: no hay que rebasar nada mas.
    angulo = anguloPorPasillo();
    velocidad = ajustes.velocidad;
  } else if (pilar.color != Color::NINGUNO) {
    angulo = anguloPorCamara();
    velocidad = ajustes.velocidad;
  } else {
    angulo = anguloPorPasillo();
    velocidad = ajustes.velocidad;
  }

  // ---------------- 4. las dos unicas escrituras ----------------

  aplicarDireccion(angulo);
  aplicarMotor(velocidad);

  // ---------------- 5. mantenimiento ----------------

  atenderSerial();

  if (ahoraMs - ultimoBrilloMs >= INTERVALO_BRILLO_MS) {
    ultimoBrilloMs = ahoraMs;
    enviarBrilloLeds(BRILLO_LEDS_PORCENTAJE);
  }

  if (telemetriaActiva &&
      ahoraMs - ultimaTelemetriaMs >= INTERVALO_TELEMETRIA_MS) {
    ultimaTelemetriaMs = ahoraMs;
    imprimirTelemetria(angulo, velocidad);
  }
}
