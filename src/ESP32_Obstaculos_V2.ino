/*
  Version 2: obstaculos, seguimiento de pasillo y telemetria de ronda.

  Prioridad de direccion:

    1. Si S3 detecta una pared demasiado cerca, el robot frena, centra
       el servo y retrocede hasta recuperar espacio.
    2. Mientras OpenMV vea un pilar valido, la camara controla el servo.
    3. Al perder el pilar, el servo vuelve gradualmente al objetivo que
       calcula el PID del pasillo.
    4. Sin pilar, los ultrasonicos laterales controlan normalmente el servo.

  El BNO085 no dirige: solo cuenta las vueltas de la mision.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_arduino_version.h>
#include <math.h>

// ====================================================================
//                  VARIABLES PARA CALIBRAR EL ROBOT
// ====================================================================

// ----------------------- Velocidad del motor ------------------------

constexpr uint8_t MOTOR_SPEED_PERCENT = 70; /////////////////////////////////speed
constexpr uint8_t MOTOR_CORNER_SPEED_PERCENT = 50; /////////////////////////////////speed

// Al detectar un pilar, limita inmediatamente la velocidad a este valor.
// Conforme el pilar se acerca, baja hasta MOTOR_CORNER_SPEED_PERCENT.
constexpr uint8_t MOTOR_PILLAR_FAR_SPEED_PERCENT = 70; ////////////////////////////////speed
constexpr bool INVERT_MOTOR_DIRECTION = true;

// Maniobra al detectar un color dentro de COLLISION_ROI.
constexpr uint8_t COLLISION_REVERSE_SPEED_PERCENT = 30;
constexpr unsigned long COLLISION_STOP_BEFORE_REVERSE_MS = 100;
constexpr uint16_t COLLISION_REVERSE_DISTANCE_MM = 100;
constexpr unsigned long COLLISION_REVERSE_MAX_MS = 1500;
constexpr uint8_t COLLISION_TRIGGER_CONFIRM_FRAMES = 2;
constexpr uint8_t COLLISION_CLEAR_CONFIRM_FRAMES = 3;

constexpr uint8_t MOTOR_PWM_PIN = D6;
constexpr uint8_t MOTOR_AIN2_PIN = D7;
constexpr uint8_t MOTOR_AIN1_PIN = D8;

// El interruptor del juez corta la potencia del motor, pero el ESP32 sigue
// preparando este PWM. La rampa comienza cuando el encoder confirma movimiento.
constexpr uint8_t MOTOR_START_DETECTION_PERCENT = 45;
constexpr uint8_t ENCODER_START_PULSES = 4;
constexpr unsigned long ENCODER_DETECTION_WINDOW_MS = 600;
constexpr unsigned long MOTOR_SOFT_START_DURATION_MS = 5000;
constexpr uint8_t ENCODER_A_PIN = D5;
constexpr uint8_t ENCODER_B_PIN = D9;

// Calibracion de 1 metro: prueba 1 = 3870, prueba 2 = 3988.
// Promedio = 3929 pulsos/m = 0.2545 mm por pulso.
constexpr float ENCODER_MM_PER_PULSE = 1000.0f / 3929.0f;
constexpr int8_t ODOMETRY_DIRECTION_STOPPED = 0;
constexpr int8_t ODOMETRY_DIRECTION_FORWARD = 1;
constexpr int8_t ODOMETRY_DIRECTION_REVERSE = -1;

// ----------------------- PID para el pasillo ------------------------

// S1 y S2: lado izquierdo. S3: frente. S4 y S5: lado derecho.
// Error = (S1*W1 + S2*W2) - (S4*W4 + S5*W5).
constexpr float SENSOR_WEIGHT_S1 = 1.0f;
constexpr float SENSOR_WEIGHT_S2 = 1.0f;
constexpr float SENSOR_WEIGHT_S4 = 1.0f;
constexpr float SENSOR_WEIGHT_S5 = 1.0f;

constexpr float CORRIDOR_KP = 0.12f;
constexpr float CORRIDOR_KI = 0.0015f;
constexpr float CORRIDOR_KD = 0.006f;
constexpr float CORRIDOR_INTEGRAL_LIMIT_DEGREES = 8.0f;
constexpr float CORRIDOR_DERIVATIVE_FILTER = 0.25f;

// Cambiar a 1 si el PID del pasillo gira al lado contrario.
constexpr int8_t CORRIDOR_SERVO_DIRECTION = -1;

constexpr uint16_t MAX_LATERAL_DISTANCE_MM = 1500;
constexpr uint16_t FRONT_CORNER_START_MM = 600;
constexpr uint16_t FRONT_CORNER_CRITICAL_MM = 180;
constexpr float CORNER_KP_MULTIPLIER = 1.8f;

// ---------------------- Direccion por OpenMV ------------------------

constexpr int OPENMV_X_MIN = -100;
constexpr int OPENMV_X_MAX = 100;

constexpr int LOW_ROI_CONE_FAR_Y = 130;
constexpr int LOW_ROI_CONE_NEAR_Y = 200;
constexpr int LOW_ROI_CONE_FAR_HALF_WIDTH = 30;
constexpr int LOW_ROI_CONE_NEAR_HALF_WIDTH = 65;

constexpr int HIGH_ROI_STEERING_GAIN_PERCENT = 200;
constexpr int LOW_ROI_EVASION_GAIN_PERCENT = 300;

// Cambiar a -1 si la direccion por camara gira al lado contrario.
constexpr int8_t OPENMV_SERVO_DIRECTION = 1;

// ------------------- Entrega camara -> ultrasonicos -----------------

// Despues de alcanzar ROI_LOW, el ID del pilar queda enclavado. La camara
// conserva el control hasta que los sensores del lado por el que se rebasa
// confirman el pilar. El bloqueo se conserva hasta que ese lado se despeja,
// para impedir que otro pilar mueva el servo antes de tiempo.
//
// Verde: el robot pasa por la izquierda y el pilar queda a su derecha (S4/S5).
// Rojo: el robot pasa por la derecha y el pilar queda a su izquierda (S1/S2).
constexpr uint16_t PILLAR_SIDE_DETECT_MM = 300;
constexpr uint16_t PILLAR_SIDE_CLEAR_MM = 360;
constexpr uint8_t PILLAR_SIDE_CONFIRM_SAMPLES = 2;

// Aunque el costado ya parezca libre, conserva el ID actual durante este
// tiempo continuo. Evita que una linea de esquina o el siguiente pilar cambien
// el objetivo inmediatamente despues del rebase.
constexpr unsigned long PILLAR_POST_CLEAR_HOLD_MS =100;    /////despues del pasarlo

// Al quedar el pilar junto al robot se deja de perseguir su ultima coordenada
// de camara, pero se conserva un giro moderado para terminar de rodearlo.
constexpr int PILLAR_WRAP_CORRECTION_DEGREES = 20;
constexpr uint8_t PILLAR_WRAP_SPEED_PERCENT = 35;

// El rodeo termina cuando los ultrasonicos de ese costado ven libre. Si lo
// que tienen enfrente es una pared y no el pilar, nunca ven libre: sin
// este tope el robot se queda girando contra la pared hasta chocar.
constexpr unsigned long PILLAR_WRAP_MAX_MS = 900;

// Con la pared de frente (S3 por debajo de FRONT_GUARD_ACTIVATE_MM) el
// pilar esta pegado a esa pared y rodearlo exige una maniobra mas larga:
// cortar a los 900 ms deja al robot atravesado. Mismo criterio que ya
// aplican FRONT_GUARD_HANDOFF_TIMEOUT_MS y
// FRONT_GUARD_LOCK_RELEASE_TIMEOUT_MS.
//
// El escape de pared de S3 a 15 cm sigue teniendo prioridad sobre esto,
// asi que alargar el rodeo no quita la proteccion contra el choque.
constexpr unsigned long FRONT_GUARD_WRAP_MAX_MS = 1600;

// Cambio de pilar (el caso verde -> rojo).
//
// Mientras un pilar esta enclavado, las tramas de OTRO color se ignoran
// por completo, pero el motor sigue dirigiendo hacia las ULTIMAS
// coordenadas del enclavado. Si el verde ya se rebaso y la camara solo ve
// rojo, el robot sigue girando hacia donde estaba el verde y se mete en la
// pared. Cuando el enclavado lleva este tiempo sin verse y la camara
// insiste con otro, se suelta el bloqueo para que el nuevo tome el mando.
//
// Nunca se abandona un rebase en curso: si el pilar enclavado sigue a la
// vista, su marca de tiempo se refresca y el cambio no se dispara.
constexpr unsigned long PILLAR_SWITCH_STALE_MS = 220;

// Durante el rodeo el pilar ya quedo al costado y la camara no lo ve, asi
// que ahi se exige mas espera antes de soltarlo.
constexpr unsigned long PILLAR_SWITCH_WRAP_STALE_MS = 450;

constexpr uint8_t PILLAR_SWITCH_CONFIRM_FRAMES = 3;

// Sentido propio de la fase de salida. En este montaje debe ser opuesto al
// signo usado para interpretar la correccion visual de aproximacion.
constexpr int8_t PILLAR_WRAP_DIRECTION = -1;

// Protecciones por si un ultrasonico no alcanza a ver el pilar. Primero se
// entrega el servo al PID del pasillo, pero se siguen ignorando pilares nuevos.
// Finalmente se libera el bloqueo para no quedar enclavado indefinidamente.
constexpr unsigned long PILLAR_SENSOR_HANDOFF_TIMEOUT_MS = 700;
constexpr unsigned long PILLAR_LOCK_RELEASE_TIMEOUT_MS = 1400;
constexpr unsigned long PILLAR_HIGH_LOSS_TIMEOUT_MS = 500;

// ---------- Proteccion lateral mientras dirige la camara ------------
//
// Mientras el pilar tiene el volante, el sketch ignora a proposito los
// ultrasonicos laterales: el robot esquiva hacia un lado aunque por ese
// lado haya pared, y solo S3 puede frenarlo de frente. Este limitador
// recorta el giro conforme se acerca la pared de ESE costado; el giro que
// aleja de la pared no se toca nunca.
//
// Es un compromiso: con el robot pegado a una pared el rodeo se recorta y
// puede rozar el pilar en vez de la pared. Subir SIDE_WALL_BLOCK_MM da mas
// margen a la pared; bajarlo deja esquivar mas.
constexpr bool PILLAR_SIDE_WALL_GUARD_ENABLED = true;

// Por encima de esta distancia el limitador no interviene.
constexpr uint16_t SIDE_WALL_FREE_MM = 300;

// A esta distancia el giro hacia esa pared queda anulado por completo.
constexpr uint16_t SIDE_WALL_BLOCK_MM = 130;

// Velocidad maxima mientras el limitador esta recortando el giro.
constexpr uint8_t SIDE_WALL_SPEED_PERCENT = 45;

// ---------------- Proteccion frontal cerca de la pared --------------

// Los cuadros de los pilares dejan aproximadamente 40 cm hasta la pared.
// S3 activa la proteccion debajo de esa distancia y la desactiva a 45 cm
// para evitar oscilaciones alrededor del umbral.
constexpr uint16_t FRONT_GUARD_ACTIVATE_MM = 400;
constexpr uint16_t FRONT_GUARD_RELEASE_MM = 450;
constexpr uint8_t FRONT_GUARD_CONFIRM_SAMPLES = 3;
constexpr uint8_t FRONT_GUARD_SPEED_PERCENT = 70; /////////////////////////////////speed
// Refuerzo del rodeo cuando la pared se acerca de frente con el pilar ya al
// costado. Antes era un valor fijo de 8 grados y se quedaba corto: al entrar
// a una esquina con un pilar al lado hay que cerrar mas para librarlo antes
// de llegar a la pared.
//
// Ahora crece con la cercania: a FRONT_GUARD_ACTIVATE_MM anade el minimo y a
// FRONT_GUARD_MAX_STEER_MM el maximo, sin escalones que den tirones.
constexpr int FRONT_GUARD_EXTRA_STEERING_DEGREES = 8;
constexpr int FRONT_GUARD_MAX_EXTRA_STEERING_DEGREES = 20;
constexpr uint16_t FRONT_GUARD_MAX_STEER_MM = 200;
constexpr unsigned long FRONT_GUARD_HANDOFF_TIMEOUT_MS = 1200;
constexpr unsigned long FRONT_GUARD_LOCK_RELEASE_TIMEOUT_MS = 2200;

// Escape de emergencia igual al de Open. Es independiente del refuerzo de
// evasion de 40 cm: solamente entra cuando S3 ya esta demasiado cerca.
constexpr uint16_t WALL_ESCAPE_TRIGGER_MM = 150;
constexpr uint16_t WALL_ESCAPE_CLEAR_MM = 250;
constexpr uint8_t WALL_ESCAPE_REVERSE_SPEED_PERCENT = 45;
constexpr unsigned long WALL_ESCAPE_BRAKE_MS = 150;
constexpr unsigned long WALL_ESCAPE_MAX_REVERSE_MS = 2500;

// Al perder un pilar, esta velocidad limita el regreso al PID del pasillo.
// Un valor menor hace la transicion mas suave; uno mayor la hace mas rapida.
constexpr float CORRIDOR_RETURN_RATE_DEGREES_PER_SECOND = 45.0f;

// La entrega gradual existe para que el volante no pegue un tiron al pasar
// de la camara a los sensores. Entrando a una esquina es justo lo contrario
// de lo que hace falta: el robot tiene que girar ya, y limitarlo a 45
// grados por segundo lo manda contra la pared exterior. Con la pared de
// frente, o pasado este tiempo, el PID toma el volante de golpe.
constexpr uint16_t CORRIDOR_RETURN_ABORT_FRONT_MM = 700;
constexpr unsigned long CORRIDOR_RETURN_MAX_MS = 700;

// ----------------------------- Servo --------------------------------

constexpr uint8_t SERVO_PIN = 27;  // D4 / GPIO27
constexpr int SERVO_CENTER_DEGREES = 90;
constexpr int SERVO_MAX_CORRECTION_DEGREES = 40;
constexpr uint16_t SERVO_MIN_PULSE_US = 500;
constexpr uint16_t SERVO_MAX_PULSE_US = 2500;

// ----------------------- Iluminacion automatica ---------------------

// Luz durante el estacionamiento. Es un valor ABSOLUTO y manda sobre el
// ajuste automatico mientras dura la maniobra, asi que un 0 aqui APAGA los
// LEDs justo cuando el robot va a buscar las paredes.
//
// Se deja igual que la luz de las vueltas: asi los umbrales de la camara se
// calibran una sola vez y nada cambia bajo sus pies al entrar a la maniobra.
constexpr uint8_t PARKING_LED_BRIGHTNESS_PERCENT = 50;

// Brillo de las vueltas. Subido de 30 a 70: con luz alta los tres colores
// se separan mucho mejor en LAB.
//
// Los dos valores iguales dejan la luz fija, sin atenuar al acercarse.
//
// OJO: los umbrales de rojo y verde hay que calibrarlos CON ESTA LUZ.
constexpr uint8_t LED_BRIGHTNESS_FAR_PERCENT = 50;
constexpr uint8_t LED_BRIGHTNESS_NEAR_PERCENT = 50;
constexpr uint8_t LED_RETURN_STEP_PERCENT = 1;
constexpr unsigned long LED_RETURN_INTERVAL_MS = 30;

constexpr int PILLAR_FAR_Y = 70;
constexpr int PILLAR_NEAR_Y = 190;

// S3 complementa la altura aparente del pilar. La camara confirma que hay
// un pilar y estas distancias convierten la lectura frontal a intensidad.
constexpr uint16_t LED_FRONT_FAR_MM = 600;
constexpr uint16_t LED_FRONT_NEAR_MM = 100;
constexpr unsigned long FRONT_DISTANCE_FRESH_MS = 150;

// ------------------ Estacionamiento final ---------------------------
//
// Al completar las vueltas el robot no se detiene: busca el hueco entre
// las dos paredes magenta y entra de reversa.
//
// Secuencia: BUSCA -> ACOMPANA -> REBASA -> FRENA -> ENTRA -> ENDEREZA.
//
// De que lado esta el hueco NO se deduce del sentido de la vuelta: la
// camara ya lo dice directamente con el signo de parkingX, que es
// evidencia directa y no depende de cual sea el convenio del BNO085.
// El sentido de giro queda solo como respaldo si la camara nunca lo vio.
constexpr bool PARKING_ENABLED = true;

// Area minima de la pared para darla por vista de verdad. Filtra destellos
// lejanos que aparecen y desaparecen.
constexpr int16_t PARKING_MIN_AREA = 250;
constexpr uint8_t PARKING_SEEN_FRAMES = 3;
constexpr uint8_t PARKING_LOST_FRAMES = 6;

// Cuantas paredes hay que dejar atras antes de frenar. Dos es lo normal:
// se rebasa la primera, luego la segunda, y el hueco queda al costado.
// Ponlo en 1 si en la camara las dos paredes se fusionan en un solo blob.
constexpr uint8_t PARKING_WALLS_TO_PASS = 2;

// Velocidades de la maniobra.
//
// Buscar y acercarse son cosas distintas. Al salir de la tercera vuelta el
// hueco puede estar en cualquier punto del circuito, asi que puede hacer
// falta recorrer una vuelta entera para encontrarlo: ir despacio ahi solo
// gasta el tiempo de la ronda. Una vez que la pared esta a la vista si
// conviene bajar, porque ahi cada centimetro cuenta.
constexpr uint8_t PARKING_SEARCH_SPEED_PERCENT = 50;
constexpr uint8_t PARKING_APPROACH_SPEED_PERCENT = 45;
constexpr uint8_t PARKING_REVERSE_SPEED_PERCENT = 32;

// Sesgo del volante para arrimarse a la pared exterior mientras busca el
// hueco. Se aplica sobre el PID de pasillo, no en su lugar: el robot sigue
// esquivando paredes, solo que rueda mas cerca del lado donde debe estar el
// estacionamiento.
//
// Subirlo lo pega mas a esa pared; pasarse hace que el PID pelee contra el
// sesgo y el robot vaya en diagonal.
constexpr int PARKING_SIDE_BIAS_DEGREES = 10;

// Tras perder de vista la ultima pared, el hueco todavia queda adelante:
// la pared sale del campo de vision antes de que el robot llegue a su
// altura. Este avance por encoder cubre esa diferencia.
constexpr uint16_t PARKING_PASS_DISTANCE_MM = 260;

// Reversa en dos tramos, como un estacionamiento en paralelo: primero
// entra girando hacia el hueco, luego contravolantea para enderezarse.
constexpr uint16_t PARKING_REVERSE_TURN_MM = 240;
constexpr uint16_t PARKING_REVERSE_STRAIGHT_MM = 150;
constexpr int PARKING_REVERSE_STEER_DEGREES = 34;
constexpr int PARKING_STRAIGHTEN_STEER_DEGREES = 26;

constexpr unsigned long PARKING_BRAKE_MS = 250;

// Topes de seguridad. Sin sensor trasero, la reversa se acota por encoder
// y por tiempo; si algo se atasca, el robot para y da por terminada la
// ronda en vez de seguir empujando.
// Durante el estacionamiento el failsafe de sensores esta desactivado a
// proposito, para que un hueco de I2C no corte la reversa. Pero si el Nano
// lleva mudo este tiempo, ya no es un hueco: es una averia.
constexpr unsigned long PARKING_SENSOR_MUTE_ABORT_MS = 1500;

constexpr unsigned long PARKING_PHASE_MAX_MS = 6000;

// Tiene que alcanzar para una vuelta completa a velocidad de busqueda: el
// hueco puede quedar justo detras del punto donde se cumplen las 3 vueltas.
// Medido en pista: ~40 s por vuelta al 90% de motor, o sea ~52 s al 70%.
constexpr unsigned long PARKING_TOTAL_MAX_MS = 70000;

// Distancia lateral con la que se confirma que el costado quedo libre: es
// el hueco entre las dos paredes.
constexpr uint16_t PARKING_GAP_SIDE_MM = 300;

// Respaldo por si la camara nunca llega a ver las paredes. En una vuelta
// ANTIHORARIA el bloque central queda a la izquierda y la pared exterior
// a la derecha, asi que el hueco esta a la derecha.
//
// PARKING_CCW_YAW_SIGN es el signo del giro acumulado del BNO085 que
// corresponde a una vuelta antihoraria. Cambialo a -1 si el robot elige
// el lado contrario cuando la camara no vio nada.
constexpr bool PARKING_SIDE_FROM_TURN_FALLBACK = true;
constexpr int8_t PARKING_CCW_YAW_SIGN = 1;

// ---------------------- BNO085, vueltas y dashboard ----------------

constexpr uint8_t BNO085_I2C_ADDRESS_PRIMARY = 0x4B;
constexpr uint8_t BNO085_I2C_ADDRESS_SECONDARY = 0x4A;
constexpr uint32_t BNO085_REPORT_INTERVAL_US = 10000;
constexpr unsigned long BNO085_FAILSAFE_TIMEOUT_MS = 500;

// El BNO085 solo se intentaba iniciar una vez, en setup(). Si no respondia en
// ese instante, o fallaba una sola vez a media ronda, quedaba muerto para el
// resto de la ronda: sin rumbo no se cuentan vueltas, y sin vueltas nunca
// arranca el estacionamiento.
//
// Parado se reintenta seguido, que no cuesta nada. Ya rodando se reintenta
// espaciado, porque begin_I2C bloquea unos cientos de milisegundos y eso se
// nota en el control del pasillo.
constexpr unsigned long BNO085_RETRY_STOPPED_MS = 1000;
constexpr unsigned long BNO085_RETRY_RUNNING_MS = 5000;

// Detectado pero mudo: se fuerza un reinicio en vez de esperar sin fin.
constexpr unsigned long BNO085_STALL_TIMEOUT_MS = 2000;
constexpr bool REQUIRE_BNO085_TO_MOVE = false;
constexpr uint8_t TARGET_LAPS = 3;
constexpr float DEGREES_PER_LAP = 360.0f;
constexpr float MAX_HEADING_STEP_DEGREES = 90.0f;



// true en pruebas; cambiar a false para apagar completamente el WiFi en ronda.
constexpr bool DASHBOARD_ENABLED = true;
constexpr char DASHBOARD_WIFI_NAME[] = "WRO-Obstaculos-V2";
constexpr char DASHBOARD_WIFI_PASSWORD[] = "wroobst26";

// ====================================================================
//                    FIN DE VARIABLES DE CALIBRACION
// ====================================================================

// -------------------------- Comunicaciones --------------------------

constexpr uint8_t OPENMV_RX_PIN = 25;
constexpr uint8_t OPENMV_TX_PIN = 26;
constexpr uint32_t OPENMV_BAUD = 19200;
constexpr unsigned long OPENMV_TIMEOUT_MS = 500;

// Devuelve a la OpenMV el estado que realmente esta aplicando el ESP32 para
// que pueda dibujar una vista previa. No cambia el control del robot.
constexpr bool OPENMV_PATH_TELEMETRY_ENABLED = false;
constexpr unsigned long OPENMV_PATH_TELEMETRY_INTERVAL_MS = 50;

// Aviso a la camara de cuando buscar el estacionamiento: "P,1" lo enciende y
// "P,0" lo apaga. Se repite como latido para que la camara quede en el estado
// correcto aunque se reinicie a media ronda o se pierda una linea.
constexpr unsigned long OPENMV_PARKING_MODE_INTERVAL_MS = 250;

// true: la camara busca las paredes desde el arranque, toda la ronda.
// false: solo las busca al completar las vueltas.
//
// Buscar siempre cuesta una busqueda de blobs mas por cuadro, pero no cambia
// el comportamiento del robot: el ESP32 ignora los campos de estacionamiento
// hasta que el mismo decide que toca aparcar.
constexpr bool OPENMV_PARKING_ALWAYS_SEARCH = true;

// ------------------ Robustez de la trama OpenMV ---------------------

// Campos aceptados por trama. 6 = firmware anterior (sin pared negra),
// 7 = firmware 5, 10 = firmware 5 con estacionamiento. Los campos extra
// se ignoran sin invalidar la linea, para que una actualizacion de la
// camara no deje ciego al ESP32.
constexpr int OPENMV_MIN_FIELDS = 6;
constexpr int OPENMV_MAX_FIELDS = 10;

// Debe coincidir con X_AS_NORMALIZED del script de la OpenMV.
// true:  la camara ya envia X entre -100 y 100.
// false: la camara envia X en pixeles y el ESP32 la convierte aqui.
constexpr bool OPENMV_X_IS_NORMALIZED = true;
constexpr long OPENMV_IMAGE_WIDTH_PX = 320;
constexpr long OPENMV_IMAGE_HEIGHT_PX = 240;

// Margen tolerado antes de recortar X. Un desvio mayor no es redondeo:
// indica trama corrupta o modo de X equivocado, y se rechaza.
constexpr long OPENMV_X_TOLERANCE = 5;

// Tope de lineas atendidas por vuelta de loop(): una rafaga acumulada
// no debe retrasar los sensores ni el servo.
constexpr uint8_t OPENMV_MAX_LINES_PER_LOOP = 8;

// Si el buffer de recepcion supera este tamano, el loop se retraso y las
// tramas guardadas ya no describen lo que la camara ve ahora. Dirigir
// con una imagen vieja es peor que perder cuadros: se descartan.
constexpr int OPENMV_MAX_BACKLOG_BYTES = 128;

// Agrupa los avisos de trama invalida. Imprimir cada una bloquea el loop
// cuando se llena el buffer de Serial.
constexpr unsigned long OPENMV_INVALID_LOG_INTERVAL_MS = 1000;

// El buffer por defecto (256 bytes) se llena si el loop tarda de mas y
// entonces se pierden tramas por la mitad.
constexpr size_t OPENMV_RX_HARDWARE_BUFFER_BYTES = 512;

HardwareSerial openMVSerial(2);

constexpr uint8_t NANO_I2C_ADDRESS = 0x08;
constexpr uint8_t SENSOR_COUNT = 5;
constexpr uint8_t DISTANCE_PACKET_BYTES = SENSOR_COUNT * 2;
constexpr uint16_t INVALID_DISTANCE_MM = 0xFFFF;
constexpr uint16_t NO_ECHO_DISTANCE_MM = 2000;

constexpr uint8_t ESP32_SDA_PIN = 21;
constexpr uint8_t ESP32_SCL_PIN = 22;
// Bajado de 100 kHz a 50 kHz por margen de ruido.
//
// El Nano es de 5 V y su umbral de HIGH es 0.6 x Vcc = 3.00 V, pero el bus lo
// levantan pull-ups a 3.3 V: solo 0.30 V de margen. Con los picos de corriente
// de los NeoPixels y del motor, ese margen se cruza y el Nano lee mal un bit,
// que es como se traba el bus aunque cada cosa tenga su regulador (el rebote
// es de MASA, no del rail).
//
// A menos velocidad los flancos tienen mas tiempo de asentarse. El paquete son
// 10 bytes cada 20 ms, asi que sobra ancho de banda.
constexpr uint32_t I2C_FREQUENCY_HZ = 50000;

// Un bus trabado no debe bloquear el loop en cada intento.
constexpr uint16_t I2C_TIMEOUT_MS = 10;

constexpr unsigned long SENSOR_POLL_INTERVAL_MS = 20;
constexpr unsigned long SENSOR_FAILSAFE_TIMEOUT_MS = 250;

// Si el Nano se calla, reiniciar el bus I2C cada tanto. Un Nano que se
// reinicia por caida de tension vuelve solo, pero un bus trabado (SDA
// enganchado a bajo) no se suelta sin volver a inicializar el maestro.
//
// Mismo criterio que el reintento del BNO085: rendirse en silencio es lo
// unico que no se puede permitir en una ronda.
constexpr unsigned long I2C_RECOVERY_INTERVAL_MS = 2000;
constexpr unsigned long SERIAL_PRINT_INTERVAL_MS = 500;

// Evita que el primer paquete despues de una pausa produzca un paso grande.
constexpr unsigned long MAX_BLEND_STEP_INTERVAL_MS = 50;

constexpr uint32_t SERVO_PWM_FREQUENCY_HZ = 50;
constexpr uint8_t SERVO_PWM_RESOLUTION_BITS = 16;
constexpr uint32_t MOTOR_PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t MOTOR_PWM_RESOLUTION_BITS = 10;

#if ESP_ARDUINO_VERSION_MAJOR < 3
constexpr uint8_t SERVO_PWM_CHANNEL = 0;
constexpr uint8_t MOTOR_PWM_CHANNEL = 1;
#endif

static_assert(CORRIDOR_SERVO_DIRECTION == 1 ||
                  CORRIDOR_SERVO_DIRECTION == -1,
              "CORRIDOR_SERVO_DIRECTION debe ser 1 o -1");
static_assert(OPENMV_SERVO_DIRECTION == 1 ||
                  OPENMV_SERVO_DIRECTION == -1,
              "OPENMV_SERVO_DIRECTION debe ser 1 o -1");
static_assert(FRONT_CORNER_START_MM > FRONT_CORNER_CRITICAL_MM,
              "La distancia inicial debe ser mayor que la critica");
static_assert(MOTOR_SPEED_PERCENT >= MOTOR_CORNER_SPEED_PERCENT,
              "La velocidad normal no puede ser menor que la de esquina");
static_assert(MOTOR_SPEED_PERCENT >= MOTOR_PILLAR_FAR_SPEED_PERCENT &&
                  MOTOR_PILLAR_FAR_SPEED_PERCENT >=
                      MOTOR_CORNER_SPEED_PERCENT,
              "La velocidad de pilar debe quedar entre normal y minima");
static_assert(CORRIDOR_RETURN_RATE_DEGREES_PER_SECOND > 0.0f,
              "La velocidad de retorno debe ser positiva");
static_assert(LED_FRONT_FAR_MM > LED_FRONT_NEAR_MM,
              "La distancia lejana de luz debe superar la cercana");
static_assert(PILLAR_SIDE_CLEAR_MM > PILLAR_SIDE_DETECT_MM,
              "La distancia de despeje debe superar la de deteccion");
static_assert(PILLAR_SIDE_CONFIRM_SAMPLES > 0,
              "La deteccion lateral necesita muestras de confirmacion");
static_assert(PILLAR_WRAP_CORRECTION_DEGREES > 0 &&
                  PILLAR_WRAP_CORRECTION_DEGREES <=
                      SERVO_MAX_CORRECTION_DEGREES,
              "El giro de rodeo debe caber en el recorrido del servo");
static_assert(PILLAR_WRAP_SPEED_PERCENT > 0 &&
                  PILLAR_WRAP_SPEED_PERCENT <= 100,
              "La velocidad de rodeo debe quedar entre 1 y 100");
static_assert(PILLAR_WRAP_DIRECTION == 1 ||
                  PILLAR_WRAP_DIRECTION == -1,
              "PILLAR_WRAP_DIRECTION debe ser 1 o -1");
static_assert(PILLAR_LOCK_RELEASE_TIMEOUT_MS >
                  PILLAR_SENSOR_HANDOFF_TIMEOUT_MS,
              "El desbloqueo debe ocurrir despues del handoff de seguridad");
static_assert(FRONT_GUARD_RELEASE_MM > FRONT_GUARD_ACTIVATE_MM,
              "La salida de proteccion debe superar la entrada");
static_assert(FRONT_GUARD_MAX_STEER_MM < FRONT_GUARD_ACTIVATE_MM,
              "El refuerzo maximo debe quedar mas cerca que la activacion");
static_assert(FRONT_GUARD_MAX_EXTRA_STEERING_DEGREES >=
                  FRONT_GUARD_EXTRA_STEERING_DEGREES,
              "El refuerzo maximo no puede ser menor que el minimo");
// Con el refuerzo al maximo, el rodeo debe seguir cabiendo en el servo: si se
// pasara, los ultimos grados se recortarian en silencio y el valor de la
// constante dejaria de significar lo que dice.
static_assert(PILLAR_WRAP_CORRECTION_DEGREES +
                  FRONT_GUARD_MAX_EXTRA_STEERING_DEGREES <=
                      SERVO_MAX_CORRECTION_DEGREES,
              "El rodeo con refuerzo maximo no cabe en el recorrido del servo");
static_assert(SIDE_WALL_FREE_MM > SIDE_WALL_BLOCK_MM,
              "La distancia libre lateral debe superar la de bloqueo");
static_assert(PILLAR_WRAP_MAX_MS > 0,
              "El rodeo necesita un tope de tiempo");
static_assert(FRONT_GUARD_WRAP_MAX_MS >= PILLAR_WRAP_MAX_MS,
              "Con pared de frente el rodeo no puede durar menos");
static_assert(PILLAR_SWITCH_CONFIRM_FRAMES > 0,
              "El cambio de pilar necesita tramas de confirmacion");
static_assert(PILLAR_SWITCH_WRAP_STALE_MS >= PILLAR_SWITCH_STALE_MS,
              "Durante el rodeo la espera no puede ser menor");
static_assert(CORRIDOR_RETURN_ABORT_FRONT_MM > FRONT_CORNER_CRITICAL_MM,
              "La entrega inmediata debe ocurrir antes de la distancia critica");
static_assert(CORRIDOR_RETURN_MAX_MS > 0,
              "La entrega gradual necesita un tope de tiempo");
static_assert(SIDE_WALL_SPEED_PERCENT > 0 &&
                  SIDE_WALL_SPEED_PERCENT <= 100,
              "La velocidad junto a la pared debe quedar entre 1 y 100");
static_assert(FRONT_GUARD_CONFIRM_SAMPLES > 0,
              "La proteccion frontal necesita muestras de confirmacion");
static_assert(FRONT_GUARD_LOCK_RELEASE_TIMEOUT_MS >
                  FRONT_GUARD_HANDOFF_TIMEOUT_MS,
              "El desbloqueo frontal debe ocurrir despues del handoff");
static_assert(WALL_ESCAPE_CLEAR_MM > WALL_ESCAPE_TRIGGER_MM,
              "La salida del escape debe superar la entrada");
static_assert(WALL_ESCAPE_REVERSE_SPEED_PERCENT > 0 &&
                  WALL_ESCAPE_REVERSE_SPEED_PERCENT <= 100,
              "La reversa de pared debe quedar entre 1 y 100");
static_assert(WALL_ESCAPE_BRAKE_MS > 0 &&
                  WALL_ESCAPE_MAX_REVERSE_MS > 0,
              "Los tiempos del escape de pared deben ser positivos");
static_assert(COLLISION_REVERSE_SPEED_PERCENT > 0 &&
                  COLLISION_REVERSE_SPEED_PERCENT <= 100,
              "La velocidad de reversa debe quedar entre 1 y 100");
static_assert(COLLISION_TRIGGER_CONFIRM_FRAMES > 0,
              "Se necesita al menos una trama para confirmar la colision");
static_assert(COLLISION_CLEAR_CONFIRM_FRAMES > 0,
              "Se necesita al menos una trama para despejar la colision");
static_assert(COLLISION_REVERSE_DISTANCE_MM > 0 &&
                  COLLISION_REVERSE_MAX_MS > 0,
              "La reversa de colision necesita distancia y timeout");
static_assert(MOTOR_START_DETECTION_PERCENT > 0 &&
                  MOTOR_START_DETECTION_PERCENT <= 100,
              "El PWM inicial debe quedar entre 1 y 100");
static_assert(ENCODER_START_PULSES > 0,
              "El encoder necesita al menos un pulso de confirmacion");
static_assert(ENCODER_MM_PER_PULSE > 0.0f,
              "La calibracion del encoder debe ser positiva");
static_assert(TARGET_LAPS > 0,
              "La mision necesita al menos una vuelta");
static_assert(PARKING_WALLS_TO_PASS > 0,
              "Hay que rebasar al menos una pared");
static_assert(PARKING_LED_BRIGHTNESS_PERCENT <= 100,
              "La luz de estacionamiento va entre 0 y 100");
static_assert(PARKING_CCW_YAW_SIGN == 1 || PARKING_CCW_YAW_SIGN == -1,
              "PARKING_CCW_YAW_SIGN debe ser 1 o -1");
static_assert(PARKING_SEEN_FRAMES > 0 && PARKING_LOST_FRAMES > 0,
              "El conteo de paredes necesita tramas de confirmacion");
static_assert(PARKING_REVERSE_SPEED_PERCENT > 0 &&
                  PARKING_REVERSE_SPEED_PERCENT <= 100,
              "La reversa de estacionamiento va entre 1 y 100");
static_assert(PARKING_SEARCH_SPEED_PERCENT >= PARKING_APPROACH_SPEED_PERCENT,
              "Buscar no puede ser mas lento que acercarse");
static_assert(PARKING_REVERSE_STEER_DEGREES <=
                  SERVO_MAX_CORRECTION_DEGREES &&
              PARKING_STRAIGHTEN_STEER_DEGREES <=
                  SERVO_MAX_CORRECTION_DEGREES,
              "El giro de estacionamiento debe caber en el servo");
static_assert(PARKING_TOTAL_MAX_MS > PARKING_PHASE_MAX_MS,
              "El tope total debe superar al de una fase");

// ----------------------------- Estados ------------------------------

struct OpenMVData {
  int16_t detectedId;
  int16_t xReference;
  int16_t yReference;
  int16_t areaPixels;
  int16_t roiCode;
  int16_t collisionId;
  int16_t wallBlack;
  // Pared del estacionamiento (magenta y ancha). La camara la separa del
  // pilar por su forma: 20x10 cm da ancho/alto = 2, el pilar 5x10 cm da
  // alto/ancho = 2.
  int16_t parkingDetected;
  int16_t parkingX;
  int16_t parkingArea;
  unsigned long receivedAtMs;
};

struct CorridorControlState {
  float leftValue;
  float rightValue;
  float error;
  float effectiveKp;
  float pTerm;
  float iTerm;
  float dTerm;
  float cornerFactor;
  int targetServoAngle;
  int appliedServoAngle;
  uint8_t speedPercent;
};

OpenMVData openMVData = {0, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0};

constexpr size_t RX_BUFFER_SIZE = 64;
char rxBuffer[RX_BUFFER_SIZE];
size_t rxLength = 0;
bool discardingLongLine = false;
unsigned long invalidFrameCount = 0;
unsigned long lastInvalidFrameLogMs = 0;
// Acumulado desde el arranque; se muestra en el tablero para ver de un
// vistazo si el enlace con la camara esta sano.
unsigned long totalInvalidFrames = 0;

bool servoReady = false;
bool motorReady = false;
bool communicationTimedOut = false;
bool pillarHasSteeringControl = false;
bool corridorReturnActive = false;
unsigned long corridorReturnStartMs = 0;

bool pillarLockActive = false;
int16_t lockedPillarId = 0;
OpenMVData lockedPillarData = {0, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0};
bool lockedPillarReachedLowRoi = false;
bool lockedPillarSeenBySideSensors = false;
// Momento en que arranco la fase de rodeo, para poder acotarla.
unsigned long pillarWrapStartMs = 0;
// Tramas seguidas viendo un pilar de otro color con el enclavado perdido.
uint8_t pillarSwitchFrames = 0;
uint8_t pillarSideDetectSamples = 0;
uint8_t pillarSideClearSamples = 0;
unsigned long lastLockedPillarSeenMs = 0;
unsigned long pillarSideClearSinceMs = 0;

// El hueco queda a la izquierda o a la derecha del robot. -1 izquierda,
// +1 derecha, 0 aun sin saber.
enum class ParkingState : uint8_t {
  IDLE,          // Todavia dando vueltas.
  SEARCH,        // Vueltas hechas, buscando las paredes.
  PASSING,       // Contando paredes rebasadas.
  ALIGNING,      // Avance final por encoder hasta quedar junto al hueco.
  BRAKING,       // Alto antes de invertir el sentido.
  REVERSE_TURN,  // Entra de reversa girando hacia el hueco.
  REVERSE_STRAIGHT,  // Contravolantea para enderezarse.
  DONE
};

ParkingState parkingState = ParkingState::IDLE;
unsigned long parkingStateStartMs = 0;
unsigned long parkingStartedMs = 0;
uint32_t parkingPhaseStartPulses = 0;
// -1 izquierda, +1 derecha, 0 aun sin determinar.
int8_t parkingSide = 0;
// Distingue el final bueno del que se rindio: el tablero decia
// "estacionado" tambien cuando la maniobra fracasaba.
bool parkingSucceeded = false;
const char *parkingFinishReason = "";
uint8_t parkingWallsPassed = 0;
uint8_t parkingSeenFrames = 0;
uint8_t parkingLostFrames = 0;
bool parkingWallInSight = false;

bool frontGuardActive = false;
uint8_t frontGuardNearSamples = 0;
// Lo pone limitSteeringBySideWall() en cada calculo del angulo de pilar.
bool sideWallGuardActive = false;
uint16_t sideWallGuardDistanceMm = INVALID_DISTANCE_MM;
uint8_t frontGuardClearSamples = 0;

enum class CollisionRecoveryState : uint8_t {
  IDLE,
  STOPPING,
  REVERSING,
  WAITING_FOR_CLEAR
};

enum class WallEscapeState : uint8_t {
  IDLE,
  BRAKING,
  REVERSING,
  WAITING_FOR_CLEAR
};

enum class MotorStartupState : uint8_t {
  WAITING_FOR_MOVEMENT,
  RAMPING,
  RUNNING
};

CollisionRecoveryState collisionRecoveryState =
    CollisionRecoveryState::IDLE;
unsigned long collisionRecoveryStateStartMs = 0;
uint32_t collisionReverseStartPulses = 0;
uint8_t collisionTriggerFrames = 0;
uint8_t collisionClearFrames = 0;
int16_t collisionRecoveryColorId = 0;

WallEscapeState wallEscapeState = WallEscapeState::IDLE;
unsigned long wallEscapeStateStartMs = 0;

float currentServoAngleDegrees = SERVO_CENTER_DEGREES;
int lastWrittenServoAngle = -1;
unsigned long lastCorridorBlendStepMs = 0;

uint8_t targetMotorSpeedPercent = 0;
uint8_t appliedMotorSpeedPercent = 0;

volatile uint32_t encoderPulseCount = 0;
MotorStartupState motorStartupState =
    MotorStartupState::WAITING_FOR_MOVEMENT;
uint32_t encoderDetectionBaseline = 0;
unsigned long encoderDetectionWindowStartMs = 0;
unsigned long motorRampStartMs = 0;
bool encoderDetectionWindowActive = false;

int8_t odometryMotorDirection = ODOMETRY_DIRECTION_STOPPED;
uint32_t odometryLastPulseCount = 0;
bool odometryHasOrigin = false;
float odometryHeadingOriginDegrees = 0.0f;
float odometryXmm = 0.0f;
float odometryYmm = 0.0f;
float odometrySignedDistanceMm = 0.0f;
float odometryTravelDistanceMm = 0.0f;

Adafruit_BNO08x bno085(-1);
sh2_SensorValue_t bno085SensorValue;
bool bno085Initialized = false;
bool bno085HasHeading = false;
bool bno085FailsafeActive = false;
uint8_t bno085I2cAddressDetected = 0;
float currentYawDegrees = 0.0f;
float previousYawDegrees = 0.0f;
float accumulatedTurnDegrees = 0.0f;
uint8_t completedLaps = 0;
unsigned long lastBno085FrameMs = 0;
unsigned long lastBno085RetryMs = 0;
bool lapTrackingActive = false;


bool missionComplete = false;

bool roundTimerStarted = false;
unsigned long roundStartMs = 0;
unsigned long roundFinishMs = 0;

bool distanceSensorFailsafeActive = false;
uint16_t latestDistancesMm[SENSOR_COUNT] = {
    NO_ECHO_DISTANCE_MM,
    NO_ECHO_DISTANCE_MM,
    NO_ECHO_DISTANCE_MM,
    NO_ECHO_DISTANCE_MM,
    NO_ECHO_DISTANCE_MM
};

WebServer dashboardServer(80);

float pidIntegralError = 0.0f;
float pidPreviousError = 0.0f;
float pidFilteredDerivative = 0.0f;
unsigned long pidPreviousUpdateUs = 0;
bool pidHasPreviousSample = false;

int currentLedBrightnessPercent = -1;
int targetLedBrightnessPercent = LED_BRIGHTNESS_FAR_PERCENT;
unsigned long lastLedReturnStepMs = 0;
bool ledI2cErrorReported = false;

uint16_t latestFrontDistanceMm = INVALID_DISTANCE_MM;
unsigned long latestFrontDistanceAtMs = 0;
// Ultimo paquete I2C valido del Nano. Global para poder mostrar su edad
// en el tablero: sin cable no hay forma de ver esto en una ronda.
unsigned long lastValidSensorMs = 0;
unsigned long lastI2cRecoveryMs = 0;

// ------------------ Encoder, BNO085, vueltas y tiempo --------------

void IRAM_ATTR handleEncoderAPulse() {
  __atomic_fetch_add(&encoderPulseCount, 1U, __ATOMIC_RELAXED);
}

uint32_t readEncoderPulseCount() {
  return __atomic_load_n(&encoderPulseCount, __ATOMIC_RELAXED);
}

void beginEncoder() {
  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);
  encoderDetectionBaseline = readEncoderPulseCount();
  attachInterrupt(
      digitalPinToInterrupt(ENCODER_A_PIN),
      handleEncoderAPulse,
      RISING
  );
}

void resetMotorStartupDetection() {
  motorStartupState = MotorStartupState::WAITING_FOR_MOVEMENT;
  encoderDetectionBaseline = readEncoderPulseCount();
  encoderDetectionWindowStartMs = 0;
  motorRampStartMs = 0;
  encoderDetectionWindowActive = false;
}

unsigned long roundElapsedMs() {
  if (!roundTimerStarted) {
    return 0;
  }

  const unsigned long endMs =
      missionComplete ? roundFinishMs : millis();
  return endMs - roundStartMs;
}

float normalizeHeadingDelta(float deltaDegrees) {
  while (deltaDegrees > 180.0f) deltaDegrees -= 360.0f;
  while (deltaDegrees < -180.0f) deltaDegrees += 360.0f;
  return deltaDegrees;
}



void finishMission() {
  if (missionComplete) {
    return;
  }

  // Con el estacionamiento activo la ronda NO termina al completar las
  // vueltas: el robot sigue rodando para buscar el hueco. La ronda la
  // cierra finishParking() al quedar dentro del cajon.
  if (PARKING_ENABLED && parkingState == ParkingState::IDLE) {
    completedLaps = TARGET_LAPS;
    beginParking(millis());
    return;
  }

  if (parkingActive()) {
    return;
  }

  missionComplete = true;
  completedLaps = TARGET_LAPS;
  if (roundTimerStarted) {
    roundFinishMs = millis();
  }
  stopMotor();
  commandServoAngle(SERVO_CENTER_DEGREES);
  Serial.println("*** 3 VUELTAS: MISION DE OBSTACULOS TERMINADA ***");
}

void processBno085Heading(float yawDegrees, unsigned long nowMs) {
  currentYawDegrees = yawDegrees;
  lastBno085FrameMs = nowMs;
  bno085FailsafeActive = false;

  if (!bno085HasHeading) {
    bno085HasHeading = true;
    previousYawDegrees = yawDegrees;
    Serial.println("BNO085: rumbo I2C recibido");
    return;
  }

  const float delta = normalizeHeadingDelta(
      yawDegrees - previousYawDegrees
  );
  previousYawDegrees = yawDegrees;

  if (missionComplete || !roundTimerStarted) {
    return;
  }

  if (!lapTrackingActive) {
    lapTrackingActive = true;
    accumulatedTurnDegrees = 0.0f;
    completedLaps = 0;
    Serial.println("Conteo de vueltas de obstaculos iniciado");
    return;
  }

  // isfinite() atrapa el NaN: sin este filtro un solo cuadro malo dejaria el
  // giro acumulado en NaN para siempre y con el el tramo y toda la ayuda.
  if (!isfinite(delta) || fabsf(delta) > MAX_HEADING_STEP_DEGREES) {
    Serial.println("BNO085: salto de rumbo descartado");
    return;
  }

  accumulatedTurnDegrees += delta;

  const uint8_t newLapCount = static_cast<uint8_t>(
      floorf(fabsf(accumulatedTurnDegrees) / DEGREES_PER_LAP)
  );

  if (newLapCount > completedLaps) {
    completedLaps = min(newLapCount, TARGET_LAPS);
    Serial.print("Vuelta de obstaculos: ");
    Serial.print(completedLaps);
    Serial.print("/");
    Serial.println(TARGET_LAPS);
  }

  if (fabsf(accumulatedTurnDegrees) >=
      DEGREES_PER_LAP * TARGET_LAPS) {
    finishMission();
  }
}

void updateBno085(unsigned long nowMs) {
  if (!bno085Initialized) {
    const unsigned long retryIntervalMs = roundTimerStarted
        ? BNO085_RETRY_RUNNING_MS
        : BNO085_RETRY_STOPPED_MS;

    if (nowMs - lastBno085RetryMs >= retryIntervalMs) {
      lastBno085RetryMs = nowMs;
      Serial.println("BNO085: reintentando iniciar por I2C");
      beginBno085I2c();
    }
    return;
  }

  // Responde en I2C pero no manda cuaterniones: reiniciarlo es la unica
  // salida, porque el conteo de vueltas depende por completo de el.
  if (lastBno085FrameMs != 0 &&
      nowMs - lastBno085FrameMs >= BNO085_STALL_TIMEOUT_MS) {
    Serial.println("BNO085: sin datos, se reinicia el sensor");
    bno085Initialized = false;
    bno085HasHeading = false;
    lastBno085RetryMs = nowMs;
    return;
  }

  if (bno085.wasReset()) {
    bno085HasHeading = false;
    if (!bno085.enableReport(
            SH2_GAME_ROTATION_VECTOR,
            BNO085_REPORT_INTERVAL_US
        )) {
      bno085Initialized = false;
      Serial.println("ERROR: reporte BNO085 no restaurado");
      return;
    }
  }

  while (bno085.getSensorEvent(&bno085SensorValue)) {
    if (bno085SensorValue.sensorId != SH2_GAME_ROTATION_VECTOR) {
      continue;
    }

    const float qr = bno085SensorValue.un.gameRotationVector.real;
    const float qi = bno085SensorValue.un.gameRotationVector.i;
    const float qj = bno085SensorValue.un.gameRotationVector.j;
    const float qk = bno085SensorValue.un.gameRotationVector.k;
    const float yawDegrees = atan2f(
        2.0f * (qi * qj + qk * qr),
        qi * qi - qj * qj - qk * qk + qr * qr
    ) * RAD_TO_DEG;
    processBno085Heading(yawDegrees, nowMs);
  }
}

void updateOdometry() {
  const uint32_t currentPulses = readEncoderPulseCount();

  if (!bno085HasHeading) {
    odometryLastPulseCount = currentPulses;
    return;
  }

  if (!odometryHasOrigin) {
    odometryHasOrigin = true;
    odometryHeadingOriginDegrees = currentYawDegrees;
    odometryLastPulseCount = currentPulses;
    return;
  }

  const uint32_t deltaPulses = currentPulses - odometryLastPulseCount;
  odometryLastPulseCount = currentPulses;

  if (deltaPulses == 0 ||
      odometryMotorDirection == ODOMETRY_DIRECTION_STOPPED) {
    return;
  }

  const float distanceMm =
      static_cast<float>(deltaPulses) *
      ENCODER_MM_PER_PULSE *
      static_cast<float>(odometryMotorDirection);
  const float relativeHeadingDegrees = normalizeHeadingDelta(
      currentYawDegrees - odometryHeadingOriginDegrees
  );
  const float headingRadians = relativeHeadingDegrees * DEG_TO_RAD;

  odometryXmm += distanceMm * cosf(headingRadians);
  odometryYmm += distanceMm * sinf(headingRadians);
  odometrySignedDistanceMm += distanceMm;
  odometryTravelDistanceMm += fabsf(distanceMm);
}

bool tryBeginBno085AtAddress(uint8_t address) {
  if (!bno085.begin_I2C(address, &Wire)) {
    return false;
  }
  if (!bno085.enableReport(
          SH2_GAME_ROTATION_VECTOR,
          BNO085_REPORT_INTERVAL_US
      )) {
    return false;
  }

  bno085Initialized = true;
  bno085I2cAddressDetected = address;
  lastBno085FrameMs = millis();
  Serial.print("BNO085 I2C listo en 0x");
  Serial.println(address, HEX);
  return true;
}

void beginBno085I2c() {
  if (tryBeginBno085AtAddress(BNO085_I2C_ADDRESS_PRIMARY) ||
      tryBeginBno085AtAddress(BNO085_I2C_ADDRESS_SECONDARY)) {
    return;
  }
  Serial.println("AVISO: BNO085 no detectado en 0x4B ni 0x4A");
}

// --------------------------- Dashboard WiFi ------------------------

const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!doctype html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>WRO Obstaculos V2</title><style>
:root{color-scheme:dark;--bg:#071018;--card:#10202b;--line:#29404e;
--text:#f1f7fa;--muted:#93a8b5;--cyan:#35d6ff;--green:#45e68a;
--orange:#ffb454;--red:#ff6474}*{box-sizing:border-box}body{margin:0;
background:radial-gradient(circle at top,#16364b,var(--bg) 48%);color:var(--text);
font:15px system-ui,sans-serif}.wrap{width:min(1050px,94vw);margin:auto;padding:24px 0}
header{display:flex;justify-content:space-between;align-items:center;gap:12px}
h1{margin:2px 0 18px;font-size:clamp(25px,5vw,38px)}.tag{color:var(--cyan);
font-size:12px;font-weight:800;letter-spacing:.12em}.status{padding:9px 13px;
border:1px solid var(--line);border-radius:999px}.ok{color:var(--green)}
.warn{color:var(--orange)}.stop{color:var(--red)}.grid{display:grid;
grid-template-columns:repeat(4,1fr);gap:12px}.card{background:linear-gradient(145deg,
#142733,#0c1821);border:1px solid var(--line);border-radius:16px;padding:16px}
.wide{grid-column:span 2}.full{grid-column:1/-1}.label{color:var(--muted);
font-size:12px;text-transform:uppercase;letter-spacing:.08em}.value{font-size:
clamp(25px,5vw,40px);font-weight:800;margin-top:4px}.unit{font-size:14px;
color:var(--muted)}.cyan{color:var(--cyan)}.sensors,.details{display:grid;
grid-template-columns:repeat(5,1fr);gap:8px;margin-top:12px}.box{background:#09141c;
border-radius:10px;padding:11px;text-align:center}.box b{display:block;margin-top:4px;
font-size:18px}#openmv .details{grid-template-columns:repeat(3,1fr)}
footer{text-align:center;
color:var(--muted);font-size:12px;margin-top:18px}
@media(max-width:700px){.grid{grid-template-columns:repeat(2,1fr)}.wide{grid-column:
span 2}.details{grid-template-columns:repeat(2,1fr)}header{align-items:flex-start;
flex-direction:column}.status{margin-bottom:12px}}
</style></head><body><main class="wrap"><header><div><div class="tag">WRO INDIA 2026</div>
<h1>Obst&aacute;culos V2</h1></div><div id="status" class="status">Conectando...</div></header>
<section class="grid"><article class="card"><div class="label">Vueltas</div>
<div class="value cyan"><span id="laps">0</span><span class="unit"> / 3</span></div></article>
<article class="card"><div class="label">Tiempo de ronda</div><div id="time" class="value">0:00.0</div></article>
<article class="card"><div class="label">Rumbo</div><div class="value"><span id="yaw">--</span><span class="unit">&deg;</span></div></article>
<article class="card"><div class="label">Motor</div><div class="value"><span id="speed">0</span><span class="unit">%</span></div></article>
<article class="card full"><div class="label">Posici&oacute;n estimada desde la salida</div>
<div class="details"><div class="box">X<b><span id="odoX">0.0</span> cm</b></div>
<div class="box">Y<b><span id="odoY">0.0</span> cm</b></div>
<div class="box">Recorrido<b><span id="odoTravel">0.0</span> cm</b></div>
<div class="box">Odometr&iacute;a<b id="odoState">--</b></div></div></article>
<article class="card full"><div class="label">Ultras&oacute;nicos</div><div id="sensors" class="sensors"></div></article>
<article class="card wide" id="openmv"><div class="label">OpenMV</div><div class="details">
<div class="box">Color<b id="color">--</b></div><div class="box">ROI<b id="roi">--</b></div>
<div class="box">X<b id="x">--</b></div><div class="box">Y<b id="y">--</b></div>
<div class="box">Pared<b id="wall">--</b></div><div class="box">Enlace<b id="link">--</b></div>
<div class="box">Estac.<b id="park">--</b></div>
<div class="box">Maniobra<b id="parkPhase">--</b></div></div></article>
<article class="card wide"><div class="label">Diagn&oacute;stico</div><div class="details">
<div class="box">Manda<b id="ctrl">--</b></div>
<div class="box">Sensores<b id="sens">--</b></div>
<div class="box">Motor<b><span id="mtr">--</span></b></div></div></article>
<article class="card wide"><div class="label">Control</div><div class="details">
<div class="box">Servo<b><span id="servo">--</span>&deg;</b></div>
<div class="box">Objetivo<b><span id="target">--</span>%</b></div>
<div class="box">Encoder<b id="encoder">--</b></div><div class="box">BNO I2C<b id="bno">--</b></div></div></article>
</section><footer>Telemetr&iacute;a local de pruebas &middot; 192.168.4.1</footer></main>
<script>const put=(id,v)=>document.getElementById(id).textContent=v;
const ss=document.getElementById('sensors');ss.innerHTML=[1,2,3,4,5].map(n=>
`<div class="box">S${n}<b id="s${n}">--</b></div>`).join('');
function clock(ms){const s=ms/1000,m=Math.floor(s/60);return m+':'+String(
Math.floor(s%60)).padStart(2,'0')+'.'+Math.floor(ms%1000/100)}
async function refresh(){try{const d=await(await fetch('/api/status',{cache:'no-store'})).json();
put('laps',d.laps);put('time',clock(d.elapsedMs));put('yaw',d.yaw.toFixed(1));
put('speed',d.speed);put('target',d.target);put('servo',d.servo);put('encoder',d.encoder);
put('odoX',(d.odoX/10).toFixed(1));put('odoY',(d.odoY/10).toFixed(1));
put('odoTravel',(d.odoTravel/10).toFixed(1));put('odoState',d.odoReady?'ACTIVA':'SIN BNO');
put('bno',d.bno?('0x'+d.bno.toString(16).toUpperCase()):'--');put('color',d.color);
put('roi',d.roi);put('x',d.x);put('y',d.y);
put('wall',d.wall?'NEGRA':'--');
put('link',(d.camAgeMs>500?'SIN DATOS':'OK')+' / '+d.camBad+' desc.');
put('park',d.park?('X '+d.parkX):'--');
put('parkPhase',d.parkPhase+' ('+d.parkWalls+')'+(d.parkWhy?' - '+d.parkWhy:''));
put('ctrl',d.ctrl);
put('sens',d.sensAge>400?('MUDO '+d.sensAge+'ms'):('OK '+d.sensAge+'ms'));
put('mtr',d.speed+'% de '+d.target+'%');d.distances.forEach((v,i)=>put('s'+(i+1),(v/10).toFixed(1)+' cm'));
const st=document.getElementById('status');st.textContent=d.state;st.className='status '+d.level;
}catch(e){const st=document.getElementById('status');st.textContent='Sin conexion';st.className='status stop'}}
refresh();setInterval(refresh,250);</script></body></html>
)rawliteral";

// Que rama del loop tiene el mando ahora mismo, en el mismo orden de
// prioridad que usa loop(). Es distinto del estado general: durante el
// estacionamiento el robot sigue rodando con el PID de pasillo, y eso el
// estado no lo dejaba ver.
const char *controlBranchText() {
  if (wallEscapeActive()) return "ESCAPE_PARED";
  if (collisionRecoveryActive()) return "COLISION";
  if (parkingActive() && !parkingIsMovingForward()) return "MANIOBRA";
  if (pillarHasSteeringControl) return "PILAR";
  return "PASILLO";
}

const char *dashboardStateText() {
  if (missionComplete) return "3 vueltas completadas";
  if (distanceSensorFailsafeActive) return "Sin sensores I2C";
  if (parkingActive()) return parkingStateText();
  if (sideWallGuardActive) return "Limitado por pared lateral";
  if (wallEscapeActive()) return "Escape de pared";
  if (collisionRecoveryActive()) return "Recuperacion de colision";
  if (pillarHasSteeringControl && lockedPillarSeenBySideSensors) {
    return "Rodeando pilar enclavado";
  }
  if (pillarHasSteeringControl) return "Control por pilar";
  if (corridorReturnActive) return "Retorno gradual al pasillo";
  if (!bno085Initialized) return "Pasillo; BNO085 no detectado";
  if (!bno085HasHeading) return "SIN GIROSCOPIO: no se cuentan vueltas";
  return "Control de pasillo";
}

const char *dashboardStateLevel() {
  if (missionComplete || distanceSensorFailsafeActive) return "stop";
  if (wallEscapeActive() || collisionRecoveryActive() ||
      !bno085HasHeading ||
      communicationTimedOut) return "warn";
  return "ok";
}

const char *dashboardColorText() {
  if (openMVData.detectedId == 3) return "Verde";
  if (openMVData.detectedId == 5) return "Rojo";
  return "Ninguno";
}

void sendDashboardStatus() {
  char json[1600];
  unsigned long pillarHoldRemainingMs = 0;
  if (pillarLockActive && pillarSideClearSinceMs != 0) {
    const unsigned long heldMs = millis() - pillarSideClearSinceMs;
    if (heldMs < PILLAR_POST_CLEAR_HOLD_MS) {
      pillarHoldRemainingMs = PILLAR_POST_CLEAR_HOLD_MS - heldMs;
    }
  }
  snprintf(
      json,
      sizeof(json),
      "{\"laps\":%u,\"elapsedMs\":%lu,\"yaw\":%.2f,\"speed\":%u,"
      "\"target\":%u,\"servo\":%d,\"encoder\":%lu,\"bno\":%u,"
      "\"odoX\":%.1f,\"odoY\":%.1f,\"odoSigned\":%.1f,"
      "\"odoTravel\":%.1f,\"odoReady\":%u,"
      "\"color\":\"%s\",\"detectedId\":%d,\"roi\":%d,\"x\":%d,\"y\":%d,"
      "\"wall\":%d,\"camAgeMs\":%lu,\"camBad\":%lu,"
      "\"sideGuard\":%u,\"sideMm\":%u,"
      "\"park\":%d,\"parkX\":%d,\"parkArea\":%d,"
      "\"parkPhase\":\"%s\",\"parkWalls\":%u,\"parkSide\":%d,"
      "\"parkWhy\":\"%s\","
      "\"ctrl\":\"%s\",\"sensAge\":%lu,"
      "\"pillarLock\":%u,\"pillarControl\":%u,\"returning\":%u,"
      "\"lockedId\":%d,\"sideCleared\":%u,\"holdMs\":%lu,"
      "\"distances\":[%u,%u,%u,%u,%u],\"state\":\"%s\",\"level\":\"%s\"}",
      completedLaps,
      roundElapsedMs(),
      currentYawDegrees,
      appliedMotorSpeedPercent,
      targetMotorSpeedPercent,
      lastWrittenServoAngle,
      static_cast<unsigned long>(readEncoderPulseCount()),
      bno085I2cAddressDetected,
      odometryXmm,
      odometryYmm,
      odometrySignedDistanceMm,
      odometryTravelDistanceMm,
      odometryHasOrigin && bno085HasHeading ? 1U : 0U,
      dashboardColorText(),
      openMVData.detectedId,
      openMVData.roiCode,
      openMVData.xReference,
      openMVData.yReference,
      openMVData.wallBlack,
      openMVData.receivedAtMs == 0
          ? 999999UL
          : millis() - openMVData.receivedAtMs,
      totalInvalidFrames,
      sideWallGuardActive ? 1U : 0U,
      sideWallGuardDistanceMm,
      openMVData.parkingDetected,
      openMVData.parkingX,
      openMVData.parkingArea,
      parkingStateText(),
      parkingWallsPassed,
      parkingSide,
      parkingFinishReason,
      controlBranchText(),
      lastValidSensorMs == 0
          ? 999999UL
          : millis() - lastValidSensorMs,
      pillarLockActive ? 1U : 0U,
      pillarHasSteeringControl ? 1U : 0U,
      corridorReturnActive ? 1U : 0U,
      lockedPillarId,
      lockedPillarSeenBySideSensors ? 1U : 0U,
      pillarHoldRemainingMs,
      latestDistancesMm[0], latestDistancesMm[1], latestDistancesMm[2],
      latestDistancesMm[3], latestDistancesMm[4],
      dashboardStateText(), dashboardStateLevel()
  );
  dashboardServer.send(200, "application/json", json);
}

void beginDashboard() {
  if (!DASHBOARD_ENABLED) {
    WiFi.mode(WIFI_OFF);
    return;
  }
  WiFi.mode(WIFI_AP);
  WiFi.softAP(DASHBOARD_WIFI_NAME, DASHBOARD_WIFI_PASSWORD);
  dashboardServer.on("/", []() {
    dashboardServer.send_P(200, "text/html; charset=utf-8", DASHBOARD_HTML);
  });
  dashboardServer.on("/api/status", sendDashboardStatus);
  dashboardServer.begin();
  Serial.print("Dashboard: ");
  Serial.print(DASHBOARD_WIFI_NAME);
  Serial.print(" -> http://");
  Serial.println(WiFi.softAPIP());
}

void updateDashboard() {
  if (DASHBOARD_ENABLED) dashboardServer.handleClient();
}

// -------------------------- Utilidades UART -------------------------

bool isDetectedIdValid(long value) {
  return value == 0 || value == 3 || value == 5;
}

bool isRoiCodeValid(long value) {
  return value >= 0 && value <= 2;
}

bool isCollisionIdValid(long value) {
  return value == 0 || value == 3 || value == 5;
}

long clampLong(long value, long minimum, long maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

// Deja la X en el rango -100..100 que espera todo el control, venga
// normalizada de la camara o en pixeles.
long normalizeOpenMVx(long xReference) {
  if (OPENMV_X_IS_NORMALIZED) {
    return xReference;
  }

  const long pixels =
      clampLong(xReference, 0, OPENMV_IMAGE_WIDTH_PX - 1);

  return (pixels * 200L + (OPENMV_IMAGE_WIDTH_PX - 1) / 2L) /
             (OPENMV_IMAGE_WIDTH_PX - 1L) -
         100L;
}

// Divide la linea en enteros separados por comas. Tolera espacios y una coma
// final. Devuelve cuantos campos venian en la linea, o -1 si aparece algo que
// no pertenece a una trama numerica: texto de arranque de la camara, ruido de
// la linea o media trama tras un reinicio.
int splitOpenMVFields(const char *line, long *fields, int maxFields) {
  int count = 0;
  const char *cursor = line;

  while (*cursor != '\0') {
    while (*cursor == ' ' || *cursor == '\t' ||
           *cursor == '\r' || *cursor == '\n') {
      ++cursor;
    }
    if (*cursor == '\0') {
      break;
    }

    bool negative = false;
    if (*cursor == '+' || *cursor == '-') {
      negative = (*cursor == '-');
      ++cursor;
    }

    if (*cursor < '0' || *cursor > '9') {
      return -1;
    }

    long value = 0;
    uint8_t digits = 0;
    while (*cursor >= '0' && *cursor <= '9') {
      if (digits < 9) {
        // Nueve digitos caben siempre en un long de 32 bits.
        value = value * 10L + static_cast<long>(*cursor - '0');
      }
      ++digits;
      ++cursor;
    }

    if (digits > 9) {
      return -1;
    }

    if (negative) {
      value = -value;
    }

    if (count < maxFields) {
      fields[count] = value;
    }
    ++count;

    while (*cursor == ' ' || *cursor == '\t' ||
           *cursor == '\r' || *cursor == '\n') {
      ++cursor;
    }

    if (*cursor == ',') {
      ++cursor;
      continue;
    }
    if (*cursor == '\0') {
      break;
    }

    return -1;
  }

  return count;
}

// Corrige los campos en lugar de tirar la trama entera. Solo se rechaza lo
// estructuralmente imposible; un valor de mas o de menos no debe apagar el
// control por camara durante toda la vuelta.
bool sanitizeOpenMVFields(long *fields, int fieldCount) {
  if (!isDetectedIdValid(fields[0]) ||
      !isRoiCodeValid(fields[4]) ||
      !isCollisionIdValid(fields[5])) {
    return false;
  }

  if (fieldCount < 7) {
    // Firmware anterior: no manda el estado de la pared negra.
    fields[6] = 0;
  }
  fields[6] = (fields[6] != 0) ? 1 : 0;

  if (fieldCount < 10) {
    // Firmware sin estacionamiento.
    fields[7] = 0;
    fields[8] = 0;
    fields[9] = 0;
  }

  fields[7] = (fields[7] != 0) ? 1 : 0;

  // Una X de pared fuera de rango es campo corrupto, no redondeo: se descarta
  // solo el estacionamiento y se conserva el resto de la trama, que puede
  // llevar el pilar con el que se esta dirigiendo ahora mismo.
  if (fields[7] != 0 &&
      (fields[8] < OPENMV_X_MIN - OPENMV_X_TOLERANCE ||
       fields[8] > OPENMV_X_MAX + OPENMV_X_TOLERANCE ||
       fields[9] <= 0)) {
    fields[7] = 0;
  }

  if (fields[7] == 0) {
    fields[8] = 0;
    fields[9] = 0;
  } else {
    fields[8] = clampLong(fields[8], OPENMV_X_MIN, OPENMV_X_MAX);
    fields[9] = clampLong(fields[9], 0, INT16_MAX);
  }

  if (fields[0] == 0) {
    // Sin deteccion se normaliza a la trama vacia canonica.
    fields[1] = -1;
    fields[2] = -1;
    fields[3] = 0;
    fields[4] = 0;
    fields[5] = 0;
    return true;
  }

  // Con pilar, la ROI y el area son obligatorias: sin ellas no hay objetivo
  // al que apuntar el servo.
  if (fields[4] != 1 && fields[4] != 2) {
    return false;
  }
  if (fields[3] <= 0) {
    return false;
  }

  fields[1] = normalizeOpenMVx(fields[1]);

  // Fuera de la tolerancia no es redondeo: suele ser trama corrupta o
  // OPENMV_X_IS_NORMALIZED mal configurado respecto al script de la camara.
  if (fields[1] < OPENMV_X_MIN - OPENMV_X_TOLERANCE ||
      fields[1] > OPENMV_X_MAX + OPENMV_X_TOLERANCE) {
    return false;
  }

  fields[1] = clampLong(fields[1], OPENMV_X_MIN, OPENMV_X_MAX);
  fields[2] = clampLong(fields[2], 0, OPENMV_IMAGE_HEIGHT_PX - 1);
  fields[3] = clampLong(fields[3], 0, INT16_MAX);

  // OpenMV solo declara colision para el mismo pilar dentro de ROI_LOW. Una
  // combinacion imposible se ignora bajando la colision a cero: se conserva la
  // direccion de la trama y no se dispara una maniobra falsa.
  if (fields[5] != 0 &&
      (fields[5] != fields[0] || fields[4] != 2)) {
    fields[5] = 0;
  }

  return true;
}

bool parseOpenMVLine(const char *line, OpenMVData &result) {
  long fields[OPENMV_MAX_FIELDS];
  for (int i = 0; i < OPENMV_MAX_FIELDS; ++i) {
    fields[i] = 0;
  }

  const int fieldsRead =
      splitOpenMVFields(line, fields, OPENMV_MAX_FIELDS);

  // Se aceptan 6 campos o mas. Los que sobran quedan fuera del arreglo y se
  // descartan sin invalidar la linea.
  if (fieldsRead < OPENMV_MIN_FIELDS) {
    return false;
  }

  const int usableFields =
      (fieldsRead < OPENMV_MAX_FIELDS) ? fieldsRead : OPENMV_MAX_FIELDS;

  if (!sanitizeOpenMVFields(fields, usableFields)) {
    return false;
  }

  result.detectedId = static_cast<int16_t>(fields[0]);
  result.xReference = static_cast<int16_t>(fields[1]);
  result.yReference = static_cast<int16_t>(fields[2]);
  result.areaPixels = static_cast<int16_t>(fields[3]);
  result.roiCode = static_cast<int16_t>(fields[4]);
  result.collisionId = static_cast<int16_t>(fields[5]);
  result.wallBlack = static_cast<int16_t>(fields[6]);
  result.parkingDetected = static_cast<int16_t>(fields[7]);
  result.parkingX = static_cast<int16_t>(fields[8]);
  result.parkingArea = static_cast<int16_t>(fields[9]);
  result.receivedAtMs = millis();
  return true;
}

bool hasValidPillarTarget(const OpenMVData &data) {
  return (data.detectedId == 3 || data.detectedId == 5) &&
         data.xReference >= OPENMV_X_MIN &&
         data.xReference <= OPENMV_X_MAX &&
         (data.roiCode == 1 || data.roiCode == 2);
}

// ------------------------- Control del servo ------------------------

bool beginServoPwm() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcAttach(
      SERVO_PIN,
      SERVO_PWM_FREQUENCY_HZ,
      SERVO_PWM_RESOLUTION_BITS
  );
#else
  ledcSetup(
      SERVO_PWM_CHANNEL,
      SERVO_PWM_FREQUENCY_HZ,
      SERVO_PWM_RESOLUTION_BITS
  );
  ledcAttachPin(SERVO_PIN, SERVO_PWM_CHANNEL);
  return true;
#endif
}

void writeServoHardware(int angleDegrees) {
  if (!servoReady) {
    return;
  }

  angleDegrees = constrain(angleDegrees, 0, 180);

  const uint32_t pulseUs =
      SERVO_MIN_PULSE_US +
      (static_cast<uint32_t>(angleDegrees) *
       (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) /
          180UL;

  constexpr uint32_t PWM_PERIOD_US =
      1000000UL / SERVO_PWM_FREQUENCY_HZ;
  constexpr uint32_t PWM_MAX_DUTY =
      (1UL << SERVO_PWM_RESOLUTION_BITS) - 1UL;

  const uint32_t duty =
      (pulseUs * PWM_MAX_DUTY + PWM_PERIOD_US / 2UL) /
      PWM_PERIOD_US;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(SERVO_PIN, duty);
#else
  ledcWrite(SERVO_PWM_CHANNEL, duty);
#endif
}

int commandServoAngle(float angleDegrees) {
  angleDegrees = constrain(
      angleDegrees,
      static_cast<float>(SERVO_CENTER_DEGREES -
                         SERVO_MAX_CORRECTION_DEGREES),
      static_cast<float>(SERVO_CENTER_DEGREES +
                         SERVO_MAX_CORRECTION_DEGREES)
  );

  currentServoAngleDegrees = angleDegrees;
  const int roundedAngle = static_cast<int>(roundf(angleDegrees));

  if (roundedAngle != lastWrittenServoAngle) {
    writeServoHardware(roundedAngle);
    lastWrittenServoAngle = roundedAngle;
  }

  return roundedAngle;
}

void resetCorridorPid() {
  pidIntegralError = 0.0f;
  pidPreviousError = 0.0f;
  pidFilteredDerivative = 0.0f;
  pidPreviousUpdateUs = 0;
  pidHasPreviousSample = false;
}

void takePillarSteeringControl() {
  if (!pillarHasSteeringControl) {
    resetCorridorPid();
  }

  pillarHasSteeringControl = true;
  corridorReturnActive = false;
}

void releasePillarSteeringControl() {
  if (!pillarHasSteeringControl) {
    return;
  }

  pillarHasSteeringControl = false;
  corridorReturnActive = true;
  corridorReturnStartMs = millis();
  lastCorridorBlendStepMs = corridorReturnStartMs;
  resetCorridorPid();
}

void resetFrontGuard() {
  frontGuardActive = false;
  frontGuardNearSamples = 0;
  frontGuardClearSamples = 0;
}

void updateFrontGuard(uint16_t frontDistanceMm) {
  if (!pillarLockActive) {
    resetFrontGuard();
    return;
  }

  if (frontDistanceMm == INVALID_DISTANCE_MM) {
    return;
  }

  if (!frontGuardActive) {
    frontGuardNearSamples = frontDistanceMm <= FRONT_GUARD_ACTIVATE_MM
        ? static_cast<uint8_t>(frontGuardNearSamples + 1)
        : 0;

    if (frontGuardNearSamples >= FRONT_GUARD_CONFIRM_SAMPLES) {
      frontGuardActive = true;
      frontGuardNearSamples = 0;
      frontGuardClearSamples = 0;
      Serial.println(
          "PROTECCION FRONTAL: S3 <= 40 cm, refuerzo de evasion"
      );
    }
    return;
  }

  frontGuardClearSamples = frontDistanceMm >= FRONT_GUARD_RELEASE_MM
      ? static_cast<uint8_t>(frontGuardClearSamples + 1)
      : 0;

  if (frontGuardClearSamples >= FRONT_GUARD_CONFIRM_SAMPLES) {
    resetFrontGuard();
    Serial.println("PROTECCION FRONTAL: S3 libre, control normal");
  }
}

void beginPillarLock(const OpenMVData &data) {
  pillarLockActive = true;
  lockedPillarId = data.detectedId;
  lockedPillarData = data;
  lockedPillarReachedLowRoi = data.roiCode == 2;
  lockedPillarSeenBySideSensors = false;
  pillarWrapStartMs = 0;
  pillarSwitchFrames = 0;
  pillarSideDetectSamples = 0;
  pillarSideClearSamples = 0;
  pillarSideClearSinceMs = 0;
  lastLockedPillarSeenMs = data.receivedAtMs;
  resetFrontGuard();
  takePillarSteeringControl();

  Serial.print("Pilar enclavado: ");
  Serial.println(lockedPillarId == 3 ? "VERDE" : "ROJO");
}

void clearPillarLock(const char *reason) {
  if (!pillarLockActive) {
    return;
  }

  releasePillarSteeringControl();
  pillarLockActive = false;
  lockedPillarId = 0;
  lockedPillarData = {0, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0};
  lockedPillarReachedLowRoi = false;
  lockedPillarSeenBySideSensors = false;
  pillarWrapStartMs = 0;
  pillarSwitchFrames = 0;
  pillarSideDetectSamples = 0;
  pillarSideClearSamples = 0;
  pillarSideClearSinceMs = 0;
  lastLockedPillarSeenMs = 0;
  resetFrontGuard();

  Serial.print("Pilar liberado: ");
  Serial.println(reason);
}

bool validDistanceAtMost(uint16_t distanceMm, uint16_t limitMm) {
  return distanceMm != INVALID_DISTANCE_MM && distanceMm <= limitMm;
}

bool distanceIsClear(uint16_t distanceMm, uint16_t limitMm) {
  return distanceMm == INVALID_DISTANCE_MM || distanceMm >= limitMm;
}

void getLockedPillarSideDistances(
    const uint16_t distancesMm[SENSOR_COUNT],
    uint16_t &firstDistanceMm,
    uint16_t &secondDistanceMm
) {
  if (lockedPillarId == 3) {
    // Pilar verde a la derecha del robot.
    firstDistanceMm = distancesMm[3];  // S4
    secondDistanceMm = distancesMm[4]; // S5
  } else {
    // Pilar rojo a la izquierda del robot.
    firstDistanceMm = distancesMm[0];  // S1
    secondDistanceMm = distancesMm[1]; // S2
  }
}

void updatePillarDistanceHandoff(
    const uint16_t distancesMm[SENSOR_COUNT],
    unsigned long nowMs
) {
  if (!pillarLockActive) {
    return;
  }

  const unsigned long timeSinceLastVisionMs =
      nowMs - lastLockedPillarSeenMs;
  const unsigned long handoffTimeoutMs = frontGuardActive
      ? FRONT_GUARD_HANDOFF_TIMEOUT_MS
      : PILLAR_SENSOR_HANDOFF_TIMEOUT_MS;
  const unsigned long lockReleaseTimeoutMs = frontGuardActive
      ? FRONT_GUARD_LOCK_RELEASE_TIMEOUT_MS
      : PILLAR_LOCK_RELEASE_TIMEOUT_MS;

  if (!lockedPillarReachedLowRoi) {
    if (timeSinceLastVisionMs >= PILLAR_HIGH_LOSS_TIMEOUT_MS) {
      clearPillarLock("perdido antes de ROI_LOW");
    }
    return;
  }

  uint16_t firstDistanceMm;
  uint16_t secondDistanceMm;
  getLockedPillarSideDistances(
      distancesMm,
      firstDistanceMm,
      secondDistanceMm
  );

  const bool pillarIsNear =
      validDistanceAtMost(firstDistanceMm, PILLAR_SIDE_DETECT_MM) ||
      validDistanceAtMost(secondDistanceMm, PILLAR_SIDE_DETECT_MM);

  const bool pillarSideIsClear =
      distanceIsClear(firstDistanceMm, PILLAR_SIDE_CLEAR_MM) &&
      distanceIsClear(secondDistanceMm, PILLAR_SIDE_CLEAR_MM);

  if (!lockedPillarSeenBySideSensors) {
    pillarSideDetectSamples = pillarIsNear
        ? static_cast<uint8_t>(pillarSideDetectSamples + 1)
        : 0;

    if (pillarSideDetectSamples >= PILLAR_SIDE_CONFIRM_SAMPLES) {
      lockedPillarSeenBySideSensors = true;
      pillarWrapStartMs = nowMs;
      pillarSideDetectSamples = 0;
      pillarSideClearSamples = 0;
      pillarSideClearSinceMs = 0;
      Serial.println(
          "Pilar confirmado por ultrasonicos: inicia rodeo lateral"
      );
      return;
    }

    if (pillarHasSteeringControl &&
        timeSinceLastVisionMs >= handoffTimeoutMs) {
      releasePillarSteeringControl();
      Serial.println(
          "Handoff de seguridad: ultrasonicos sin confirmar el pilar"
      );
    }

    if (timeSinceLastVisionMs >= lockReleaseTimeoutMs) {
      clearPillarLock("timeout de sensores laterales");
    }
    return;
  }

  if (pillarSideIsClear) {
    if (pillarSideClearSinceMs == 0) {
      pillarSideClearSinceMs = nowMs;
    }
    if (pillarSideClearSamples < UINT8_MAX) {
      ++pillarSideClearSamples;
    }
  } else {
    pillarSideClearSamples = 0;
    pillarSideClearSinceMs = 0;
  }

  if (pillarSideClearSamples >= PILLAR_SIDE_CONFIRM_SAMPLES &&
      pillarSideClearSinceMs != 0 &&
      nowMs - pillarSideClearSinceMs >= PILLAR_POST_CLEAR_HOLD_MS) {
    clearPillarLock("ultrasonicos confirman costado libre");
    return;
  }

  // Tope del rodeo. Si el costado no se despeja es que ahi hay una pared,
  // no el pilar: seguir girando solo lleva a chocar contra ella.
  //
  // Con la pared de frente se concede mas tiempo, porque ahi el pilar esta
  // pegado a esa pared y la vuelta que hay que darle es mas larga.
  // Aqui el rodeo ya arranco, asi que el sello siempre es valido.
  const unsigned long wrapMaxMs = frontGuardActive
      ? FRONT_GUARD_WRAP_MAX_MS
      : PILLAR_WRAP_MAX_MS;

  if (nowMs - pillarWrapStartMs >= wrapMaxMs) {
    clearPillarLock(
        frontGuardActive
            ? "tope de rodeo largo: ni con pared de frente se despejo"
            : "tope de rodeo: el costado no se despeja"
    );
  }
}

int mapVisionErrorToCorrection(int visionError, int gainPercent) {
  visionError = constrain(visionError, OPENMV_X_MIN, OPENMV_X_MAX);

  const long baseCorrectionDegrees = map(
      static_cast<long>(visionError),
      OPENMV_X_MIN,
      OPENMV_X_MAX,
      -SERVO_MAX_CORRECTION_DEGREES,
      SERVO_MAX_CORRECTION_DEGREES
  );

  const long amplifiedCorrection = baseCorrectionDegrees * gainPercent;

  return constrain(
      static_cast<int>(
          amplifiedCorrection >= 0
              ? (amplifiedCorrection + 50) / 100
              : (amplifiedCorrection - 50) / 100
      ),
      -SERVO_MAX_CORRECTION_DEGREES,
      SERVO_MAX_CORRECTION_DEGREES
  );
}

int lowRoiConeHalfWidth(int yReference) {
  const int constrainedY = constrain(
      yReference,
      LOW_ROI_CONE_FAR_Y,
      LOW_ROI_CONE_NEAR_Y
  );

  return static_cast<int>(
      map(
          constrainedY,
          LOW_ROI_CONE_FAR_Y,
          LOW_ROI_CONE_NEAR_Y,
          LOW_ROI_CONE_FAR_HALF_WIDTH,
          LOW_ROI_CONE_NEAR_HALF_WIDTH
      )
  );
}

int lowRoiTargetX(const OpenMVData &data) {
  const int halfWidth = lowRoiConeHalfWidth(data.yReference);
  return data.detectedId == 3 ? halfWidth : -halfWidth;
}

int calculateLowRoiError(const OpenMVData &data) {
  return constrain(
      data.xReference - lowRoiTargetX(data),
      OPENMV_X_MIN,
      OPENMV_X_MAX
  );
}

int calculatePillarServoAngle(const OpenMVData &data) {
  int visionError = 0;
  int gainPercent = 100;

  if (data.roiCode == 1) {
    visionError = data.xReference;
    gainPercent = HIGH_ROI_STEERING_GAIN_PERCENT;
  } else if (data.roiCode == 2) {
    visionError = calculateLowRoiError(data);
    gainPercent = LOW_ROI_EVASION_GAIN_PERCENT;
  }

  const int correctionDegrees =
      mapVisionErrorToCorrection(visionError, gainPercent);

  return SERVO_CENTER_DEGREES +
         OPENMV_SERVO_DIRECTION * correctionDegrees;
}

int calculatePillarServoWithFrontGuard(const OpenMVData &data) {
  int angleDegrees = calculatePillarServoAngle(data);

  if (!frontGuardActive || !lockedPillarReachedLowRoi) {
    return limitSteeringBySideWall(angleDegrees);
  }

  // Verde se evade hacia el lado negativo del servo y rojo hacia el positivo.
  const int colorDirection = data.detectedId == 3 ? -1 : 1;
  angleDegrees +=
      OPENMV_SERVO_DIRECTION *
      colorDirection *
      frontGuardExtraSteeringDegrees();

  angleDegrees = constrain(
      angleDegrees,
      SERVO_CENTER_DEGREES - SERVO_MAX_CORRECTION_DEGREES,
      SERVO_CENTER_DEGREES + SERVO_MAX_CORRECTION_DEGREES
  );

  return limitSteeringBySideWall(angleDegrees);
}

// Distancia mas cercana del costado pedido. S1/S2 miran a la izquierda y
// S4/S5 a la derecha; se toma la menor de las dos lecturas validas.
uint16_t nearestSideDistanceMm(
    bool leftSide,
    const uint16_t distancesMm[SENSOR_COUNT]
) {
  const uint16_t first = leftSide ? distancesMm[0] : distancesMm[3];
  const uint16_t second = leftSide ? distancesMm[1] : distancesMm[4];

  if (first == INVALID_DISTANCE_MM) {
    return second;
  }
  if (second == INVALID_DISTANCE_MM) {
    return first;
  }

  return min(first, second);
}

// Recorta el giro que lleva al robot contra la pared de ese costado. El
// recorte es proporcional: a SIDE_WALL_FREE_MM no toca nada y a
// SIDE_WALL_BLOCK_MM centra el volante, sin escalones que den tirones.
int limitSteeringBySideWall(int angleDegrees) {
  sideWallGuardActive = false;
  sideWallGuardDistanceMm = INVALID_DISTANCE_MM;

  if (!PILLAR_SIDE_WALL_GUARD_ENABLED) {
    return angleDegrees;
  }

  // Sin lecturas frescas no se limita nada: un dato viejo cortaria el
  // rodeo por una pared que el robot ya dejo atras.
  if (latestFrontDistanceAtMs == 0 ||
      millis() - latestFrontDistanceAtMs > SENSOR_FAILSAFE_TIMEOUT_MS) {
    return angleDegrees;
  }

  const int offsetDegrees = angleDegrees - SERVO_CENTER_DEGREES;
  if (offsetDegrees == 0) {
    return angleDegrees;
  }

  // El signo lo define el PID de pasillo, que ya esta calibrado: con error
  // positivo hay mas espacio a la izquierda y gira hacia alla, asi que ese
  // es el signo de "izquierda". Si recalibras CORRIDOR_SERVO_DIRECTION,
  // este limitador lo sigue solo.
  const bool steeringLeft =
      (offsetDegrees * CORRIDOR_SERVO_DIRECTION) > 0;

  const uint16_t sideMm =
      nearestSideDistanceMm(steeringLeft, latestDistancesMm);

  if (sideMm == INVALID_DISTANCE_MM || sideMm >= SIDE_WALL_FREE_MM) {
    return angleDegrees;
  }

  sideWallGuardActive = true;
  sideWallGuardDistanceMm = sideMm;

  if (sideMm <= SIDE_WALL_BLOCK_MM) {
    return SERVO_CENTER_DEGREES;
  }

  const long spanMm =
      static_cast<long>(SIDE_WALL_FREE_MM) - SIDE_WALL_BLOCK_MM;
  const long allowedOffset =
      (static_cast<long>(offsetDegrees) *
       (static_cast<long>(sideMm) - SIDE_WALL_BLOCK_MM)) /
      spanMm;

  return SERVO_CENTER_DEGREES + static_cast<int>(allowedOffset);
}

// Grados extra de rodeo segun lo cerca que este la pared del frente. Solo
// actua con la proteccion frontal activa, que a su vez exige pilar enclavado.
int frontGuardExtraSteeringDegrees() {
  if (!frontGuardActive) {
    return 0;
  }

  if (latestFrontDistanceAtMs == 0 ||
      millis() - latestFrontDistanceAtMs > SENSOR_FAILSAFE_TIMEOUT_MS ||
      latestFrontDistanceMm == INVALID_DISTANCE_MM) {
    return FRONT_GUARD_EXTRA_STEERING_DEGREES;
  }

  if (latestFrontDistanceMm <= FRONT_GUARD_MAX_STEER_MM) {
    return FRONT_GUARD_MAX_EXTRA_STEERING_DEGREES;
  }

  if (latestFrontDistanceMm >= FRONT_GUARD_ACTIVATE_MM) {
    return FRONT_GUARD_EXTRA_STEERING_DEGREES;
  }

  const long spanMm =
      static_cast<long>(FRONT_GUARD_ACTIVATE_MM) - FRONT_GUARD_MAX_STEER_MM;
  const long closenessMm =
      static_cast<long>(FRONT_GUARD_ACTIVATE_MM) - latestFrontDistanceMm;
  const long spanDegrees =
      FRONT_GUARD_MAX_EXTRA_STEERING_DEGREES -
      FRONT_GUARD_EXTRA_STEERING_DEGREES;

  return FRONT_GUARD_EXTRA_STEERING_DEGREES +
         static_cast<int>((closenessMm * spanDegrees) / spanMm);
}

int calculatePillarWrapServoAngle() {
  // Verde queda a la derecha: rodear por la izquierda. Rojo queda a la
  // izquierda: rodear por la derecha.
  const int colorDirection = lockedPillarId == 3 ? -1 : 1;
  const int correctionDegrees =
      PILLAR_WRAP_CORRECTION_DEGREES +
      frontGuardExtraSteeringDegrees();
  const int angleDegrees = constrain(
      SERVO_CENTER_DEGREES +
          OPENMV_SERVO_DIRECTION *
          PILLAR_WRAP_DIRECTION *
          colorDirection *
          correctionDegrees,
      SERVO_CENTER_DEGREES - SERVO_MAX_CORRECTION_DEGREES,
      SERVO_CENTER_DEGREES + SERVO_MAX_CORRECTION_DEGREES
  );

  return limitSteeringBySideWall(angleDegrees);
}

// Corta la entrega gradual cuando esperar deja de ser seguro: con la pared
// de frente el robot esta entrando a la esquina y necesita el giro completo
// ahora, no dentro de medio segundo.
bool corridorReturnMustFinishNow(unsigned long nowMs) {
  if (nowMs - corridorReturnStartMs >= CORRIDOR_RETURN_MAX_MS) {
    return true;
  }

  if (latestFrontDistanceAtMs == 0 ||
      nowMs - latestFrontDistanceAtMs > SENSOR_FAILSAFE_TIMEOUT_MS) {
    return false;
  }

  return validDistanceAtMost(
      latestFrontDistanceMm,
      CORRIDOR_RETURN_ABORT_FRONT_MM
  );
}

int applyCorridorServoTarget(int targetAngle) {
  if (corridorReturnActive && corridorReturnMustFinishNow(millis())) {
    corridorReturnActive = false;
    Serial.println(
        "PASILLO: entrega inmediata del volante (esquina o tiempo agotado)"
    );
  }

  targetAngle = constrain(
      targetAngle,
      SERVO_CENTER_DEGREES - SERVO_MAX_CORRECTION_DEGREES,
      SERVO_CENTER_DEGREES + SERVO_MAX_CORRECTION_DEGREES
  );

  if (!corridorReturnActive) {
    return commandServoAngle(static_cast<float>(targetAngle));
  }

  const unsigned long nowMs = millis();
  unsigned long elapsedMs = nowMs - lastCorridorBlendStepMs;
  lastCorridorBlendStepMs = nowMs;
  elapsedMs = min(elapsedMs, MAX_BLEND_STEP_INTERVAL_MS);

  const float maxStep =
      CORRIDOR_RETURN_RATE_DEGREES_PER_SECOND *
      static_cast<float>(elapsedMs) /
      1000.0f;
  const float difference =
      static_cast<float>(targetAngle) - currentServoAngleDegrees;

  if (fabsf(difference) <= maxStep) {
    corridorReturnActive = false;
    return commandServoAngle(static_cast<float>(targetAngle));
  }

  if (difference > 0.0f) {
    return commandServoAngle(currentServoAngleDegrees + maxStep);
  }

  return commandServoAngle(currentServoAngleDegrees - maxStep);
}

// ---------------------- Control del motor TB6612 --------------------

bool beginMotorPwm() {
  pinMode(MOTOR_AIN1_PIN, OUTPUT);
  pinMode(MOTOR_AIN2_PIN, OUTPUT);
  digitalWrite(MOTOR_AIN1_PIN, LOW);
  digitalWrite(MOTOR_AIN2_PIN, LOW);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcAttach(
      MOTOR_PWM_PIN,
      MOTOR_PWM_FREQUENCY_HZ,
      MOTOR_PWM_RESOLUTION_BITS
  );
#else
  ledcSetup(
      MOTOR_PWM_CHANNEL,
      MOTOR_PWM_FREQUENCY_HZ,
      MOTOR_PWM_RESOLUTION_BITS
  );
  ledcAttachPin(MOTOR_PWM_PIN, MOTOR_PWM_CHANNEL);
  return true;
#endif
}

void setMotorDirectionForward() {
  odometryMotorDirection = ODOMETRY_DIRECTION_FORWARD;
  digitalWrite(
      MOTOR_AIN1_PIN,
      INVERT_MOTOR_DIRECTION ? LOW : HIGH
  );
  digitalWrite(
      MOTOR_AIN2_PIN,
      INVERT_MOTOR_DIRECTION ? HIGH : LOW
  );
}

void setMotorDirectionReverse() {
  odometryMotorDirection = ODOMETRY_DIRECTION_REVERSE;
  digitalWrite(
      MOTOR_AIN1_PIN,
      INVERT_MOTOR_DIRECTION ? HIGH : LOW
  );
  digitalWrite(
      MOTOR_AIN2_PIN,
      INVERT_MOTOR_DIRECTION ? LOW : HIGH
  );
}

void writeMotorPwmPercent(uint8_t percent) {
  if (!motorReady) {
    return;
  }

  percent = constrain(percent, 0, 100);
  constexpr uint32_t MOTOR_PWM_MAX_DUTY =
      (1UL << MOTOR_PWM_RESOLUTION_BITS) - 1UL;
  const uint32_t duty =
      (static_cast<uint32_t>(percent) * MOTOR_PWM_MAX_DUTY + 50UL) /
      100UL;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PWM_PIN, duty);
#else
  ledcWrite(MOTOR_PWM_CHANNEL, duty);
#endif
  appliedMotorSpeedPercent = percent;
}

void stopMotor() {
  writeMotorPwmPercent(0);
  digitalWrite(MOTOR_AIN1_PIN, LOW);
  digitalWrite(MOTOR_AIN2_PIN, LOW);
  targetMotorSpeedPercent = 0;
  appliedMotorSpeedPercent = 0;
  odometryMotorDirection = ODOMETRY_DIRECTION_STOPPED;
  resetMotorStartupDetection();
  resetCorridorPid();
}

void setMotorSpeedPercent(uint8_t percent) {
  targetMotorSpeedPercent = constrain(percent, 0, 100);

  if (targetMotorSpeedPercent == 0) {
    stopMotor();
    return;
  }

  setMotorDirectionForward();
}

void setMotorReverseSpeedPercent(uint8_t percent) {
  targetMotorSpeedPercent = constrain(percent, 0, 100);

  if (targetMotorSpeedPercent == 0) {
    stopMotor();
    return;
  }

  setMotorDirectionReverse();
  writeMotorPwmPercent(targetMotorSpeedPercent);
}

void serviceMotorSoftStart(unsigned long nowMs) {
  if (!motorReady || targetMotorSpeedPercent == 0 ||
      wallEscapeActive() || collisionRecoveryActive() ||
      missionComplete) {
    return;
  }

  setMotorDirectionForward();

  if (motorStartupState == MotorStartupState::WAITING_FOR_MOVEMENT) {
    // Nunca superar la velocidad solicitada por el control de esquina o
    // pilar. Antes, una orden segura de 30 % podia provocar 45 % durante la
    // deteccion del encoder y durante buena parte de la supuesta rampa suave.
    const uint8_t startupSpeedPercent = min(
        MOTOR_START_DETECTION_PERCENT,
        targetMotorSpeedPercent
    );
    writeMotorPwmPercent(startupSpeedPercent);
    const uint32_t currentPulses = readEncoderPulseCount();
    const uint32_t detectedPulses =
        currentPulses - encoderDetectionBaseline;

    if (detectedPulses > 0 && !encoderDetectionWindowActive) {
      encoderDetectionWindowActive = true;
      encoderDetectionWindowStartMs = nowMs;
    }

    if (encoderDetectionWindowActive &&
        detectedPulses >= ENCODER_START_PULSES) {
      motorStartupState = MotorStartupState::RAMPING;
      motorRampStartMs = nowMs;
      if (!roundTimerStarted) {
        roundTimerStarted = true;
        roundStartMs = nowMs;
        roundFinishMs = 0;
        Serial.println("Cronometro de obstaculos iniciado");
      }

      if (targetMotorSpeedPercent <= MOTOR_START_DETECTION_PERCENT) {
        motorStartupState = MotorStartupState::RUNNING;
        writeMotorPwmPercent(targetMotorSpeedPercent);
        Serial.println(
            "Encoder confirma movimiento: velocidad objetivo aplicada"
        );
      } else {
        Serial.println(
            "Encoder confirma movimiento: inicia rampa suave"
        );
      }
      return;
    }

    if (encoderDetectionWindowActive &&
        nowMs - encoderDetectionWindowStartMs >=
            ENCODER_DETECTION_WINDOW_MS) {
      encoderDetectionBaseline = currentPulses;
      encoderDetectionWindowActive = false;
    }
    return;
  }

  if (motorStartupState == MotorStartupState::RAMPING) {
    const unsigned long elapsedMs = nowMs - motorRampStartMs;
    if (elapsedMs >= MOTOR_SOFT_START_DURATION_MS) {
      motorStartupState = MotorStartupState::RUNNING;
      writeMotorPwmPercent(targetMotorSpeedPercent);
      Serial.println("Rampa suave de obstaculos terminada");
      return;
    }

    const int16_t speedRange =
        static_cast<int16_t>(targetMotorSpeedPercent) -
        static_cast<int16_t>(MOTOR_START_DETECTION_PERCENT);
    const int16_t rampSpeed =
        static_cast<int16_t>(MOTOR_START_DETECTION_PERCENT) +
        static_cast<int32_t>(speedRange) * elapsedMs /
            MOTOR_SOFT_START_DURATION_MS;
    // Si el control reduce su objetivo durante la rampa, obedecerlo de
    // inmediato en vez de mantener temporalmente un PWM mayor.
    writeMotorPwmPercent(
        min(
            static_cast<uint8_t>(rampSpeed),
            targetMotorSpeedPercent
        )
    );
    return;
  }

  writeMotorPwmPercent(targetMotorSpeedPercent);
}

bool collisionRecoveryActive() {
  return collisionRecoveryState != CollisionRecoveryState::IDLE;
}

bool wallEscapeActive() {
  return wallEscapeState != WallEscapeState::IDLE;
}

void startCollisionRecovery(int16_t colorId) {
  clearPillarLock("colision detectada por OpenMV");
  collisionRecoveryState = CollisionRecoveryState::STOPPING;
  collisionRecoveryStateStartMs = millis();
  collisionReverseStartPulses = 0;
  collisionTriggerFrames = 0;
  collisionClearFrames = 0;
  collisionRecoveryColorId = colorId;

  stopMotor();
  corridorReturnActive = false;
  commandServoAngle(static_cast<float>(SERVO_CENTER_DEGREES));

  Serial.print("COLISION ");
  Serial.print(colorId == 3 ? "VERDE" : "ROJA");
  Serial.println(": volante centrado, preparando reversa");
}

void serviceCollisionRecovery(unsigned long nowMs) {
  if (!collisionRecoveryActive()) {
    return;
  }

  // La maniobra completa conserva el volante centrado.
  commandServoAngle(static_cast<float>(SERVO_CENTER_DEGREES));

  if (collisionRecoveryState == CollisionRecoveryState::STOPPING) {
    if (nowMs - collisionRecoveryStateStartMs >=
        COLLISION_STOP_BEFORE_REVERSE_MS) {
      collisionRecoveryState = CollisionRecoveryState::REVERSING;
      collisionRecoveryStateStartMs = nowMs;
      collisionReverseStartPulses = readEncoderPulseCount();
      setMotorReverseSpeedPercent(COLLISION_REVERSE_SPEED_PERCENT);
      Serial.print("COLISION: reversa de ");
      Serial.print(COLLISION_REVERSE_DISTANCE_MM);
      Serial.println(" mm iniciada");
    }
    return;
  }

  if (collisionRecoveryState == CollisionRecoveryState::REVERSING) {
    const uint32_t traveledPulses =
        readEncoderPulseCount() - collisionReverseStartPulses;
    const float traveledMm =
        static_cast<float>(traveledPulses) * ENCODER_MM_PER_PULSE;
    const bool targetDistanceReached =
        traveledMm >= static_cast<float>(COLLISION_REVERSE_DISTANCE_MM);
    const bool reverseTimedOut =
        nowMs - collisionRecoveryStateStartMs >= COLLISION_REVERSE_MAX_MS;

    if (targetDistanceReached || reverseTimedOut) {
      stopMotor();
      collisionRecoveryState = CollisionRecoveryState::WAITING_FOR_CLEAR;
      collisionRecoveryStateStartMs = nowMs;
      collisionClearFrames = 0;
      Serial.print("COLISION: reversa terminada a ");
      Serial.print(traveledMm, 1);
      Serial.print(" mm");
      if (reverseTimedOut && !targetDistanceReached) {
        Serial.print(" (timeout de encoder)");
      }
      Serial.println(", esperando ROI libre");
    }
    return;
  }

  // WAITING_FOR_CLEAR permanece detenido hasta recibir varias tramas libres.
  stopMotor();
}

bool handleCollisionPacket(const OpenMVData &data) {
  // S3 tiene prioridad. No se inicia ni se reanuda una maniobra de OpenMV
  // mientras el robot esta escapando fisicamente de una pared.
  if (wallEscapeActive()) {
    collisionTriggerFrames = 0;
    collisionClearFrames = 0;
    return true;
  }

  const bool collisionDetected =
      data.collisionId == 3 || data.collisionId == 5;

  const bool collisionMatchesLockedPillar =
      collisionDetected &&
      pillarLockActive &&
      lockedPillarReachedLowRoi &&
      data.collisionId == lockedPillarId;

  if (!collisionRecoveryActive()) {
    collisionTriggerFrames = collisionMatchesLockedPillar
        ? static_cast<uint8_t>(collisionTriggerFrames + 1)
        : 0;

    if (collisionTriggerFrames >= COLLISION_TRIGGER_CONFIRM_FRAMES) {
      startCollisionRecovery(data.collisionId);
      return true;
    }
    return false;
  }

  if (collisionRecoveryState != CollisionRecoveryState::WAITING_FOR_CLEAR) {
    return true;
  }

  collisionClearFrames = collisionDetected
      ? 0
      : static_cast<uint8_t>(collisionClearFrames + 1);

  if (collisionClearFrames < COLLISION_CLEAR_CONFIRM_FRAMES) {
    return true;
  }

  collisionRecoveryState = CollisionRecoveryState::IDLE;
  collisionRecoveryStateStartMs = 0;
  collisionReverseStartPulses = 0;
  collisionTriggerFrames = 0;
  collisionClearFrames = 0;
  collisionRecoveryColorId = 0;
  resetCorridorPid();
  Serial.println("COLISION despejada: nuevo intento habilitado");
  return false;
}

bool updateWallEscape(
    uint16_t frontDistanceMm,
    unsigned long nowMs
) {
  const bool validFrontDistance =
      frontDistanceMm != INVALID_DISTANCE_MM;

  if (!wallEscapeActive() && validFrontDistance &&
      frontDistanceMm <= WALL_ESCAPE_TRIGGER_MM) {
    // La pared tiene prioridad sobre el pilar y cualquier recuperacion por
    // color que estuviera en curso.
    clearPillarLock("escape frontal de pared");
    collisionRecoveryState = CollisionRecoveryState::IDLE;
    collisionRecoveryStateStartMs = 0;
    collisionReverseStartPulses = 0;
    collisionTriggerFrames = 0;
    collisionClearFrames = 0;
    collisionRecoveryColorId = 0;

    stopMotor();
    corridorReturnActive = false;
    wallEscapeState = WallEscapeState::BRAKING;
    wallEscapeStateStartMs = nowMs;
    commandServoAngle(static_cast<float>(SERVO_CENTER_DEGREES));

    Serial.print("PARED a ");
    Serial.print(frontDistanceMm / 10);
    Serial.println(" cm: frenando antes de retroceder");
  }

  if (!wallEscapeActive()) {
    return false;
  }

  commandServoAngle(static_cast<float>(SERVO_CENTER_DEGREES));

  const bool frontClear =
      validFrontDistance && frontDistanceMm >= WALL_ESCAPE_CLEAR_MM;
  if (frontClear) {
    stopMotor();
    wallEscapeState = WallEscapeState::IDLE;
    wallEscapeStateStartMs = 0;
    setMotorDirectionForward();
    resetMotorStartupDetection();
    resetCorridorPid();
    Serial.println("PARED liberada: termina el retroceso de escape");
    return false;
  }

  if (wallEscapeState == WallEscapeState::BRAKING) {
    if (nowMs - wallEscapeStateStartMs >= WALL_ESCAPE_BRAKE_MS) {
      wallEscapeState = WallEscapeState::REVERSING;
      wallEscapeStateStartMs = nowMs;
      setMotorReverseSpeedPercent(WALL_ESCAPE_REVERSE_SPEED_PERCENT);
      Serial.print("PARED: retroceso iniciado al ");
      Serial.print(WALL_ESCAPE_REVERSE_SPEED_PERCENT);
      Serial.println("%");
    }
    return true;
  }

  if (wallEscapeState == WallEscapeState::REVERSING) {
    if (nowMs - wallEscapeStateStartMs >=
        WALL_ESCAPE_MAX_REVERSE_MS) {
      stopMotor();
      wallEscapeState = WallEscapeState::WAITING_FOR_CLEAR;
      wallEscapeStateStartMs = nowMs;
      Serial.println(
          "PARED: limite de reversa alcanzado, motor detenido"
      );
    } else {
      setMotorReverseSpeedPercent(WALL_ESCAPE_REVERSE_SPEED_PERCENT);
    }
    return true;
  }

  // Si S3 nunca llega al umbral libre, permanece detenido de forma segura.
  stopMotor();
  return true;
}

// --------------------------- PID pasillo ----------------------------

float lateralDistanceForControl(uint16_t rawDistanceMm) {
  if (rawDistanceMm == INVALID_DISTANCE_MM ||
      rawDistanceMm > MAX_LATERAL_DISTANCE_MM) {
    return static_cast<float>(MAX_LATERAL_DISTANCE_MM);
  }

  return static_cast<float>(rawDistanceMm);
}

float calculateCornerFactor(uint16_t frontDistanceMm) {
  if (frontDistanceMm == INVALID_DISTANCE_MM ||
      frontDistanceMm >= FRONT_CORNER_START_MM) {
    return 0.0f;
  }

  if (frontDistanceMm <= FRONT_CORNER_CRITICAL_MM) {
    return 1.0f;
  }

  return static_cast<float>(FRONT_CORNER_START_MM - frontDistanceMm) /
         static_cast<float>(FRONT_CORNER_START_MM -
                            FRONT_CORNER_CRITICAL_MM);
}

uint8_t calculateMotorSpeed(float cornerFactor) {
  const float reduction =
      cornerFactor *
      static_cast<float>(MOTOR_SPEED_PERCENT -
                         MOTOR_CORNER_SPEED_PERCENT);

  return static_cast<uint8_t>(
      roundf(static_cast<float>(MOTOR_SPEED_PERCENT) - reduction)
  );
}

CorridorControlState calculateCorridorControl(
    const uint16_t distancesMm[SENSOR_COUNT]
) {
  const float s1 = lateralDistanceForControl(distancesMm[0]);
  const float s2 = lateralDistanceForControl(distancesMm[1]);
  const float s4 = lateralDistanceForControl(distancesMm[3]);
  const float s5 = lateralDistanceForControl(distancesMm[4]);

  CorridorControlState state;
  state.leftValue =
      s1 * SENSOR_WEIGHT_S1 + s2 * SENSOR_WEIGHT_S2;
  state.rightValue =
      s4 * SENSOR_WEIGHT_S4 + s5 * SENSOR_WEIGHT_S5;
  state.error = state.leftValue - state.rightValue;
  state.cornerFactor = calculateCornerFactor(distancesMm[2]);
  state.effectiveKp =
      CORRIDOR_KP *
      (1.0f +
       state.cornerFactor * (CORNER_KP_MULTIPLIER - 1.0f));

  const unsigned long nowUs = micros();
  float derivative = 0.0f;

  if (pidHasPreviousSample) {
    const float dtSeconds =
        static_cast<float>(nowUs - pidPreviousUpdateUs) / 1000000.0f;

    if (dtSeconds > 0.0f && dtSeconds <= 0.25f) {
      pidIntegralError += state.error * dtSeconds;

      if (CORRIDOR_KI > 0.0f) {
        const float integralLimitError =
            CORRIDOR_INTEGRAL_LIMIT_DEGREES / CORRIDOR_KI;
        pidIntegralError = constrain(
            pidIntegralError,
            -integralLimitError,
            integralLimitError
        );
      } else {
        pidIntegralError = 0.0f;
      }

      const float rawDerivative =
          (state.error - pidPreviousError) / dtSeconds;
      pidFilteredDerivative +=
          CORRIDOR_DERIVATIVE_FILTER *
          (rawDerivative - pidFilteredDerivative);
      derivative = pidFilteredDerivative;
    }
  } else {
    pidHasPreviousSample = true;
  }

  pidPreviousError = state.error;
  pidPreviousUpdateUs = nowUs;

  state.pTerm = state.effectiveKp * state.error;
  state.iTerm = constrain(
      CORRIDOR_KI * pidIntegralError,
      -CORRIDOR_INTEGRAL_LIMIT_DEGREES,
      CORRIDOR_INTEGRAL_LIMIT_DEGREES
  );
  state.dTerm = CORRIDOR_KD * derivative;

  float correctionDegrees =
      static_cast<float>(CORRIDOR_SERVO_DIRECTION) *
      (state.pTerm + state.iTerm + state.dTerm);
  correctionDegrees += parkingCorridorBiasDegrees();
  correctionDegrees = constrain(
      correctionDegrees,
      -static_cast<float>(SERVO_MAX_CORRECTION_DEGREES),
      static_cast<float>(SERVO_MAX_CORRECTION_DEGREES)
  );

  state.targetServoAngle = static_cast<int>(
      roundf(SERVO_CENTER_DEGREES + correctionDegrees)
  );
  state.appliedServoAngle = applyCorridorServoTarget(
      state.targetServoAngle
  );
  state.speedPercent = calculateMotorSpeed(state.cornerFactor);

  // Durante el estacionamiento se rueda despacio: aqui no se gana tiempo
  // y un error cuesta la maniobra entera.
  if (parkingIsMovingForward()) {
    state.speedPercent =
        min(state.speedPercent, parkingForwardSpeedCap());
  }

  setMotorSpeedPercent(state.speedPercent);

  return state;
}

float calculateCameraPillarProximity(const OpenMVData &data) {
  const int constrainedY = constrain(
      static_cast<int>(data.yReference),
      PILLAR_FAR_Y,
      PILLAR_NEAR_Y
  );

  return static_cast<float>(constrainedY - PILLAR_FAR_Y) /
         static_cast<float>(PILLAR_NEAR_Y - PILLAR_FAR_Y);
}

uint8_t calculatePillarMotorSpeed(
    const OpenMVData &data,
    uint16_t frontDistanceMm
) {
  const float cameraProximity = calculateCameraPillarProximity(data);
  const float frontProximity = calculateCornerFactor(frontDistanceMm);
  const float combinedProximity = max(cameraProximity, frontProximity);

  const float reduction =
      combinedProximity *
      static_cast<float>(MOTOR_PILLAR_FAR_SPEED_PERCENT -
                         MOTOR_CORNER_SPEED_PERCENT);

  return static_cast<uint8_t>(
      roundf(
          static_cast<float>(MOTOR_PILLAR_FAR_SPEED_PERCENT) - reduction
      )
  );
}

void updateMotorWhilePillarControls(
    const OpenMVData &data,
    const uint16_t distancesMm[SENSOR_COUNT]
) {
  uint8_t speedPercent =
      calculatePillarMotorSpeed(data, distancesMm[2]);

  if (frontGuardActive) {
    speedPercent = min(speedPercent, FRONT_GUARD_SPEED_PERCENT);
  }

  // El angulo se calcula antes de fijar la velocidad porque el limitador
  // lateral avisa, al calcularlo, que el robot va contra una pared.
  if (lockedPillarSeenBySideSensors) {
    speedPercent = min(speedPercent, PILLAR_WRAP_SPEED_PERCENT);
    const int wrapAngle = calculatePillarWrapServoAngle();

    if (sideWallGuardActive) {
      speedPercent = min(speedPercent, SIDE_WALL_SPEED_PERCENT);
    }

    commandServoAngle(static_cast<float>(wrapAngle));
    setMotorSpeedPercent(speedPercent);
    return;
  }

  const int pillarAngle = calculatePillarServoWithFrontGuard(data);

  if (sideWallGuardActive) {
    speedPercent = min(speedPercent, SIDE_WALL_SPEED_PERCENT);
  }

  commandServoAngle(static_cast<float>(pillarAngle));
  setMotorSpeedPercent(speedPercent);
}

// ------------------------- Comunicacion I2C -------------------------

bool sendLedBrightnessPercent(uint8_t percent) {
  percent = constrain(percent, 0, 100);

  Wire.beginTransmission(NANO_I2C_ADDRESS);
  Wire.write(percent);
  return Wire.endTransmission() == 0;
}

bool readDistancesMm(uint16_t outputMm[SENSOR_COUNT]) {
  const uint8_t received = Wire.requestFrom(
      static_cast<uint8_t>(NANO_I2C_ADDRESS),
      static_cast<uint8_t>(DISTANCE_PACKET_BYTES)
  );

  if (received != DISTANCE_PACKET_BYTES) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    const uint8_t lowByte = Wire.read();
    const uint8_t highByte = Wire.read();
    const uint16_t decodedDistance =
        static_cast<uint16_t>(lowByte) |
        (static_cast<uint16_t>(highByte) << 8);
    outputMm[i] = decodedDistance == INVALID_DISTANCE_MM
                      ? NO_ECHO_DISTANCE_MM
                      : decodedDistance;
  }

  return true;
}

// ----------------------- Intensidad proporcional --------------------

uint8_t calculateCameraLedBrightnessPercent(const OpenMVData &data) {
  const int constrainedY = constrain(
      static_cast<int>(data.yReference),
      PILLAR_FAR_Y,
      PILLAR_NEAR_Y
  );

  return static_cast<uint8_t>(
      map(
          constrainedY,
          PILLAR_FAR_Y,
          PILLAR_NEAR_Y,
          LED_BRIGHTNESS_FAR_PERCENT,
          LED_BRIGHTNESS_NEAR_PERCENT
      )
  );
}

uint8_t calculateFrontLedBrightnessPercent(uint16_t distanceMm) {
  const uint16_t constrainedDistance = constrain(
      distanceMm,
      LED_FRONT_NEAR_MM,
      LED_FRONT_FAR_MM
  );

  return static_cast<uint8_t>(
      map(
          constrainedDistance,
          LED_FRONT_NEAR_MM,
          LED_FRONT_FAR_MM,
          LED_BRIGHTNESS_NEAR_PERCENT,
          LED_BRIGHTNESS_FAR_PERCENT
      )
  );
}

bool hasFreshFrontDistance(unsigned long nowMs) {
  return latestFrontDistanceAtMs != 0 &&
         nowMs - latestFrontDistanceAtMs <= FRONT_DISTANCE_FRESH_MS &&
         latestFrontDistanceMm != INVALID_DISTANCE_MM;
}

uint8_t calculateCombinedLedBrightnessPercent(
    const OpenMVData &data,
    unsigned long nowMs
) {
  // S3 solo participa si la camara confirma que existe un pilar.
  const bool cameraTargetIsFresh =
      data.receivedAtMs != 0 &&
      nowMs - data.receivedAtMs <= OPENMV_TIMEOUT_MS &&
      !communicationTimedOut;

  if (!cameraTargetIsFresh || !hasValidPillarTarget(data)) {
    return LED_BRIGHTNESS_FAR_PERCENT;
  }

  const uint8_t cameraPercent =
      calculateCameraLedBrightnessPercent(data);

  if (!hasFreshFrontDistance(nowMs)) {
    return cameraPercent;
  }

  const uint8_t frontPercent =
      calculateFrontLedBrightnessPercent(latestFrontDistanceMm);

  // Menor porcentaje significa mayor cercania y mas atenuacion. Se usa
  // la estimacion mas cercana para evitar sobreexponer un pilar proximo.
  return min(cameraPercent, frontPercent);
}

void refreshLedBrightnessTarget() {
  // Estacionando manda la luz fija: el ajuste automatico la bajaria justo
  // cuando el robot tiene las paredes cerca, que es cuando mas falta hace.
  if (parkingActive()) {
    setLedBrightnessTarget(PARKING_LED_BRIGHTNESS_PERCENT);
    return;
  }

  setLedBrightnessTarget(
      calculateCombinedLedBrightnessPercent(openMVData, millis())
  );
}

bool applyLedBrightnessNow(uint8_t percent) {
  if (currentLedBrightnessPercent == percent) {
    return true;
  }

  if (!sendLedBrightnessPercent(percent)) {
    if (!ledI2cErrorReported) {
      ledI2cErrorReported = true;
      Serial.println("ERROR I2C: no se pudo actualizar los LEDs");
    }
    return false;
  }

  ledI2cErrorReported = false;
  currentLedBrightnessPercent = percent;
  return true;
}

void setLedBrightnessTarget(uint8_t percent) {
  targetLedBrightnessPercent = constrain(percent, 0, 100);
}

void serviceLedBrightnessTransition() {
  if (currentLedBrightnessPercent < 0) {
    applyLedBrightnessNow(targetLedBrightnessPercent);
    return;
  }

  if (targetLedBrightnessPercent < currentLedBrightnessPercent) {
    applyLedBrightnessNow(targetLedBrightnessPercent);
    return;
  }

  if (targetLedBrightnessPercent == currentLedBrightnessPercent) {
    return;
  }

  const unsigned long nowMs = millis();
  if (nowMs - lastLedReturnStepMs < LED_RETURN_INTERVAL_MS) {
    return;
  }

  lastLedReturnStepMs = nowMs;
  const int nextPercent = min(
      currentLedBrightnessPercent + LED_RETURN_STEP_PERCENT,
      targetLedBrightnessPercent
  );
  applyLedBrightnessNow(static_cast<uint8_t>(nextPercent));
}

// ------------------- Estacionamiento final ---------------------------

bool parkingActive() {
  return parkingState != ParkingState::IDLE &&
         parkingState != ParkingState::DONE;
}

const char *parkingStateText() {
  switch (parkingState) {
    case ParkingState::SEARCH: return "buscando paredes";
    case ParkingState::PASSING: return "rebasando paredes";
    case ParkingState::ALIGNING: return "avance final al hueco";
    case ParkingState::BRAKING: return "frenando";
    case ParkingState::REVERSE_TURN: return "entrando de reversa";
    case ParkingState::REVERSE_STRAIGHT: return "enderezando";
    case ParkingState::DONE:
      return parkingSucceeded ? "estacionado" : "SIN ESTACIONAR";
    default: return "sin iniciar";
  }
}

void setParkingState(ParkingState nextState, const unsigned long nowMs) {
  parkingState = nextState;
  parkingStateStartMs = nowMs;
  parkingPhaseStartPulses = readEncoderPulseCount();

  Serial.print("ESTACIONAMIENTO: ");
  Serial.println(parkingStateText());
}

float parkingPhaseDistanceMm() {
  const uint32_t pulses =
      readEncoderPulseCount() - parkingPhaseStartPulses;
  return static_cast<float>(pulses) * ENCODER_MM_PER_PULSE;
}

// Fases en las que el robot todavia avanza y el pasillo sigue mandando.
bool parkingIsMovingForward() {
  return parkingState == ParkingState::SEARCH ||
         parkingState == ParkingState::PASSING ||
         parkingState == ParkingState::ALIGNING;
}

uint8_t parkingForwardSpeedCap() {
  // Mientras no haya visto nada, buscar rapido. Con la pared a la vista, la
  // precision importa mas que el tiempo.
  if (parkingState == ParkingState::SEARCH) {
    return PARKING_SEARCH_SPEED_PERCENT;
  }

  return PARKING_APPROACH_SPEED_PERCENT;
}

// El servo se manda en grados respecto al centro, hacia el lado del hueco.
// parkingSide vale +1 a la derecha y -1 a la izquierda; CORRIDOR_SERVO_DIRECTION
// ya trae calibrado que signo de servo corresponde a cada lado.
int parkingServoTowardGap(int correctionDegrees) {
  if (parkingSide == 0) {
    return SERVO_CENTER_DEGREES;
  }

  // Girar hacia la izquierda es el signo CORRIDOR_SERVO_DIRECTION; hacia la
  // derecha, el contrario.
  const int leftSign = CORRIDOR_SERVO_DIRECTION;
  const int towardGapSign = (parkingSide < 0) ? leftSign : -leftSign;

  return constrain(
      SERVO_CENTER_DEGREES + towardGapSign * correctionDegrees,
      SERVO_CENTER_DEGREES - SERVO_MAX_CORRECTION_DEGREES,
      SERVO_CENTER_DEGREES + SERVO_MAX_CORRECTION_DEGREES
  );
}

// Sesgo que se suma a la correccion del PID para arrimarse al lado del
// hueco. Acorta la maniobra de reversa sin soltar el seguimiento de pared.
float parkingCorridorBiasDegrees() {
  if (!parkingIsMovingForward() || parkingSide == 0) {
    return 0.0f;
  }

  const int leftSign = CORRIDOR_SERVO_DIRECTION;
  const int towardGapSign = (parkingSide < 0) ? leftSign : -leftSign;

  return static_cast<float>(
      towardGapSign * PARKING_SIDE_BIAS_DEGREES
  );
}

void finishParking(const char *reason, const unsigned long nowMs) {
  parkingFinishReason = reason;
  stopMotor();
  commandServoAngle(static_cast<float>(SERVO_CENTER_DEGREES));
  setParkingState(ParkingState::DONE, nowMs);

  missionComplete = true;
  completedLaps = TARGET_LAPS;
  if (roundTimerStarted) {
    roundFinishMs = millis();
  }

  Serial.print("*** ESTACIONAMIENTO TERMINADO: ");
  Serial.print(reason);
  Serial.println(" ***");
}

void beginParking(const unsigned long nowMs) {
  if (parkingState != ParkingState::IDLE) {
    return;
  }

  clearPillarLock("inicio del estacionamiento");
  collisionRecoveryState = CollisionRecoveryState::IDLE;
  collisionRecoveryStateStartMs = 0;
  collisionReverseStartPulses = 0;
  collisionTriggerFrames = 0;
  collisionClearFrames = 0;
  collisionRecoveryColorId = 0;
  resetCorridorPid();

  parkingStartedMs = nowMs;
  parkingSucceeded = false;
  parkingFinishReason = "";
  parkingSide = 0;
  parkingWallsPassed = 0;
  parkingSeenFrames = 0;
  parkingLostFrames = 0;
  parkingWallInSight = false;

  // El hueco esta en la pared EXTERIOR, y de que lado queda esa pared se
  // sabe por el sentido de la vuelta. Fijarlo aqui, y no al final, hace que
  // el sesgo lateral arrime al robot a esa pared mientras busca: la camara
  // barre justo donde tiene que estar el estacionamiento.
  //
  // La camara lo corrige despues si resulta estar del otro lado.
  if (PARKING_SIDE_FROM_TURN_FALLBACK &&
      fabsf(accumulatedTurnDegrees) >= DEGREES_PER_LAP) {
    const bool counterClockwise =
        (accumulatedTurnDegrees > 0.0f) == (PARKING_CCW_YAW_SIGN > 0);
    parkingSide = counterClockwise ? 1 : -1;

    Serial.print("ESTACIONAMIENTO: se busca pegado a la pared ");
    Serial.println(parkingSide > 0 ? "DERECHA" : "IZQUIERDA");
  }

  setParkingState(ParkingState::SEARCH, nowMs);

  // Luz alta desde el primer instante de la busqueda, para que el magenta se
  // vea como se calibro.
  setLedBrightnessTarget(PARKING_LED_BRIGHTNESS_PERCENT);

  Serial.print(
      "*** VUELTAS COMPLETAS: empieza la busqueda del estacionamiento, luz al "
  );
  Serial.print(PARKING_LED_BRIGHTNESS_PERCENT);
  Serial.println("% ***");
}

// Cuenta paredes con histeresis: una pared se da por vista tras varias tramas
// seguidas y por rebasada tras varias sin ella. Asi un parpadeo no suma.
void updateParkingWallCount(const OpenMVData &data) {
  const bool wallVisible =
      data.parkingDetected != 0 && data.parkingArea >= PARKING_MIN_AREA;

  if (wallVisible) {
    parkingLostFrames = 0;

    if (!parkingWallInSight) {
      if (parkingSeenFrames < 255) {
        ++parkingSeenFrames;
      }

      if (parkingSeenFrames >= PARKING_SEEN_FRAMES) {
        parkingWallInSight = true;
        parkingSeenFrames = 0;

        // La camara dice de que lado estan las paredes. Es evidencia directa
        // y no depende del convenio de signos del BNO085.
        if (parkingSide == 0 && data.parkingX != 0) {
          parkingSide = (data.parkingX > 0) ? 1 : -1;
          Serial.print("ESTACIONAMIENTO: hueco a la ");
          Serial.println(parkingSide > 0 ? "DERECHA" : "IZQUIERDA");
        }

        Serial.print("ESTACIONAMIENTO: pared a la vista, X=");
        Serial.println(data.parkingX);
      }
    }
    return;
  }

  parkingSeenFrames = 0;

  if (!parkingWallInSight) {
    return;
  }

  if (parkingLostFrames < 255) {
    ++parkingLostFrames;
  }

  if (parkingLostFrames >= PARKING_LOST_FRAMES) {
    parkingWallInSight = false;
    parkingLostFrames = 0;
    ++parkingWallsPassed;

    Serial.print("ESTACIONAMIENTO: pared rebasada ");
    Serial.print(parkingWallsPassed);
    Serial.print("/");
    Serial.println(PARKING_WALLS_TO_PASS);
  }
}

// Confirma con los ultrasonicos que el costado del hueco esta despejado. Es
// la comprobacion que la camara no puede dar: cuando el robot llega a la
// altura del hueco, las paredes ya salieron de su campo de vision.
bool parkingGapConfirmedBySensors() {
  if (parkingSide == 0) {
    return false;
  }

  if (latestFrontDistanceAtMs == 0 ||
      millis() - latestFrontDistanceAtMs > SENSOR_FAILSAFE_TIMEOUT_MS) {
    return false;
  }

  const uint16_t sideMm =
      nearestSideDistanceMm(parkingSide < 0, latestDistancesMm);

  return sideMm != INVALID_DISTANCE_MM && sideMm >= PARKING_GAP_SIDE_MM;
}

void serviceParking(const unsigned long nowMs) {
  if (!parkingActive()) {
    return;
  }

  // Sin ultrasonicos el robot no puede ni seguir el pasillo ni confirmar el
  // hueco: seguir "buscando" 70 segundos es fingir que trabaja. Se corta ya y
  // se dice por que, que es lo unico util en ese momento.
  if (lastValidSensorMs != 0 &&
      nowMs - lastValidSensorMs >= PARKING_SENSOR_MUTE_ABORT_MS) {
    finishParking("SIN SENSORES I2C: el Nano dejo de responder", nowMs);
    return;
  }

  if (nowMs - parkingStartedMs >= PARKING_TOTAL_MAX_MS) {
    // Sin camara no hay estacionamiento posible: conviene decirlo aqui, que
    // es donde se nota, y no dejarlo como un "se acabo el tiempo" a secas.
    if (communicationTimedOut || openMVData.receivedAtMs == 0) {
      finishParking("se agoto el tiempo SIN DATOS DE LA CAMARA", nowMs);
    } else if (parkingWallsPassed == 0) {
      finishParking("se agoto el tiempo sin ver ninguna pared", nowMs);
    } else {
      finishParking("se agoto el tiempo a medias", nowMs);
    }
    return;
  }

  const unsigned long phaseMs = nowMs - parkingStateStartMs;

  switch (parkingState) {
    case ParkingState::SEARCH: {
      // Sigue el pasillo normal, mas despacio, hasta ver la primera
      // pared. El tope de velocidad se aplica sobre el PID de pasillo.
      if (parkingWallInSight || parkingWallsPassed > 0) {
        setParkingState(ParkingState::PASSING, nowMs);
      }
      return;
    }

    case ParkingState::PASSING: {
      // El volante lo sigue llevando el PID de pasillo, con un sesgo
      // hacia el lado del hueco: soltar el seguimiento de pared aqui
      // dejaria al robot a la deriva justo al lado de la pared.
      if (parkingWallsPassed >= PARKING_WALLS_TO_PASS) {
        setParkingState(ParkingState::ALIGNING, nowMs);
      }
      return;
    }

    case ParkingState::ALIGNING: {
      // Ultima oportunidad de saber el lado: si la camara nunca lo dijo,
      // se deduce del sentido de la vuelta.
      if (parkingSide == 0 && PARKING_SIDE_FROM_TURN_FALLBACK &&
          fabsf(accumulatedTurnDegrees) >= DEGREES_PER_LAP) {
        const bool counterClockwise =
            (accumulatedTurnDegrees > 0.0f) ==
            (PARKING_CCW_YAW_SIGN > 0);
        parkingSide = counterClockwise ? 1 : -1;

        Serial.print("ESTACIONAMIENTO: lado deducido del giro -> ");
        Serial.println(parkingSide > 0 ? "DERECHA" : "IZQUIERDA");
      }

      const bool distanceReached =
          parkingPhaseDistanceMm() >=
          static_cast<float>(PARKING_PASS_DISTANCE_MM);

      if (distanceReached || parkingGapConfirmedBySensors() ||
          phaseMs >= PARKING_PHASE_MAX_MS) {
        stopMotor();

        // Sin saber de que lado esta el hueco, entrar de reversa seria
        // adivinar: mejor quedarse quieto que meterse contra una pared.
        if (parkingSide == 0) {
          finishParking("no se identifico el lado del hueco", nowMs);
          return;
        }

        setParkingState(ParkingState::BRAKING, nowMs);
      }
      return;
    }

    case ParkingState::BRAKING: {
      stopMotor();

      // El volante se preposiciona mientras el robot todavia se detiene.
      commandServoAngle(
          static_cast<float>(
              parkingServoTowardGap(PARKING_REVERSE_STEER_DEGREES)
          )
      );

      if (phaseMs >= PARKING_BRAKE_MS) {
        setParkingState(ParkingState::REVERSE_TURN, nowMs);
      }
      return;
    }

    case ParkingState::REVERSE_TURN: {
      // En reversa la geometria invierte el efecto del volante, asi que para
      // meter la cola en el hueco las ruedas apuntan al lado contrario.
      commandServoAngle(
          static_cast<float>(
              parkingServoTowardGap(-PARKING_REVERSE_STEER_DEGREES)
          )
      );
      setMotorReverseSpeedPercent(PARKING_REVERSE_SPEED_PERCENT);

      if (parkingPhaseDistanceMm() >=
              static_cast<float>(PARKING_REVERSE_TURN_MM) ||
          phaseMs >= PARKING_PHASE_MAX_MS) {
        setParkingState(ParkingState::REVERSE_STRAIGHT, nowMs);
      }
      return;
    }

    case ParkingState::REVERSE_STRAIGHT: {
      commandServoAngle(
          static_cast<float>(
              parkingServoTowardGap(PARKING_STRAIGHTEN_STEER_DEGREES)
          )
      );
      setMotorReverseSpeedPercent(PARKING_REVERSE_SPEED_PERCENT);

      if (parkingPhaseDistanceMm() >=
              static_cast<float>(PARKING_REVERSE_STRAIGHT_MM) ||
          phaseMs >= PARKING_PHASE_MAX_MS) {
        parkingSucceeded = true;
        finishParking("dentro del cajon", nowMs);
      }
      return;
    }

    default:
      return;
  }
}

// ----------------------- Recepcion no bloqueante --------------------

void printOpenMVPacket(const OpenMVData &data, int servoAngle) {
  Serial.print("OpenMV ID=");
  Serial.print(data.detectedId);
  Serial.print(" X=");
  Serial.print(data.xReference);
  Serial.print(" Y=");
  Serial.print(data.yReference);
  Serial.print(" ROI=");
  Serial.print(data.roiCode);
  Serial.print(" COL=");
  Serial.print(data.collisionId);
  Serial.print(" W=");
  Serial.print(data.wallBlack);
  if (data.parkingDetected) {
    Serial.print(" | ESTACIONAMIENTO X=");
    Serial.print(data.parkingX);
    Serial.print(" area=");
    Serial.print(data.parkingArea);
  }
  if (sideWallGuardActive) {
    Serial.print(" | PARED_LATERAL a ");
    Serial.print(sideWallGuardDistanceMm / 10);
    Serial.print(" cm");
  }
  Serial.print(" | control=");

  if (wallEscapeActive()) {
    Serial.println("ESCAPE_PARED");
  } else if (collisionRecoveryActive()) {
    Serial.println("RECUPERACION_COLISION");
  } else if (pillarLockActive && pillarHasSteeringControl &&
             lockedPillarSeenBySideSensors) {
    Serial.print("RODEO_PILAR servo=");
    Serial.println(servoAngle);
  } else if (pillarLockActive && pillarHasSteeringControl) {
    Serial.print("PILAR_ENCLAVADO servo=");
    Serial.println(servoAngle);
  } else if (pillarLockActive) {
    Serial.println("DESPEJE_PILAR");
  } else if (corridorReturnActive) {
    Serial.println("RETORNO_GRADUAL");
  } else {
    Serial.println("PASILLO");
  }
}

// Los avisos se agrupan: imprimir cada trama mala a 115200 baudios llena
// el buffer de Serial, y ahi Serial.print bloquea el loop completo.
void reportInvalidOpenMVLine(const char *line) {
  ++invalidFrameCount;
  ++totalInvalidFrames;

  const unsigned long nowMs = millis();
  if (lastInvalidFrameLogMs != 0 &&
      nowMs - lastInvalidFrameLogMs < OPENMV_INVALID_LOG_INTERVAL_MS) {
    return;
  }
  lastInvalidFrameLogMs = nowMs;

  Serial.print("TRAMA OpenMV DESCARTADA (");
  Serial.print(invalidFrameCount);
  Serial.print(" desde el ultimo aviso): ");
  Serial.println(line);
  invalidFrameCount = 0;
}

// Suelta el pilar enclavado cuando ya no se ve y la camara insiste con uno
// de otro color. Sin esto el robot sigue dirigiendo hacia las coordenadas
// viejas del anterior, que es lo que lo mete contra la pared al pasar de
// verde a rojo.
void updatePillarSwitchRequest(const OpenMVData &data) {
  if (!pillarLockActive) {
    pillarSwitchFrames = 0;
    return;
  }

  const bool otherPillarVisible =
      hasValidPillarTarget(data) && data.detectedId != lockedPillarId;

  if (!otherPillarVisible) {
    pillarSwitchFrames = 0;
    return;
  }

  const unsigned long staleLimitMs = lockedPillarSeenBySideSensors
      ? PILLAR_SWITCH_WRAP_STALE_MS
      : PILLAR_SWITCH_STALE_MS;

  // Si el enclavado sigue apareciendo, el rebase esta en curso y no se
  // abandona por mucho que asome el siguiente.
  if (data.receivedAtMs - lastLockedPillarSeenMs < staleLimitMs) {
    pillarSwitchFrames = 0;
    return;
  }

  if (++pillarSwitchFrames < PILLAR_SWITCH_CONFIRM_FRAMES) {
    return;
  }

  Serial.print("Cambio de pilar: aparece ");
  Serial.print(data.detectedId == 3 ? "VERDE" : "ROJO");
  Serial.print(" y el ");
  Serial.print(lockedPillarId == 3 ? "VERDE" : "ROJO");
  Serial.println(" enclavado ya no se ve");

  clearPillarLock("otro pilar a la vista y el anterior perdido");
}

void processCompleteLine() {
  rxBuffer[rxLength] = '\0';

  OpenMVData newData;
  if (parseOpenMVLine(rxBuffer, newData)) {
    openMVData = newData;
    communicationTimedOut = false;

    // El conteo de paredes corre a ritmo de camara, no de sensores.
    if (parkingActive()) {
      updateParkingWallCount(openMVData);
    }

    // Con el estacionamiento en marcha nada mas puede tomar el mando: ni
    // una colision de color ni un cambio de pilar deben interrumpir la
    // maniobra a medias.
    if (parkingActive()) {
      refreshLedBrightnessTarget();
      printOpenMVPacket(openMVData, lastWrittenServoAngle);
      rxLength = 0;
      return;
    }

    int servoAngle = lastWrittenServoAngle;
    const bool validTarget = hasValidPillarTarget(openMVData);

    if (handleCollisionPacket(openMVData)) {
      refreshLedBrightnessTarget();
      printOpenMVPacket(openMVData, SERVO_CENTER_DEGREES);
      rxLength = 0;
      return;
    }

    // Se evalua antes de enclavar para que el pilar nuevo pueda tomar el
    // mando en esta misma trama, sin un ciclo perdido dirigiendo a ciegas.
    updatePillarSwitchRequest(openMVData);

    if (!pillarLockActive && validTarget) {
      beginPillarLock(openMVData);
    }

    if (pillarLockActive && validTarget &&
        openMVData.detectedId == lockedPillarId &&
        !lockedPillarSeenBySideSensors) {
      lockedPillarData = openMVData;
      lockedPillarReachedLowRoi =
          lockedPillarReachedLowRoi || openMVData.roiCode == 2;
      lastLockedPillarSeenMs = openMVData.receivedAtMs;
      takePillarSteeringControl();
      servoAngle = commandServoAngle(
          static_cast<float>(
              calculatePillarServoWithFrontGuard(openMVData)
          )
      );
    }

    refreshLedBrightnessTarget();
    printOpenMVPacket(openMVData, servoAngle);
  } else {
    reportInvalidOpenMVLine(rxBuffer);
  }

  rxLength = 0;
}

// Si el loop se retraso, la camara siguio enviando y el buffer guarda
// imagenes viejas. Se tiran para dirigir siempre con la mas reciente.
void discardStaleOpenMVBacklog() {
  if (openMVSerial.available() <= OPENMV_MAX_BACKLOG_BYTES) {
    return;
  }

  bool endedOnNewline = false;
  while (openMVSerial.available() > OPENMV_MAX_BACKLOG_BYTES) {
    endedOnNewline =
        (static_cast<char>(openMVSerial.read()) == '\n');
  }

  rxLength = 0;
  // Si el corte quedo a mitad de una linea se ignora lo que resta de ella,
  // porque media trama con comas todavia parece valida.
  discardingLongLine = !endedOnNewline;

  reportInvalidOpenMVLine("atraso descartado: loop lento");
}

void readOpenMV() {
  discardStaleOpenMVBacklog();

  uint8_t linesProcessed = 0;

  while (openMVSerial.available() > 0) {
    const char character = static_cast<char>(openMVSerial.read());

    if (character == '\n') {
      if (discardingLongLine) {
        discardingLongLine = false;
        rxLength = 0;
        reportInvalidOpenMVLine("linea demasiado larga o incompleta");
      } else if (rxLength > 0) {
        processCompleteLine();

        // El resto espera a la siguiente vuelta: los sensores y el servo
        // no pueden quedarse esperando a que se vacie una rafaga.
        if (++linesProcessed >= OPENMV_MAX_LINES_PER_LOOP) {
          return;
        }
      }
      continue;
    }

    // El nulo y el retorno de carro no rompen la linea en curso.
    if (character == '\r' || character == '\0' || discardingLongLine) {
      continue;
    }

    if (rxLength < RX_BUFFER_SIZE - 1) {
      rxBuffer[rxLength++] = character;
    } else {
      rxLength = 0;
      discardingLongLine = true;
    }
  }
}

void applyOpenMVCommunicationFailsafe() {
  if (openMVData.receivedAtMs == 0 ||
      millis() - openMVData.receivedAtMs <= OPENMV_TIMEOUT_MS) {
    return;
  }

  if (!communicationTimedOut) {
    communicationTimedOut = true;
    clearPillarLock("timeout de comunicacion OpenMV");
    setLedBrightnessTarget(LED_BRIGHTNESS_FAR_PERCENT);
    Serial.println(
        "TIMEOUT OpenMV: regreso gradual al control de pasillo"
    );
  }
}

// -------------------------- Depuracion -------------------------------

void printDistances(const uint16_t distancesMm[SENSOR_COUNT]) {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    Serial.print("S");
    Serial.print(i + 1);
    Serial.print("=");

    if (distancesMm[i] == INVALID_DISTANCE_MM) {
      Serial.print("sin eco");
    } else {
      Serial.print(distancesMm[i] / 10);
      Serial.print(".");
      Serial.print(distancesMm[i] % 10);
      Serial.print("cm");
    }

    if (i < SENSOR_COUNT - 1) {
      Serial.print(" | ");
    }
  }
  Serial.println();
}

void printCorridorControl(const CorridorControlState &state) {
  Serial.print("Pasillo error=");
  Serial.print(state.error, 1);
  Serial.print(" P=");
  Serial.print(state.pTerm, 1);
  Serial.print(" I=");
  Serial.print(state.iTerm, 1);
  Serial.print(" D=");
  Serial.print(state.dTerm, 1);
  Serial.print(" | objetivo=");
  Serial.print(state.targetServoAngle);
  Serial.print(" aplicado=");
  Serial.print(state.appliedServoAngle);
  Serial.print(" | modo=");
  Serial.print(corridorReturnActive ? "RETORNO" : "PASILLO");
  Serial.print(" | motor=");
  Serial.print(state.speedPercent);
  Serial.print("% | luz=");
  Serial.print(currentLedBrightnessPercent);
  Serial.print("% objetivo=");
  Serial.print(targetLedBrightnessPercent);
  Serial.println("%");
}

// Durante las vueltas la camara NO debe buscar paredes de estacionamiento:
// vistas de canto parecen pilares y el robot se iba directo contra ellas.
void sendOpenMVParkingMode(const unsigned long nowMs) {
  static unsigned long lastSentMs = 0;
  static bool lastSentEnabled = false;
  static bool everSent = false;

  const bool enabled = OPENMV_PARKING_ALWAYS_SEARCH || parkingActive();

  const bool changed = !everSent || enabled != lastSentEnabled;
  if (!changed &&
      nowMs - lastSentMs < OPENMV_PARKING_MODE_INTERVAL_MS) {
    return;
  }

  lastSentMs = nowMs;
  lastSentEnabled = enabled;
  everSent = true;

  openMVSerial.print("P,");
  openMVSerial.print(enabled ? 1 : 0);
  openMVSerial.print('\n');

  if (changed) {
    Serial.print("ESTACIONAMIENTO: se avisa a la camara -> ");
    Serial.println(enabled ? "BUSCAR paredes" : "ignorar paredes");
  }
}

void sendOpenMVPathTelemetry(const unsigned long nowMs) {
  static unsigned long lastTelemetryMs = 0;

  if (!OPENMV_PATH_TELEMETRY_ENABLED ||
      nowMs - lastTelemetryMs < OPENMV_PATH_TELEMETRY_INTERVAL_MS) {
    return;
  }
  lastTelemetryMs = nowMs;

  const int servoAngle = lastWrittenServoAngle >= 0
                             ? lastWrittenServoAngle
                             : SERVO_CENTER_DEGREES;

  // Formato: T,servo,motor,S1,S2,S3,S4,S5\n
  openMVSerial.print("T,");
  openMVSerial.print(servoAngle);
  openMVSerial.print(',');
  openMVSerial.print(appliedMotorSpeedPercent);
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    openMVSerial.print(',');
    openMVSerial.print(latestDistancesMm[i]);
  }
  openMVSerial.print('\n');
}

// ----------------------------- Arduino -------------------------------

void setup() {
  Serial.begin(115200);
  beginEncoder();

  Wire.begin(
      ESP32_SDA_PIN,
      ESP32_SCL_PIN,
      I2C_FREQUENCY_HZ
  );
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  beginBno085I2c();

  // Debe pedirse antes de begin() para que el driver lo reserve.
  openMVSerial.setRxBufferSize(OPENMV_RX_HARDWARE_BUFFER_BYTES);
  openMVSerial.begin(
      OPENMV_BAUD,
      SERIAL_8N1,
      OPENMV_RX_PIN,
      OPENMV_TX_PIN
  );

  // Lo que llego mientras arrancaba el ESP32 casi siempre es media trama.
  delay(5);
  while (openMVSerial.available() > 0) {
    openMVSerial.read();
  }
  rxLength = 0;
  discardingLongLine = false;

  servoReady = beginServoPwm();
  if (servoReady) {
    commandServoAngle(SERVO_CENTER_DEGREES);
  }

  motorReady = beginMotorPwm();
  if (motorReady) {
    stopMotor();
  }

  lastValidSensorMs = millis();

  delay(10);
  const bool ledsConfigured =
      applyLedBrightnessNow(LED_BRIGHTNESS_FAR_PERCENT);
  beginDashboard();

  Serial.println();
  Serial.println("ESP32 obstaculos V2 + pasillo + BNO085");
  Serial.print("OpenMV UART2 RX=");
  Serial.print(OPENMV_RX_PIN);
  Serial.print(" TX=");
  Serial.print(OPENMV_TX_PIN);
  Serial.print(" @ ");
  Serial.print(OPENMV_BAUD);
  Serial.print(" baudios; tramas de ");
  Serial.print(OPENMV_MIN_FIELDS);
  Serial.print(" o mas campos, X ");
  Serial.println(
      OPENMV_X_IS_NORMALIZED ? "normalizada" : "en pixeles"
  );
  Serial.println(
      "Prioridad: PILAR -> RETORNO GRADUAL -> PASILLO"
  );
  Serial.print("Retorno al pasillo: ");
  Serial.print(CORRIDOR_RETURN_RATE_DEGREES_PER_SECOND, 1);
  Serial.println(" grados/segundo");

  if (!servoReady) {
    Serial.println("ERROR: no se pudo iniciar el PWM del servo");
  }
  if (!motorReady) {
    Serial.println("ERROR: no se pudo iniciar el PWM del motor");
  }
  if (!ledsConfigured) {
    Serial.println("ERROR I2C: no se pudo configurar los LEDs");
  }

  Serial.println(
      "Motor detenido; arrancara al recibir sensores validos"
  );
  Serial.print("Encoder A=D5, B=D9; PWM inicial=");
  Serial.print(MOTOR_START_DETECTION_PERCENT);
  Serial.println("%");
}

void loop() {
  updateDashboard();
  if (missionComplete) {
    return;
  }

  readOpenMV();
  applyOpenMVCommunicationFailsafe();
  serviceLedBrightnessTransition();

  static unsigned long lastPollMs = 0;
  static unsigned long lastPrintMs = 0;

  const unsigned long nowMs = millis();
  updateBno085(nowMs);
  updateOdometry();

  if (lapTrackingActive && bno085Initialized && bno085HasHeading &&
      nowMs - lastBno085FrameMs >= BNO085_FAILSAFE_TIMEOUT_MS &&
      !bno085FailsafeActive) {
    bno085FailsafeActive = true;
    Serial.println("AVISO: se perdio el rumbo del BNO085");
  }

  if (missionComplete) {
    return;
  }

  if (REQUIRE_BNO085_TO_MOVE &&
      (!bno085HasHeading || bno085FailsafeActive)) {
    if (targetMotorSpeedPercent > 0 || appliedMotorSpeedPercent > 0) {
      stopMotor();
    }
    return;
  }

  if (!wallEscapeActive()) {
    serviceCollisionRecovery(nowMs);
  }

  serviceParking(nowMs);

  // El Nano lleva mudo demasiado: se reinicia el bus por si quedo trabado.
  if (lastValidSensorMs != 0 &&
      nowMs - lastValidSensorMs >= I2C_RECOVERY_INTERVAL_MS &&
      nowMs - lastI2cRecoveryMs >= I2C_RECOVERY_INTERVAL_MS) {
    lastI2cRecoveryMs = nowMs;
    Serial.println("I2C: sensores mudos, se reinicia el bus");
    Wire.end();
    Wire.begin(ESP32_SDA_PIN, ESP32_SCL_PIN, I2C_FREQUENCY_HZ);
    Wire.setTimeOut(I2C_TIMEOUT_MS);
  }

  if (nowMs - lastPollMs >= SENSOR_POLL_INTERVAL_MS) {
    lastPollMs = nowMs;

    uint16_t distancesMm[SENSOR_COUNT];
    if (readDistancesMm(distancesMm)) {
      lastValidSensorMs = nowMs;
      distanceSensorFailsafeActive = false;
      for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        latestDistancesMm[i] = distancesMm[i];
      }
      latestFrontDistanceMm = distancesMm[2];
      latestFrontDistanceAtMs = nowMs;
      refreshLedBrightnessTarget();
      const bool escapingWall =
          (parkingActive() && !parkingIsMovingForward())
              ? false
              : updateWallEscape(distancesMm[2], nowMs);

      if (!escapingWall) {
        updateFrontGuard(distancesMm[2]);
      } else {
        resetFrontGuard();
      }

      if (!escapingWall && !collisionRecoveryActive()) {
        updatePillarDistanceHandoff(distancesMm, nowMs);
      }

      if (escapingWall) {
        if (nowMs - lastPrintMs >= SERIAL_PRINT_INTERVAL_MS) {
          lastPrintMs = nowMs;
          printDistances(distancesMm);
          Serial.print("Control=ESCAPE_PARED | servo=");
          Serial.print(lastWrittenServoAngle);
          Serial.print(" | motor=");
          Serial.print(appliedMotorSpeedPercent);
          Serial.println("%");
        }
      } else if (collisionRecoveryActive()) {
        if (nowMs - lastPrintMs >= SERIAL_PRINT_INTERVAL_MS) {
          lastPrintMs = nowMs;
          printDistances(distancesMm);
          Serial.print("Control=RECUPERACION_COLISION | servo=");
          Serial.print(lastWrittenServoAngle);
          Serial.print(" | motor=");
          Serial.print(targetMotorSpeedPercent);
          Serial.println("%");
        }
      } else if (parkingActive() && !parkingIsMovingForward()) {
        // Solo al frenar y entrar de reversa la maniobra toma el mando.
        // Mientras avanza sigue mandando el PID de pasillo, con el sesgo
        // y el tope de velocidad del estacionamiento.
        if (nowMs - lastPrintMs >= SERIAL_PRINT_INTERVAL_MS) {
          lastPrintMs = nowMs;
          printDistances(distancesMm);
          Serial.print("Control=ESTACIONAMIENTO ");
          Serial.print(parkingStateText());
          Serial.print(" | servo=");
          Serial.print(lastWrittenServoAngle);
          Serial.print(" | paredes=");
          Serial.print(parkingWallsPassed);
          Serial.print("/");
          Serial.println(PARKING_WALLS_TO_PASS);
        }
      } else if (pillarHasSteeringControl) {
        // Los sensores no tocan el servo mientras exista un pilar.
        updateMotorWhilePillarControls(lockedPillarData, distancesMm);

        if (nowMs - lastPrintMs >= SERIAL_PRINT_INTERVAL_MS) {
          lastPrintMs = nowMs;
          printDistances(distancesMm);
          Serial.print("Control=PILAR | servo=");
          Serial.print(lastWrittenServoAngle);
          Serial.print(" | motor=");
          Serial.print(targetMotorSpeedPercent);
          Serial.print("% | proteccion_frontal=");
          Serial.print(frontGuardActive ? "SI" : "no");
          Serial.print(" | luz=");
          Serial.print(currentLedBrightnessPercent);
          Serial.print("% objetivo=");
          Serial.print(targetLedBrightnessPercent);
          Serial.println("%");
        }
      } else {
        const CorridorControlState state =
            calculateCorridorControl(distancesMm);

        if (nowMs - lastPrintMs >= SERIAL_PRINT_INTERVAL_MS) {
          lastPrintMs = nowMs;
          printDistances(distancesMm);
          printCorridorControl(state);
        }
      }
    }
  }

  serviceMotorSoftStart(nowMs);

  sendOpenMVParkingMode(nowMs);
  sendOpenMVPathTelemetry(nowMs);

  // Durante la reversa del estacionamiento no hay sensores que mirar
  // hacia atras: la maniobra ya esta acotada por encoder y por tiempo.
  if (!parkingActive() && targetMotorSpeedPercent > 0 &&
      nowMs - lastValidSensorMs >= SENSOR_FAILSAFE_TIMEOUT_MS) {
    stopMotor();

    if (!distanceSensorFailsafeActive) {
      distanceSensorFailsafeActive = true;
      Serial.println("Motor detenido: sin datos I2C de sensores");
    }
  }
}
