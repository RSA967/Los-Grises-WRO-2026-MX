# ============================================================
# OPENMV FIRMWARE 5
# Detección de pilares rojos y verdes
#
# UART:
# ID, X, Y, AREA, ROI, COLISION, PARED_NEGRA
#
# X siempre conserva la esquina inferior original del color:
# - Verde: esquina inferior izquierda.
# - Rojo: esquina inferior derecha.
# Se transmite mapeada entre -100 y 100. El ESP32 convierte ese
# rango al recorrido angular seguro del servo.
#
# En ROI_HIGH el robot sigue directamente esa esquina.
# En ROI_LOW esa misma esquina sirve para medir la separacion
# respecto al ancho del robot (ROI de colision).
# ROI: 0 = ninguna, 1 = alta, 2 = baja.
#
# ID:
# 0 = Sin detección
# 3 = Verde
# 5 = Rojo
# ============================================================

import sensor
import time
import micropython

from machine import UART

try:
    import binascii
except ImportError:
    import ubinascii as binascii


# ============================================================
# CONFIGURACIÓN GENERAL
# ============================================================

SHOW_DEBUG = True
DRAW_BOXES = True
DRAW_TEXT = True

# Superpone la trayectoria que el ESP32 esta ordenando al robot. El ESP32
# devuelve por UART una linea "T,servo,motor,S1,S2,S3,S4,S5". Esta ayuda es
# solamente visual: no modifica la deteccion ni los datos de control enviados.
SHOW_PATH_PREVIEW = False

PRINT_UART_DATA = True
PRINT_FPS = False

# Imprimir la imagen completa en Base64 reduce mucho los FPS.
PRINT_BASE64_FRAME = False
BASE64_JPEG_QUALITY = 35

# Exposicion fija usada despues del ajuste automatico inicial.
# 10000 us oscurecia demasiado la imagen.
CAMERA_EXPOSURE_US = 30000

# True:
# X se envía normalizada entre -100 y 100.
#
# False:
# X se envía en píxeles entre 0 y 319.
X_AS_NORMALIZED = True

# Permite utilizar los dos umbrales de cada color.
USE_MULTI_RANGES = True


# ============================================================
# VISTA PREVIA DE TRAYECTORIA
# ============================================================

SERVO_CENTER_DEG = 90
SERVO_MAX_CORRECTION_DEG = 40

# Cambiar a -1 si la curva aparece al lado contrario del giro fisico.
PATH_SCREEN_DIRECTION = 1

# La proyeccion es intencionalmente calibrable: no pretende medir distancias
# metricas hasta realizar una calibracion de camara sobre el robot.
PATH_NEAR_Y = 236
PATH_FAR_Y = 86
PATH_NEAR_HALF_WIDTH_PX = 34
PATH_FAR_HALF_WIDTH_PX = 7
PATH_MAX_BEND_PX = 105
PATH_SEGMENTS = 12

TELEMETRY_TIMEOUT_MS = 350
PATH_FRONT_DANGER_MM = 250
PATH_SIDE_DANGER_MM = 180


# ============================================================
# UMBRALES ACTIVOS
# ============================================================

USE_RED_1 = True
USE_RED_2 = False

USE_GREEN_1 = True
USE_GREEN_2 = False

USE_MAGENTA_1 = True
USE_MAGENTA_2 = False


# ============================================================
# REGIONES DE INTERÉS
# Formato: (x, y, ancho, alto)
# ============================================================

ROI_HIGH = (90, 100, 140, 40)
ROI_LOW = (0, 140, 320, 100)

COLLISION_ROI = (120, 200, 80, 15)

# ROI de calibracion, siempre centrada horizontalmente.
# Cambiar unicamente CENTER_ROI_CENTER_Y para subirla o bajarla:
# 0 es la parte superior y 239 la parte inferior de la imagen.
SHOW_CENTER_ROI = True
CENTER_ROI_WIDTH = 6
CENTER_ROI_HEIGHT = 10
CENTER_ROI_CENTER_Y = 105
# Cada pixel con luminosidad LAB menor o igual a este valor cuenta como negro.
# Subirlo acepta tonos mas claros; bajarlo hace la deteccion mas estricta.
WALL_BLACK_L_MAX = 35
# Porcentaje minimo de pixeles negros dentro del ROI para enviar Wall=1.
WALL_BLACK_MIN_PERCENT = 30

CENTER_ROI = (
    (320 - CENTER_ROI_WIDTH) // 2,
    max(
        0,
        min(
            240 - CENTER_ROI_HEIGHT,
            CENTER_ROI_CENTER_Y - CENTER_ROI_HEIGHT // 2
        )
    ),
    CENTER_ROI_WIDTH,
    CENTER_ROI_HEIGHT
)


# ============================================================
# ZONAS MUERTAS LATERALES
#
# Rechazan falsas detecciones producidas por lineas de pared/esquina. El
# margen crece hacia abajo para seguir la perspectiva de la pista.
# ============================================================

USE_CORNER_DEAD_ZONES = False
CORNER_DEAD_ZONE_TOP_Y = 80
CORNER_DEAD_ZONE_BOTTOM_Y = 214
CORNER_DEAD_ZONE_FAR_MARGIN_PX = 18
CORNER_DEAD_ZONE_NEAR_MARGIN_PX = 58


# ============================================================
# CONFIGURACIÓN UART
# ============================================================

UART_BUS = 3
UART_BAUD = 19200

uart = UART(
    UART_BUS,
    UART_BAUD,
    bits=8,
    parity=None,
    stop=1,
    timeout_char=10
)

micropython.alloc_emergency_exception_buf(200)


# ============================================================
# UMBRALES LAB
#
# Formato:
# (L mínimo, L máximo,
#  A mínimo, A máximo,
#  B mínimo, B máximo)
# ============================================================
# cerca
# lejos

TH_RED_1 = (20, 55, 31, 71, -19, 43)
TH_RED_2 = (28, 69, 47, 127, 0, 127)

TH_GREEN_1 = (32, 68, -128, -16, -128, 127)
TH_GREEN_2 = (34, 72, -74, -18, -128, 127)

TH_MAGENTA_1 = (23, 61, 14, 47, -128, 14)
TH_MAGENTA_2 = (15, 80, 20, 110, -90, -20)


def active_by_flags(threshold_list, flags):
    active_thresholds = []

    for threshold, enabled in zip(threshold_list, flags):
        if enabled and threshold is not None:
            active_thresholds.append(threshold)

    if not active_thresholds:
        active_thresholds.append(threshold_list[0])

    if USE_MULTI_RANGES:
        return active_thresholds

    return [active_thresholds[0]]


THS_RED = active_by_flags(
    [TH_RED_1, TH_RED_2],
    [USE_RED_1, USE_RED_2]
)

THS_GREEN = active_by_flags(
    [TH_GREEN_1, TH_GREEN_2],
    [USE_GREEN_1, USE_GREEN_2]
)

THS_MAGENTA = active_by_flags(
    [TH_MAGENTA_1, TH_MAGENTA_2],
    [USE_MAGENTA_1, USE_MAGENTA_2]
)


# ============================================================
# PARÁMETROS DE DETECCIÓN
# ============================================================

AREA_TH_RG_PX = 30
AREA_TH_RG_PCT = 0

MIN_H_PILLAR = 12
MIN_H_OVER_W = 1.20
MAX_TILT_DEG = 35
MIN_PILLAR_AREA = 60

# Densidad minima: pixeles del blob divididos entre el area de su caja.
#
# Un pilar es un cuerpo solido y llena casi toda su caja (85% o mas). Una
# linea del tapete cruza la caja en diagonal y deja las dos esquinas
# vacias, asi que llena bastante menos (alrededor del 45%). Es lo unico que
# separa de verdad las dos cosas: la franja naranja pasa el filtro de
# proporcion alto/ancho y tambien el de inclinacion.
#
# El porcentaje real se dibuja en pantalla como D:xx% para poder ajustarlo.
# Subirlo es mas estricto; bajarlo acepta pilares mas recortados.
MIN_PILLAR_DENSITY_PERCENT = 60


# Evita confundir una línea horizontal con un pilar.
MAX_LINE_H = 12
MAX_LINE_W_OVER_H = 2.0

# Parámetros de find_blobs.
PIX_TH = 8
AREA_TH = 8
MERGE = True
MARGIN = 5

# Valor enviado cuando no se detecta una esquina válida.
CORNER_SENTINEL = -1


# ============================================================
# ESTACIONAMIENTO
#
# Las paredes son magenta, pero con esta luz el magenta y el rojo se
# confunden. La forma si los separa sin ambiguedad:
#
#   Pilar:   5 cm de ancho x 10 cm de alto -> alto/ancho = 2.0
#   Pared:  20 cm de ancho x 10 cm de alto -> ancho/alto = 2.0
#
# Son cuatro veces distintos en proporcion, asi que el color solo sirve
# para encontrar candidatos y la decision la toma la forma.
# ============================================================

DETECT_PARKING = True

# Estado de la busqueda de estacionamiento MIENTRAS no diga nada el ESP32.
#
# Va en True a proposito. El ESP32 manda "P,0" en cuanto arranca, antes de que
# el robot se mueva, asi que sobre el robot el comportamiento es el que se
# quiere: apagada durante las vueltas y encendida al terminarlas.
#
# En el banco, con el IDE y sin ESP32, no llega nada y se queda encendida:
# asi se puede calibrar TH_MAGENTA sin tocar ninguna constante ni acordarse
# de devolverla despues.
#
# Que se quede encendida por un cable suelto no rompe nada: el ESP32 ignora
# los campos de estacionamiento hasta que el mismo decide que toca aparcar.
PARKING_DETECTION_DEFAULT = True

# Separar pared de pilar por COLOR resulto imposible de afinar: con luz
# artificial el magenta y el rojo se solapan en el canal B, y cada intento
# de apretar el magenta acababa comiendose pilares rojos.
#
# Se separan por FORMA, que no depende de ninguna calibracion:
#
#   Pilar:   5 cm de ancho x 10 cm de alto -> alto/ancho = 2.0
#   Pared:  20 cm de ancho x 10 cm de alto -> ancho/alto = 2.0
#
# Por eso las paredes se buscan en los MISMOS umbrales de rojo y lo unico
# que decide es la proporcion. TH_MAGENTA queda como refuerzo opcional,
# por si algun dia quieres reforzar la deteccion con color.
PARKING_USES_MAGENTA_THRESHOLD = False

# Las paredes se ven desde mas lejos que los pilares, asi que el
# estacionamiento usa su propia ROI, mas alta. Las ROI de pilares
# (ROI_HIGH y ROI_LOW) no cambian: siguen mandando sobre la deteccion de
# pilares y sobre el codigo de ROI que se envia por UART.
#
# 0 es la parte superior de la imagen y 239 la inferior: bajar este valor
# hace que las paredes se vean desde mas lejos.
PARKING_ROI_TOP_Y = 70

# Un blob ancho es pared; uno alto es pilar. Entre 1/1.5 y 1.20 queda una
# tierra de nadie donde no se acepta ninguna de las dos cosas.
# Medido en pista: una pared vista de frente da 2.0, pero en escorzo baja
# a 1.5. Con el umbral en 1.50 quedaba justo en el filo y cualquier
# angulo un poco peor la perdia, asi que se baja a 1.30.
#
# Sigue sin tocar a los pilares: ellos exigen alto/ancho >= 1.20, o sea
# ancho/alto <= 0.83. Entre 0.83 y 1.30 no se acepta ninguna de las dos.
MIN_W_OVER_H_PARKING = 1.30

# Ancho minimo en pixeles. Protege del caso en que un pilar cercano queda
# recortado por el borde de la ROI y su trozo visible parece ancho: la
# pared es cuatro veces mas ancha que el pilar, asi que siempre supera
# este valor con holgura.
MIN_PARKING_W = 20

MIN_PARKING_H = 6
MIN_PARKING_AREA = 120

# Un pilar cercano recortado por el borde superior de PARKING_ROI deja un
# trozo visible ancho y bajo que puede parecer una pared. Si el blob toca
# ese borde no se sabe su altura real, asi que no se acepta como pared.
# Las paredes miden 10 cm y se apoyan en el piso: quedan abajo en la
# imagen, no arriba, asi que esta guarda no les quita nada.
PARKING_REJECT_ROI_TOP_EDGE = True

# Las paredes tambien son cuerpos solidos: descarta lineas del tapete.
MIN_PARKING_DENSITY_PERCENT = 55

# El estacionamiento se apoya solo en TH_MAGENTA. Se quito la opcion de
# aceptar tambien blobs rojos porque obligaba a una busqueda extra por
# cuadro, y ese coste es justo lo que puede tumbar la camara. Ahora hay
# una sola dependencia que ajustar: el umbral magenta.


# ============================================================
# INICIALIZACIÓN DE LA CÁMARA
# ============================================================

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)

# Ajuste automático inicial.
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.set_auto_exposure(True)

sensor.skip_frames(time=800)

# Bloqueo de los valores para estabilizar los colores.
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(
    False,
    exposure_us=CAMERA_EXPOSURE_US
)

sensor.skip_frames(time=300)

clock = time.clock()


# Ultima orden realmente aplicada por el ESP32. Los valores iniciales dibujan
# una trayectoria recta y amarilla hasta recibir la primera telemetria.
telemetry_servo_deg = SERVO_CENTER_DEG
telemetry_motor_percent = 0
telemetry_distances_mm = [2000, 2000, 2000, 2000, 2000]
telemetry_last_ms = 0
telemetry_rx_line = ""

# Lo enciende el ESP32 al terminar las vueltas.
parking_detection_enabled = PARKING_DETECTION_DEFAULT


# ============================================================
# DIMENSIONES DE LA IMAGEN
# ============================================================

IMG_W = 320
IMG_H = 240


# ============================================================
# ROI UNIFICADA PARA LOS PILARES
# ============================================================

pillars_top = min(
    ROI_HIGH[1],
    ROI_LOW[1]
)

pillars_bottom = max(
    ROI_HIGH[1] + ROI_HIGH[3],
    ROI_LOW[1] + ROI_LOW[3]
)

PILLARS_ROI = (
    0,
    pillars_top,
    IMG_W,
    pillars_bottom - pillars_top
)


# ============================================================
# ROI DEL ESTACIONAMIENTO
#
# Empieza mas arriba que la de pilares y llega hasta el borde inferior.
# ============================================================

PARKING_ROI = (
    0,
    PARKING_ROI_TOP_Y,
    IMG_W,
    IMG_H - PARKING_ROI_TOP_Y
)


# ============================================================
# FUNCIONES AUXILIARES
# ============================================================

def best_blob_area(blobs):
    if not blobs:
        return None

    return max(
        blobs,
        key=lambda blob: blob.area
    )


def pillar_area_ok(blob, roi):
    if blob is None:
        return False

    ok_pixels = blob.area >= AREA_TH_RG_PX

    if AREA_TH_RG_PCT > 0:
        roi_area = max(
            1,
            roi[2] * roi[3]
        )

        percentage = (
            blob.area * 100
        ) // roi_area

        ok_percentage = percentage >= AREA_TH_RG_PCT

    else:
        ok_percentage = True

    return ok_pixels and ok_percentage


def corner_lower_left(blob):
    x, y, width, height = blob.rect

    corner_x = x
    corner_y = y + height - 1

    return int(corner_x), int(corner_y)


def corner_lower_right(blob):
    x, y, width, height = blob.rect

    corner_x = x + width - 1
    corner_y = y + height - 1

    return int(corner_x), int(corner_y)


def point_in_roi(x, y, roi):
    roi_x, roi_y, roi_width, roi_height = roi

    inside_x = roi_x <= x < roi_x + roi_width
    inside_y = roi_y <= y < roi_y + roi_height

    return inside_x and inside_y


def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum

    if value > maximum:
        return maximum

    return value


def clamp_i16(value):
    if value < -32768:
        return -32768

    if value > 32767:
        return 32767

    return int(value)


def normalize_x100(x_pixel):
    # Mapeo lineal con extremos exactos:
    # 0 px -> -100, centro -> 0, 319 px -> 100.
    x_pixel = clamp(
        int(x_pixel),
        0,
        IMG_W - 1
    )

    normalized_x = (
        (
            x_pixel * 200
            + (IMG_W - 1) // 2
        )
        // (IMG_W - 1)
    ) - 100

    return clamp(
        normalized_x,
        -100,
        100
    )


def blob_threshold_index(blob):
    code = blob.code

    if code == 0:
        return 0

    # Verifica si solamente existe un bit activo.
    if (code & (code - 1)) == 0:
        index = 1

        while ((code >> (index - 1)) & 1) == 0:
            index += 1

        return index

    return 0


def radians_to_degrees(radians):
    if radians is None:
        return 0.0

    return radians * 57.29578


def looks_like_horizontal_line(blob):
    if blob.h <= MAX_LINE_H:
        return True

    width_over_height = (
        blob.w
        / max(1, blob.h)
    )

    return width_over_height >= MAX_LINE_W_OVER_H


def blob_density_percent(blob):
    # En OpenMV blob.area es el area de la CAJA (ancho por alto) y
    # blob.pixels es cuantos pixeles pertenecen de verdad al blob.
    if blob is None:
        return 0

    box_area = max(1, int(blob.w) * int(blob.h))

    try:
        filled_pixels = int(blob.pixels)
    except Exception:
        # Sin el dato no se puede juzgar: se da por lleno para no
        # descartar pilares buenos por una diferencia de firmware.
        return 100

    return (filled_pixels * 100) // box_area


def is_vertical_pillar(blob):
    if blob is None:
        return False

    if blob.area < MIN_PILLAR_AREA:
        return False

    if blob.h < MIN_H_PILLAR:
        return False

    # Descarta las lineas del tapete: cruzan su caja en diagonal y la
    # dejan medio vacia, mientras que un pilar la llena.
    if blob_density_percent(blob) < MIN_PILLAR_DENSITY_PERCENT:
        return False

    height_over_width = (
        blob.h
        / max(1, blob.w)
    )

    if height_over_width < MIN_H_OVER_W:
        return False

    try:
        rotation_degrees = (
            abs(
                radians_to_degrees(
                    blob.rotation
                )
            )
            % 180.0
        )

        tilt_from_zero = abs(rotation_degrees)
        tilt_from_vertical = abs(rotation_degrees - 90.0)

        tilt = min(
            tilt_from_zero,
            tilt_from_vertical
        )

        if tilt > MAX_TILT_DEG:
            return False

    except Exception:
        # La proporción alto/ancho permanece como filtro.
        pass

    return True


def keep_valid_pillars(blobs):
    valid_blobs = []

    for blob in blobs:
        valid_area = pillar_area_ok(
            blob,
            PILLARS_ROI
        )

        valid_shape = is_vertical_pillar(blob)

        horizontal_line = looks_like_horizontal_line(
            blob
        )

        if (
            valid_area
            and valid_shape
            and not horizontal_line
        ):
            valid_blobs.append(blob)

    return valid_blobs


def corner_dead_zone_margin(y_pixel):
    y_pixel = clamp(
        int(y_pixel),
        CORNER_DEAD_ZONE_TOP_Y,
        CORNER_DEAD_ZONE_BOTTOM_Y
    )

    y_span = max(
        1,
        CORNER_DEAD_ZONE_BOTTOM_Y - CORNER_DEAD_ZONE_TOP_Y
    )

    margin_span = (
        CORNER_DEAD_ZONE_NEAR_MARGIN_PX
        - CORNER_DEAD_ZONE_FAR_MARGIN_PX
    )

    return (
        CORNER_DEAD_ZONE_FAR_MARGIN_PX
        + (
            (y_pixel - CORNER_DEAD_ZONE_TOP_Y)
            * margin_span
        ) // y_span
    )


def blob_in_corner_dead_zone(blob, detected_id):
    if not USE_CORNER_DEAD_ZONES:
        return False

    if detected_id == 3:
        reference_x, reference_y = corner_lower_left(blob)
    else:
        reference_x, reference_y = corner_lower_right(blob)

    margin = corner_dead_zone_margin(reference_y)

    return (
        reference_x < margin
        or reference_x >= IMG_W - margin
    )


def remove_corner_dead_zone_blobs(blobs, detected_id):
    accepted = []

    for blob in blobs:
        if not blob_in_corner_dead_zone(blob, detected_id):
            accepted.append(blob)

    return accepted


def is_parking_wall(blob):
    if blob is None:
        return False

    if blob.area < MIN_PARKING_AREA:
        return False

    if blob.h < MIN_PARKING_H:
        return False

    if blob.w < MIN_PARKING_W:
        return False

    if PARKING_REJECT_ROI_TOP_EDGE:
        if blob.rect[1] <= PARKING_ROI[1] + 1:
            return False

    width_over_height = (
        blob.w
        / max(1, blob.h)
    )

    if width_over_height < MIN_W_OVER_H_PARKING:
        return False

    if blob_density_percent(blob) < MIN_PARKING_DENSITY_PERCENT:
        return False

    return True


def keep_valid_parking(blobs):
    accepted = []

    for blob in blobs:
        if is_parking_wall(blob):
            accepted.append(blob)

    return accepted


def parking_thresholds():
    if PARKING_USES_MAGENTA_THRESHOLD:
        return THS_RED + THS_MAGENTA

    return THS_RED


# Las paredes salen de la MISMA lista de blobs rojos que los pilares: son
# las anchas. Una sola busqueda sirve para las dos cosas, asi que la camara
# se queda en 3 llamadas a find_blobs por cuadro, las del script original.
def find_parking_wall(red_blobs_raw):
    if not DETECT_PARKING or not parking_detection_enabled:
        return None

    return best_blob_area(
        keep_valid_parking(red_blobs_raw)
    )


def find_red_green_pillars(img):
    # El rojo se busca sobre PARKING_ROI, que contiene por completo a
    # PILLARS_ROI: la misma pasada da los pilares (blobs altos) y las
    # paredes del estacionamiento (blobs anchos). Los blobs que caen por
    # encima de PILLARS_ROI se descartan solos mas adelante, cuando se
    # comprueba que su esquina caiga en ROI_HIGH o ROI_LOW.
    red_blobs = img.find_blobs(
        parking_thresholds(),
        roi=PARKING_ROI,
        pixels_threshold=PIX_TH,
        area_threshold=AREA_TH,
        merge=MERGE,
        margin=MARGIN
    ) or []

    green_blobs = img.find_blobs(
        THS_GREEN,
        roi=PILLARS_ROI,
        pixels_threshold=PIX_TH,
        area_threshold=AREA_TH,
        merge=MERGE,
        margin=MARGIN
    ) or []

    # Los blobs sin filtrar se devuelven para el estacionamiento: los que
    # keep_valid_pillars descarta por anchos son justo las paredes.
    red_blobs_raw = red_blobs

    red_blobs = keep_valid_pillars(red_blobs)
    green_blobs = keep_valid_pillars(green_blobs)

    red_blobs = remove_corner_dead_zone_blobs(
        red_blobs,
        5
    )
    green_blobs = remove_corner_dead_zone_blobs(
        green_blobs,
        3
    )

    selected_red = best_blob_area(red_blobs)
    selected_green = best_blob_area(green_blobs)

    return selected_red, selected_green, red_blobs_raw


def send_uart_payload(payload):
    message = "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n" % tuple(payload)

    uart.write(message.encode("ascii"))

    if PRINT_UART_DATA:
        print(
            "UART TX: %s"
            % message.strip()
        )


def reinitialize_uart():
    try:
        uart.init(
            UART_BAUD,
            bits=8,
            parity=None,
            stop=1,
            timeout_char=10
        )

    except Exception as error:
        print(
            "UART_INIT_ERR:",
            error
        )


def print_base64_frame(img):
    try:
        # copy=True evita comprimir la imagen original.
        jpeg_image = img.compress(
            quality=BASE64_JPEG_QUALITY,
            copy=True
        )

        encoded_frame = binascii.b2a_base64(
            bytes(jpeg_image)
        ).decode("utf-8").strip()

        print(
            "**FRAME**:"
            + encoded_frame
        )

    except Exception as error:
        print(
            "IMG_ERR:",
            error
        )


def parse_parking_command(fields):
    # "P,1" enciende la busqueda del estacionamiento; "P,0" la apaga.
    global parking_detection_enabled

    if len(fields) < 2:
        return

    try:
        enabled = int(fields[1]) != 0
    except Exception:
        return

    if enabled != parking_detection_enabled:
        print(
            "ESTACIONAMIENTO: deteccion "
            + ("ENCENDIDA" if enabled else "apagada")
        )

    parking_detection_enabled = enabled


def parse_telemetry_line(line):
    global telemetry_servo_deg
    global telemetry_motor_percent
    global telemetry_distances_mm
    global telemetry_last_ms

    fields = line.split(",")

    if fields[0] == "P":
        parse_parking_command(fields)
        return

    if len(fields) != 8 or fields[0] != "T":
        return

    try:
        servo_deg = int(fields[1])
        motor_percent = int(fields[2])
        distances_mm = [
            int(fields[3]),
            int(fields[4]),
            int(fields[5]),
            int(fields[6]),
            int(fields[7])
        ]
    except Exception:
        return

    telemetry_servo_deg = clamp(servo_deg, 0, 180)
    telemetry_motor_percent = clamp(motor_percent, -100, 100)
    telemetry_distances_mm = distances_mm
    telemetry_last_ms = time.ticks_ms()


def read_esp32_telemetry():
    global telemetry_rx_line

    try:
        available = uart.any()
        if not available:
            return

        incoming = uart.read(available)
        if not incoming:
            return

        for byte_value in incoming:
            if isinstance(byte_value, int):
                character = chr(byte_value)
            else:
                character = byte_value

            if character == "\n":
                if telemetry_rx_line:
                    parse_telemetry_line(telemetry_rx_line)
                telemetry_rx_line = ""
            elif character != "\r":
                if len(telemetry_rx_line) < 95:
                    telemetry_rx_line += character
                else:
                    telemetry_rx_line = ""

    except Exception as error:
        print("UART_RX_ERR:", error)


def trajectory_warning_state(now_ms):
    fresh = (
        telemetry_last_ms != 0
        and time.ticks_diff(now_ms, telemetry_last_ms)
        <= TELEMETRY_TIMEOUT_MS
    )

    if not fresh:
        return 0  # Sin telemetria: amarillo.

    correction = telemetry_servo_deg - SERVO_CENTER_DEG
    front_mm = telemetry_distances_mm[2]
    danger = 0 < front_mm <= PATH_FRONT_DANGER_MM

    if correction < -4:
        # Giro hacia la izquierda: S1/S2.
        side_mm = min(
            telemetry_distances_mm[0],
            telemetry_distances_mm[1]
        )
        danger = danger or 0 < side_mm <= PATH_SIDE_DANGER_MM
    elif correction > 4:
        # Giro hacia la derecha: S4/S5.
        side_mm = min(
            telemetry_distances_mm[3],
            telemetry_distances_mm[4]
        )
        danger = danger or 0 < side_mm <= PATH_SIDE_DANGER_MM

    return 2 if danger else 1


def trajectory_point(progress, lateral_offset_px):
    # progress=0: parte baja/frente del robot; progress=1: horizonte.
    correction = clamp(
        telemetry_servo_deg - SERVO_CENTER_DEG,
        -SERVO_MAX_CORRECTION_DEG,
        SERVO_MAX_CORRECTION_DEG
    )
    steering = correction / float(SERVO_MAX_CORRECTION_DEG)

    # Curva cuadratica: estable cerca del robot y mas visible a distancia.
    bend = (
        PATH_SCREEN_DIRECTION
        * steering
        * PATH_MAX_BEND_PX
        * progress
        * progress
    )

    half_width_scale = 1.0 - progress
    perspective_offset = (
        PATH_FAR_HALF_WIDTH_PX
        + (
            PATH_NEAR_HALF_WIDTH_PX
            - PATH_FAR_HALF_WIDTH_PX
        ) * half_width_scale
    )

    x = (IMG_W // 2) + bend + lateral_offset_px * perspective_offset
    y = PATH_NEAR_Y + (PATH_FAR_Y - PATH_NEAR_Y) * progress

    return (
        clamp(int(x), 0, IMG_W - 1),
        clamp(int(y), 0, IMG_H - 1)
    )


def draw_trajectory_preview(img):
    now_ms = time.ticks_ms()
    warning_state = trajectory_warning_state(now_ms)

    if warning_state == 2:
        path_color = (255, 40, 0)
        state_text = "RIESGO"
    elif warning_state == 1:
        path_color = (0, 255, 255)
        state_text = "RUTA OK"
    else:
        path_color = (255, 220, 0)
        state_text = "SIN TELEMETRIA"

    previous_left = trajectory_point(0.0, -1.0)
    previous_center = trajectory_point(0.0, 0.0)
    previous_right = trajectory_point(0.0, 1.0)

    for index in range(1, PATH_SEGMENTS + 1):
        progress = index / float(PATH_SEGMENTS)
        current_left = trajectory_point(progress, -1.0)
        current_center = trajectory_point(progress, 0.0)
        current_right = trajectory_point(progress, 1.0)

        img.draw_line(
            previous_left + current_left,
            color=path_color,
            thickness=2
        )
        img.draw_line(
            previous_right + current_right,
            color=path_color,
            thickness=2
        )
        img.draw_line(
            previous_center + current_center,
            color=path_color,
            thickness=3
        )

        if (index % 3) == 0:
            img.draw_line(
                current_left + current_right,
                color=path_color,
                thickness=1
            )

        previous_left = current_left
        previous_center = current_center
        previous_right = current_right

    img.draw_string(
        (196, 2),
        state_text,
        color=path_color
    )
    img.draw_string(
        (196, 12),
        "S:%d M:%d%%" % (
            telemetry_servo_deg,
            telemetry_motor_percent
        ),
        color=path_color
    )


# ============================================================
# PROGRAMA PRINCIPAL
# ============================================================

while True:
    clock.tick()

    # UART es full-duplex: se recibe la orden aplicada por el ESP32 sin
    # interrumpir las tramas de vision que se envian mas abajo.
    read_esp32_telemetry()

    img = sensor.snapshot()

    # Sensor optico frontal: porcentaje real de pixeles negros dentro del ROI.
    # Esta salida no decide por si sola una esquina; el ESP32 exige tambien
    # que S3 confirme la distancia configurada.
    center_roi_stats = img.get_statistics(roi=CENTER_ROI)
    center_roi_l_mean = int(center_roi_stats.l_mean)
    center_roi_black_blobs = img.find_blobs(
        [(0, WALL_BLACK_L_MAX, -128, 127, -128, 127)],
        roi=CENTER_ROI,
        pixels_threshold=1,
        area_threshold=1,
        merge=False
    ) or []
    center_roi_black_pixels = 0
    for black_blob in center_roi_black_blobs:
        center_roi_black_pixels += int(black_blob.pixels)
    center_roi_total_pixels = CENTER_ROI_WIDTH * CENTER_ROI_HEIGHT
    center_roi_black_percent = (
        center_roi_black_pixels * 100
    ) // center_roi_total_pixels
    wall_black_detected = (
        1
        if center_roi_black_percent >= WALL_BLACK_MIN_PERCENT
        else 0
    )

    # --------------------------------------------------------
    # DETECCIÓN DE PILARES
    # --------------------------------------------------------

    red_blob, green_blob, red_blobs_raw = find_red_green_pillars(
        img
    )

    # Las paredes son los blobs ANCHOS de esa misma lista, justo los que
    # la deteccion de pilares descarta.
    parking_blob = find_parking_wall(red_blobs_raw)

    parking_detected = 1 if parking_blob else 0
    parking_x = 0
    parking_area = 0

    if parking_blob:
        parking_center_x = int(
            parking_blob.rect[0] + parking_blob.rect[2] // 2
        )

        if X_AS_NORMALIZED:
            parking_x = normalize_x100(parking_center_x)
        else:
            parking_x = parking_center_x

        parking_area = int(parking_blob.area)

    selected_blob = None
    detected_id = 0

    # ID 3 = verde
    # ID 5 = rojo

    if green_blob and (
        not red_blob
        or green_blob.area >= red_blob.area
    ):
        selected_blob = green_blob
        detected_id = 3

    elif red_blob:
        selected_blob = red_blob
        detected_id = 5

    roi_code = 0
    area_pixels = 0
    threshold_index = 0

    # Solo para la pantalla: permiten ajustar MIN_PILLAR_DENSITY_PERCENT y
    # comprobar que convencion usa blob.rotation en esta camara.
    selected_density_percent = 0
    selected_rotation_degrees = 0

    x_reference = CORNER_SENTINEL
    y_reference = CORNER_SENTINEL

    # Coordenadas en pixeles usadas tambien para dibujar exactamente
    # la referencia enviada por UART.
    reference_x_pixel = CORNER_SENTINEL
    reference_y_pixel = CORNER_SENTINEL

    # --------------------------------------------------------
    # CÁLCULO DE LA ESQUINA
    # --------------------------------------------------------

    if selected_blob:

        if detected_id == 3:
            edge_x, lower_y = corner_lower_left(
                selected_blob
            )

        else:
            edge_x, lower_y = corner_lower_right(
                selected_blob
            )

        inside_low = point_in_roi(
            edge_x,
            lower_y,
            ROI_LOW
        )

        inside_high = point_in_roi(
            edge_x,
            lower_y,
            ROI_HIGH
        )

        if inside_low or inside_high:

            if inside_high:
                roi_code = 1
            else:
                roi_code = 2

            # La referencia no cambia al pasar de HIGH a LOW.
            reference_x_pixel = edge_x
            reference_y_pixel = lower_y

            area_pixels = int(
                selected_blob.area
            )

            threshold_index = blob_threshold_index(
                selected_blob
            )

            selected_density_percent = blob_density_percent(
                selected_blob
            )

            try:
                selected_rotation_degrees = int(
                    abs(
                        radians_to_degrees(
                            selected_blob.rotation
                        )
                    )
                    % 180.0
                )
            except Exception:
                selected_rotation_degrees = 0

            if X_AS_NORMALIZED:
                x_reference = normalize_x100(
                    reference_x_pixel
                )
            else:
                x_reference = reference_x_pixel

            y_reference = reference_y_pixel

        else:
            detected_id = 0
            x_reference = CORNER_SENTINEL
            y_reference = CORNER_SENTINEL
            area_pixels = 0
            roi_code = 0
            threshold_index = 0

    # --------------------------------------------------------
    # DETECCIÓN DE COLISIÓN
    # --------------------------------------------------------

    # El pilar ya paso los filtros de area, forma vertical e inclinacion.
    # La colision usa su misma esquina de referencia; una linea del piso no
    # puede activar esta salida porque no es un selected_blob vertical valido.
    collision_id = 0
    if (
        selected_blob
        and detected_id in (3, 5)
        and roi_code == 2
        and reference_x_pixel != CORNER_SENTINEL
        and point_in_roi(
            reference_x_pixel,
            reference_y_pixel,
            COLLISION_ROI
        )
    ):
        collision_id = detected_id

    # --------------------------------------------------------
    # DATOS ENVIADOS POR UART
    #
    # 0: ID del color
    # 1: X de la esquina
    # 2: Y de la esquina
    # 3: Área del blob
    # 4: Código de ROI
    # 5: ID de colisión
    # 6: Pared negra en ROI central (0/1)
    # 7: Pared de estacionamiento a la vista (0/1)
    # 8: X del centro de esa pared
    # 9: Área de esa pared
    # --------------------------------------------------------

    data_to_send = [
        clamp_i16(detected_id),
        clamp_i16(x_reference),
        clamp_i16(y_reference),
        clamp_i16(area_pixels),
        clamp_i16(roi_code),
        clamp_i16(collision_id),
        clamp_i16(wall_black_detected),
        clamp_i16(parking_detected),
        clamp_i16(parking_x),
        clamp_i16(parking_area)
    ]

    try:
        send_uart_payload(
            data_to_send
        )

    except Exception as error:
        print(
            "UART_ERR:",
            error
        )

        reinitialize_uart()

    # --------------------------------------------------------
    # DIBUJOS DE DEPURACIÓN
    # --------------------------------------------------------

    if SHOW_DEBUG:
        img.draw_rectangle(
            ROI_LOW,
            color=(255, 255, 0)
        )

        img.draw_rectangle(
            ROI_HIGH,
            color=(255, 200, 0)
        )

        img.draw_rectangle(
            PILLARS_ROI,
            color=(0, 200, 255)
        )

        if DETECT_PARKING:
            img.draw_rectangle(
                PARKING_ROI,
                color=(180, 0, 180)
            )

        img.draw_rectangle(
            COLLISION_ROI,
            color=(0, 255, 255)
        )

        if SHOW_CENTER_ROI:
            img.draw_rectangle(
                CENTER_ROI,
                color=(
                    (0, 255, 0)
                    if wall_black_detected
                    else (255, 0, 255)
                )
            )

        if USE_CORNER_DEAD_ZONES:
            # Bordes interiores de las dos zonas muertas trapezoidales.
            img.draw_line(
                (
                    CORNER_DEAD_ZONE_FAR_MARGIN_PX,
                    CORNER_DEAD_ZONE_TOP_Y,
                    CORNER_DEAD_ZONE_NEAR_MARGIN_PX,
                    CORNER_DEAD_ZONE_BOTTOM_Y
                ),
                color=(255, 40, 40),
                thickness=2
            )
            img.draw_line(
                (
                    IMG_W - 1 - CORNER_DEAD_ZONE_FAR_MARGIN_PX,
                    CORNER_DEAD_ZONE_TOP_Y,
                    IMG_W - 1 - CORNER_DEAD_ZONE_NEAR_MARGIN_PX,
                    CORNER_DEAD_ZONE_BOTTOM_Y
                ),
                color=(255, 40, 40),
                thickness=2
            )

        if DRAW_BOXES:

            if green_blob:
                img.draw_rectangle(
                    green_blob.rect,
                    color=(0, 255, 0)
                )

            if red_blob:
                img.draw_rectangle(
                    red_blob.rect,
                    color=(255, 0, 0)
                )

            if parking_blob:
                img.draw_rectangle(
                    parking_blob.rect,
                    color=(255, 0, 255),
                    thickness=2
                )

        # ----------------------------------------------------
        # CRUZ DE LA REFERENCIA ENVIADA
        # HIGH y LOW conservan la misma esquina original.
        # ----------------------------------------------------

        if (
            detected_id != 0
            and roi_code != 0
            and reference_x_pixel != CORNER_SENTINEL
        ):
            img.draw_cross(
                (
                    int(reference_x_pixel),
                    int(reference_y_pixel)
                ),
                color=(
                    (0, 255, 0)
                    if detected_id == 3
                    else (255, 0, 0)
                ),
                size=5,
                thickness=1
            )

        # ----------------------------------------------------
        # TEXTO EN PANTALLA
        # ----------------------------------------------------

        if DRAW_TEXT:
            text_y = 2

            if X_AS_NORMALIZED:
                x_mode_text = "X100"
            else:
                x_mode_text = "X"

            img.draw_rectangle(
                (0, 0, 320, 70),
                color=(0, 0, 0),
                fill=True
            )

            img.draw_string(
                (2, text_y),
                "FPS: %.1f" % clock.fps(),
                color=(255, 255, 255)
            )

            text_y += 10

            x_show = data_to_send[1]
            y_show = data_to_send[2]

            if x_show == CORNER_SENTINEL:
                x_string = "--"
            else:
                x_string = "%4d" % x_show

            if y_show == CORNER_SENTINEL:
                y_string = "--"
            else:
                y_string = "%3d" % y_show

            img.draw_string(
                (2, text_y),
                "ID:%d %s:%s Y:%s"
                % (
                    data_to_send[0],
                    x_mode_text,
                    x_string,
                    y_string
                ),
                color=(255, 255, 255)
            )

            text_y += 10

            if data_to_send[4] == 1:
                roi_string = "HIGH"
            elif data_to_send[4] == 2:
                roi_string = "LOW"
            else:
                roi_string = "--"

            img.draw_string(
                (2, text_y),
                "Area:%d D:%d%% R:%d ROI:%s TH:%d"
                % (
                    data_to_send[3],
                    selected_density_percent,
                    selected_rotation_degrees,
                    roi_string,
                    threshold_index
                ),
                color=(255, 255, 255)
            )

            text_y += 10

            img.draw_string(
                (2, text_y),
                "CollisionID:%d W:%d B:%d%% L:%d"
                % (
                    collision_id,
                    wall_black_detected,
                    center_roi_black_percent,
                    center_roi_l_mean
                ),
                color=(255, 255, 255)
            )

            text_y += 10

            if parking_blob:
                parking_ratio_x10 = (
                    int(parking_blob.w) * 10
                ) // max(1, int(parking_blob.h))

                parking_string = (
                    "Park:SI X:%d A:%d W/H:%d.%d D:%d%%"
                    % (
                        parking_x,
                        parking_area,
                        parking_ratio_x10 // 10,
                        parking_ratio_x10 % 10,
                        blob_density_percent(parking_blob)
                    )
                )
            elif not parking_detection_enabled:
                parking_string = "Park:OFF (esperando al ESP32)"
            else:
                parking_string = "Park:--"

            img.draw_string(
                (2, text_y),
                parking_string,
                color=(255, 120, 255)
            )

        # Se dibuja al final para que la trayectoria sea visible sobre las
        # ROIs y cajas de depuracion.
        if SHOW_PATH_PREVIEW:
            draw_trajectory_preview(img)

    # --------------------------------------------------------
    # FPS EN LA TERMINAL
    # --------------------------------------------------------

    if PRINT_FPS:
        print(
            "FPS: %.1f"
            % clock.fps()
        )

    # --------------------------------------------------------
    # IMAGEN BASE64
    #
    # Siempre debe ejecutarse al final, después de detectar
    # y dibujar.
    # --------------------------------------------------------

    if PRINT_BASE64_FRAME:
        print_base64_frame(
            img
        )
