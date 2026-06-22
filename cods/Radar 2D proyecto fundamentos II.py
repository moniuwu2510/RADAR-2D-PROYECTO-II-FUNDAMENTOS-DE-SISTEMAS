import datetime
import math
import time
import tkinter as tk
from collections import deque

# Intenta cargar pyserial para hablar con el Arduino real.
try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


# Configuracion base de la conexion serial.
PUERTO_SERIAL_PREFERIDO = "COM3"
BAUDIOS = 9600
SERIAL_TIMEOUT = 0.05
ESPERA_ARRANQUE_ARDUINO_S = 2
INTERVALO_ACTUALIZACION_MS = 50
INTERVALO_RECONEXION_S = 3
MAX_LECTURAS_POR_TICK = 200

# Parametros del radar, filtros y seguimiento.
DISTANCIA_MAXIMA_CM = 100.0
DISTANCIA_MINIMA_CM = 0.0
UMBRAL_OBJETO_CM = 92.0
SALTO_ANGULAR_MAXIMO = 12.0
SALTO_DISTANCIA_MAXIMO = 18.0
MIN_MUESTRAS_POR_OBJETO = 2
MAX_OBJETOS_DETECTADOS = 3
MAX_OBJETOS_PANEL = 3
UMBRAL_ASOCIACION_CM = 25.0
HORIZONTE_PREDICCION_S = 0.8
AMORTIGUACION_ACELERACION = 0.35
MAX_DESPLAZAMIENTO_PREDICCION_CM = 35.0

HISTORIAL_PUNTOS_MAXIMO = 240
HISTORIAL_TRACK_MAXIMO = 6
PASO_DEMO_ANGULAR = 5.625


# Estado global de la conexion actual.
arduino = None
modo_demo = True
estado_conexion = "Buscando Arduino..."
puerto_conectado = "DEMO"
ultimo_intento_reconexion = 0.0
arduino_listo_en = 0.0

# Estado global del radar y de los objetos detectados.
historial_puntos = deque(maxlen=HISTORIAL_PUNTOS_MAXIMO)
objetos_detectados = []
velocidad_promedio = 0.0
angulo_actual = 0.0

muestras_barrido_actual = []
ultimo_angulo_leido = None
angulo_demo = 0.0


# Mantiene cualquier angulo dentro del rango 0-359.
def normalizar_angulo(angulo):
    return float(angulo) % 360.0


# Invierte el giro visual para que el radar se vea al lado opuesto.
def angulo_para_vista(angulo):
    return normalizar_angulo(-angulo)


# Calcula la menor separacion entre dos angulos.
def distancia_angular(angulo_a, angulo_b):
    diferencia = abs(normalizar_angulo(angulo_a) - normalizar_angulo(angulo_b))
    return min(diferencia, 360.0 - diferencia)


# Convierte una medicion polar a coordenadas cartesianas.
def polares_a_cartesianas(radio, angulo):
    theta = math.radians(angulo)
    return radio * math.cos(theta), radio * math.sin(theta)


# Convierte coordenadas cartesianas de vuelta a polar.
def cartesianas_a_polares(x, y):
    distancia = math.hypot(x, y)
    angulo = normalizar_angulo(math.degrees(math.atan2(y, x)))
    return distancia, angulo


# Lleva una lectura al sistema de coordenadas del canvas.
def coordenadas_canvas(distancia, angulo, centro_x, centro_y, radio_max):
    escala = radio_max / DISTANCIA_MAXIMA_CM
    x, y = polares_a_cartesianas(distancia, angulo_para_vista(angulo))
    return centro_x + x * escala, centro_y - y * escala


# Calcula la velocidad lineal entre dos posiciones.
def calcular_velocidad(posicion_anterior, posicion_nueva, tiempo_anterior, tiempo_actual):
    delta_tiempo = tiempo_actual - tiempo_anterior
    if delta_tiempo <= 0:
        return 0.0

    distancia = math.hypot(
        posicion_nueva[0] - posicion_anterior[0],
        posicion_nueva[1] - posicion_anterior[1],
    )
    return distancia / delta_tiempo


# Promedia angulos sin fallar cerca del 0/360.
def promedio_angular(angulos):
    if not angulos:
        return 0.0

    suma_x = sum(math.cos(math.radians(angulo)) for angulo in angulos)
    suma_y = sum(math.sin(math.radians(angulo)) for angulo in angulos)
    return normalizar_angulo(math.degrees(math.atan2(suma_y, suma_x)))


# Reserva siempre IDs dentro del rango 1..3.
def obtener_id_disponible(ids_en_uso):
    for identificador in range(1, MAX_OBJETOS_DETECTADOS + 1):
        if identificador not in ids_en_uso:
            return identificador
    return 1


# Crea una lista priorizada de puertos seriales probables.
def obtener_puertos_candidatos():
    if serial is None or list_ports is None:
        return []

    puertos = list(list_ports.comports())
    candidatos = []

    if PUERTO_SERIAL_PREFERIDO:
        candidatos.append(PUERTO_SERIAL_PREFERIDO)

    palabras_clave = ("arduino", "wch", "ch340", "cp210", "usb serial", "serial")

    # Primero intenta puertos que parezcan adaptadores USB-serial comunes.
    for puerto in puertos:
        descripcion = " ".join(
            valor
            for valor in [puerto.device, puerto.description or "", puerto.manufacturer or ""]
            if valor
        ).lower()

        if puerto.device not in candidatos and any(clave in descripcion for clave in palabras_clave):
            candidatos.append(puerto.device)

    # Luego agrega cualquier otro puerto como respaldo.
    for puerto in puertos:
        if puerto.device not in candidatos:
            candidatos.append(puerto.device)

    return candidatos


# Refresca el panel lateral segun el estado de la conexion.
def refrescar_estado_conexion():
    if "conexion_label" not in globals():
        return

    if serial is None:
        conexion_label.config(text="PYSERIAL FALTA", fg="#FF5555")
    elif modo_demo:
        conexion_label.config(text="MODO DEMO", fg="#FFAA00")
    else:
        conexion_label.config(text="CONECTADO", fg="#00FF66")

    puerto_label.config(text=puerto_conectado)
    estado_label.config(text=estado_conexion)


# Cierra el puerto serial actual de forma segura.
def cerrar_arduino():
    global arduino, arduino_listo_en

    if arduino is not None:
        try:
            arduino.close()
        except Exception:
            pass
        arduino = None
    arduino_listo_en = 0.0


# Intenta conectar con el Arduino y pasar a modo real.
def conectar_arduino():
    global arduino, modo_demo, estado_conexion, puerto_conectado, ultimo_intento_reconexion, arduino_listo_en

    ultimo_intento_reconexion = time.time()

    if serial is None or list_ports is None:
        modo_demo = True
        estado_conexion = "Instale pyserial con: py -m pip install pyserial"
        puerto_conectado = "DEMO"
        refrescar_estado_conexion()
        return False

    cerrar_arduino()
    ultimo_error = "No se encontro un puerto serial utilizable"

    for puerto in obtener_puertos_candidatos():
        try:
            conexion = serial.Serial(puerto, BAUDIOS, timeout=SERIAL_TIMEOUT)
            arduino = conexion
            # Al abrir el puerto el Arduino suele reiniciarse.
            arduino_listo_en = time.time() + ESPERA_ARRANQUE_ARDUINO_S
            modo_demo = False
            puerto_conectado = puerto
            estado_conexion = f"Conectado a {puerto}. Esperando reinicio del Arduino..."
            refrescar_estado_conexion()
            return True
        except Exception as error:
            ultimo_error = f"{puerto}: {error}"

    modo_demo = True
    estado_conexion = f"Sin conexion con Arduino. {ultimo_error}"
    puerto_conectado = "DEMO"
    refrescar_estado_conexion()
    return False


# Espera a que el Arduino termine de reiniciar antes de leer.
def arduino_esta_listo_para_leer():
    global modo_demo, estado_conexion, arduino_listo_en, puerto_conectado

    if arduino is None:
        return False

    if arduino_listo_en == 0.0:
        return True

    if time.time() < arduino_listo_en:
        return False

    try:
        arduino.reset_input_buffer()
    except Exception as error:
        cerrar_arduino()
        modo_demo = True
        puerto_conectado = "DEMO"
        estado_conexion = f"Error preparando serial: {error}"
        refrescar_estado_conexion()
        return False

    arduino_listo_en = 0.0
    estado_conexion = f"Recibiendo datos desde {puerto_conectado}"
    refrescar_estado_conexion()
    return True


# Genera lecturas simuladas cuando no hay hardware conectado.
def generar_datos_demo():
    global angulo_demo

    tiempo_actual = time.time()
    angulo = angulo_demo

    # Tres objetos ficticios para probar el tracking.
    objetos_demo = [
        {
            "angulo": normalizar_angulo(45 + 8 * math.sin(tiempo_actual * 0.5)),
            "distancia": 30 + 4 * math.sin(tiempo_actual * 0.8),
        },
        {
            "angulo": normalizar_angulo(155 + 10 * math.sin(tiempo_actual * 0.35)),
            "distancia": 58 + 6 * math.cos(tiempo_actual * 0.6),
        },
        {
            "angulo": normalizar_angulo(285 + 12 * math.cos(tiempo_actual * 0.4)),
            "distancia": 42 + 5 * math.sin(tiempo_actual * 0.9),
        },
    ]

    distancia = DISTANCIA_MAXIMA_CM
    for objeto in objetos_demo:
        diferencia = distancia_angular(angulo, objeto["angulo"])
        if diferencia <= 10:
            distancia = min(distancia, objeto["distancia"] + diferencia * 1.4)

    angulo_demo = normalizar_angulo(angulo_demo + PASO_DEMO_ANGULAR)
    distancia = max(5.0, min(DISTANCIA_MAXIMA_CM, distancia))
    return distancia, angulo


# Lee varias muestras del Arduino en cada ciclo de GUI.
def leer_datos_arduino():
    global modo_demo, estado_conexion, puerto_conectado

    if arduino is None:
        if time.time() - ultimo_intento_reconexion >= INTERVALO_RECONEXION_S:
            conectar_arduino()

        if arduino is None:
            return [generar_datos_demo()]

    if not arduino_esta_listo_para_leer():
        return []

    lecturas = []

    try:
        # Consume el buffer rapido para que el radar no se atrase.
        while arduino.in_waiting > 0 and len(lecturas) < MAX_LECTURAS_POR_TICK:
            linea = arduino.readline().decode("utf-8", errors="ignore").strip()
            if not linea:
                continue

            partes = [parte.strip() for parte in linea.split(",")]
            if len(partes) != 2:
                continue

            distancia = float(partes[0])
            angulo = normalizar_angulo(float(partes[1]))
            distancia = max(DISTANCIA_MINIMA_CM, min(DISTANCIA_MAXIMA_CM, distancia))
            lecturas.append((distancia, angulo))
    except Exception as error:
        cerrar_arduino()
        modo_demo = True
        puerto_conectado = "DEMO"
        estado_conexion = f"Error serial: {error}"
        refrescar_estado_conexion()
        return [generar_datos_demo()]

    return lecturas


# Resume un grupo de muestras como un solo objeto candidato.
def construir_objeto_desde_grupo(grupo):
    angulos = [muestra["angulo"] for muestra in grupo]
    distancias = [muestra["distancia"] for muestra in grupo]

    angulo = promedio_angular(angulos)
    distancia = sum(distancias) / len(distancias)
    x, y = polares_a_cartesianas(distancia, angulo)

    return {
        "angulo": angulo,
        "distancia": distancia,
        "x": x,
        "y": y,
        "muestras": len(grupo),
        "ancho_angular": distancia_angular(angulos[0], angulos[-1]),
    }


# Agrupa muestras cercanas para estimar objetos en un barrido.
def agrupar_objetos_en_barrido(muestras):
    grupos = []
    grupo_actual = []

    for muestra in muestras:
        # Las distancias lejanas se interpretan como fondo libre.
        if muestra["distancia"] >= UMBRAL_OBJETO_CM:
            if len(grupo_actual) >= MIN_MUESTRAS_POR_OBJETO:
                grupos.append(grupo_actual)
            grupo_actual = []
            continue

        if not grupo_actual:
            grupo_actual = [muestra]
            continue

        muestra_anterior = grupo_actual[-1]
        if (
            distancia_angular(muestra["angulo"], muestra_anterior["angulo"]) <= SALTO_ANGULAR_MAXIMO
            and abs(muestra["distancia"] - muestra_anterior["distancia"]) <= SALTO_DISTANCIA_MAXIMO
        ):
            grupo_actual.append(muestra)
        else:
            if len(grupo_actual) >= MIN_MUESTRAS_POR_OBJETO:
                grupos.append(grupo_actual)
            grupo_actual = [muestra]

    if len(grupo_actual) >= MIN_MUESTRAS_POR_OBJETO:
        grupos.append(grupo_actual)

    # Une extremos cuando el mismo objeto cruza el corte 359 -> 0 grados.
    if len(grupos) > 1:
        primero = grupos[0][0]
        ultimo = grupos[-1][-1]
        if (
            distancia_angular(primero["angulo"], ultimo["angulo"]) <= SALTO_ANGULAR_MAXIMO
            and abs(primero["distancia"] - ultimo["distancia"]) <= SALTO_DISTANCIA_MAXIMO
        ):
            grupos[0] = grupos[-1] + grupos[0]
            grupos.pop()

    candidatos = [construir_objeto_desde_grupo(grupo) for grupo in grupos]

    # Prioriza agrupaciones mas estables para evitar ruido cuando aparezcan
    # mas de 3 candidatos en un barrido.
    candidatos.sort(key=lambda objeto: (-objeto["muestras"], objeto["distancia"]))
    return candidatos[:MAX_OBJETOS_DETECTADOS]


# Estima velocidad y aceleracion usando varias muestras recientes.
def estimar_movimiento(historial_track):
    if len(historial_track) < 2:
        return None

    segmentos = []

    for indice in range(1, len(historial_track)):
        anterior = historial_track[indice - 1]
        actual = historial_track[indice]
        delta_tiempo = actual["tiempo"] - anterior["tiempo"]
        if delta_tiempo <= 0:
            continue

        segmentos.append(
            {
                "vx": (actual["x"] - anterior["x"]) / delta_tiempo,
                "vy": (actual["y"] - anterior["y"]) / delta_tiempo,
                "dt": delta_tiempo,
            }
        )

    if not segmentos:
        return None

    pesos_segmentos = list(range(1, len(segmentos) + 1))
    suma_pesos = sum(pesos_segmentos)
    velocidad_x = sum(segmento["vx"] * peso for segmento, peso in zip(segmentos, pesos_segmentos)) / suma_pesos
    velocidad_y = sum(segmento["vy"] * peso for segmento, peso in zip(segmentos, pesos_segmentos)) / suma_pesos

    aceleracion_x = 0.0
    aceleracion_y = 0.0
    if len(segmentos) >= 2:
        aceleraciones = []
        for indice in range(1, len(segmentos)):
            segmento_anterior = segmentos[indice - 1]
            segmento_actual = segmentos[indice]
            delta_tiempo = max(0.001, (segmento_anterior["dt"] + segmento_actual["dt"]) / 2.0)
            aceleraciones.append(
                {
                    "ax": (segmento_actual["vx"] - segmento_anterior["vx"]) / delta_tiempo,
                    "ay": (segmento_actual["vy"] - segmento_anterior["vy"]) / delta_tiempo,
                }
            )

        pesos_aceleracion = list(range(1, len(aceleraciones) + 1))
        suma_pesos_aceleracion = sum(pesos_aceleracion)
        aceleracion_x = (
            sum(cambio["ax"] * peso for cambio, peso in zip(aceleraciones, pesos_aceleracion))
            / suma_pesos_aceleracion
        ) * AMORTIGUACION_ACELERACION
        aceleracion_y = (
            sum(cambio["ay"] * peso for cambio, peso in zip(aceleraciones, pesos_aceleracion))
            / suma_pesos_aceleracion
        ) * AMORTIGUACION_ACELERACION

    return {
        "velocidad_x": velocidad_x,
        "velocidad_y": velocidad_y,
        "velocidad": math.hypot(velocidad_x, velocidad_y),
        "aceleracion_x": aceleracion_x,
        "aceleracion_y": aceleracion_y,
    }


# Proyecta la siguiente posicion usando movimiento suavizado.
def predecir_posicion(historial_track):
    if len(historial_track) < 2:
        return None

    movimiento = estimar_movimiento(historial_track)
    if movimiento is None:
        return None

    ultimo = historial_track[-1]
    desplazamiento_x = (
        movimiento["velocidad_x"] * HORIZONTE_PREDICCION_S
        + 0.5 * movimiento["aceleracion_x"] * (HORIZONTE_PREDICCION_S ** 2)
    )
    desplazamiento_y = (
        movimiento["velocidad_y"] * HORIZONTE_PREDICCION_S
        + 0.5 * movimiento["aceleracion_y"] * (HORIZONTE_PREDICCION_S ** 2)
    )

    desplazamiento_total = math.hypot(desplazamiento_x, desplazamiento_y)
    if desplazamiento_total > MAX_DESPLAZAMIENTO_PREDICCION_CM:
        factor_ajuste = MAX_DESPLAZAMIENTO_PREDICCION_CM / desplazamiento_total
        desplazamiento_x *= factor_ajuste
        desplazamiento_y *= factor_ajuste

    x_predicho = ultimo["x"] + desplazamiento_x
    y_predicho = ultimo["y"] + desplazamiento_y

    distancia, angulo = cartesianas_a_polares(x_predicho, y_predicho)
    distancia = max(DISTANCIA_MINIMA_CM, min(DISTANCIA_MAXIMA_CM, distancia))
    x_ajustado, y_ajustado = polares_a_cartesianas(distancia, angulo)

    return {
        "x": x_ajustado,
        "y": y_ajustado,
        "distancia": distancia,
        "angulo": angulo,
        "velocidad": movimiento["velocidad"],
    }


# Asocia candidatos nuevos con tracks anteriores.
def actualizar_tracks_desde_barrido(candidatos, tiempo_actual):
    global objetos_detectados, velocidad_promedio

    tracks_previos = objetos_detectados
    tracks_disponibles = set(range(len(tracks_previos)))
    tracks_actualizados = []
    ids_en_uso = set()

    for candidato in candidatos:
        mejor_indice = None
        mejor_distancia = None

        for indice in tracks_disponibles:
            track = tracks_previos[indice]
            distancia_cartesiana = math.hypot(
                candidato["x"] - track["x"],
                candidato["y"] - track["y"],
            )

            # Solo enlaza objetos si siguen relativamente cerca.
            if distancia_cartesiana > UMBRAL_ASOCIACION_CM:
                continue

            if mejor_distancia is None or distancia_cartesiana < mejor_distancia:
                mejor_distancia = distancia_cartesiana
                mejor_indice = indice

        if mejor_indice is not None:
            # Actualiza un track ya existente.
            track_previo = tracks_previos[mejor_indice]
            tracks_disponibles.remove(mejor_indice)
            ids_en_uso.add(track_previo["id"])

            historial_track = deque(track_previo["historial"], maxlen=HISTORIAL_TRACK_MAXIMO)
            historial_track.append(
                {
                    "x": candidato["x"],
                    "y": candidato["y"],
                    "distancia": candidato["distancia"],
                    "angulo": candidato["angulo"],
                    "tiempo": tiempo_actual,
                }
            )
            movimiento = estimar_movimiento(historial_track)
            velocidad = movimiento["velocidad"] if movimiento is not None else 0.0
            prediccion = predecir_posicion(historial_track)

            tracks_actualizados.append(
                {
                    "id": track_previo["id"],
                    "distancia": candidato["distancia"],
                    "angulo": candidato["angulo"],
                    "x": candidato["x"],
                    "y": candidato["y"],
                    "velocidad": velocidad,
                    "prediccion": prediccion,
                    "historial": historial_track,
                    "muestras": candidato["muestras"],
                    "ancho_angular": candidato["ancho_angular"],
                }
            )
        else:
            # Crea un track nuevo cuando no hay coincidencia.
            nuevo_id = obtener_id_disponible(ids_en_uso)
            ids_en_uso.add(nuevo_id)
            historial_track = deque(maxlen=HISTORIAL_TRACK_MAXIMO)
            historial_track.append(
                {
                    "x": candidato["x"],
                    "y": candidato["y"],
                    "distancia": candidato["distancia"],
                    "angulo": candidato["angulo"],
                    "tiempo": tiempo_actual,
                }
            )

            tracks_actualizados.append(
                {
                    "id": nuevo_id,
                    "distancia": candidato["distancia"],
                    "angulo": candidato["angulo"],
                    "x": candidato["x"],
                    "y": candidato["y"],
                    "velocidad": 0.0,
                    "prediccion": None,
                    "historial": historial_track,
                    "muestras": candidato["muestras"],
                    "ancho_angular": candidato["ancho_angular"],
                }
            )

    tracks_actualizados.sort(
        key=lambda objeto: (-objeto["muestras"], objeto["distancia"])
    )
    objetos_detectados = sorted(
        tracks_actualizados[:MAX_OBJETOS_DETECTADOS],
        key=lambda objeto: objeto["angulo"],
    )
    velocidades = [objeto["velocidad"] for objeto in objetos_detectados]
    velocidad_promedio = sum(velocidades) / len(velocidades) if velocidades else 0.0


# Cierra el barrido actual y recalcula los objetos detectados.
def procesar_barrido_completo(tiempo_actual):
    candidatos = agrupar_objetos_en_barrido(muestras_barrido_actual)
    actualizar_tracks_desde_barrido(candidatos, tiempo_actual)


# Guarda la lectura y detecta cuando termina una vuelta completa.
def registrar_lectura(distancia, angulo, tiempo_actual):
    global ultimo_angulo_leido, muestras_barrido_actual

    # Solo guarda puntos visibles en el historial brillante del radar.
    if distancia < UMBRAL_OBJETO_CM:
        historial_puntos.append(
            {
                "distancia": distancia,
                "angulo": angulo,
                "tiempo": tiempo_actual,
            }
        )

    # Un salto grande hacia atras indica que arranco un nuevo barrido.
    if ultimo_angulo_leido is not None and angulo < ultimo_angulo_leido - 180:
        if muestras_barrido_actual:
            procesar_barrido_completo(tiempo_actual)
        muestras_barrido_actual = []

    muestras_barrido_actual.append(
        {
            "distancia": distancia,
            "angulo": angulo,
            "tiempo": tiempo_actual,
        }
    )
    ultimo_angulo_leido = angulo


# Actualiza las etiquetas del panel lateral.
def actualizar_panel_objetos(ultima_distancia, ultimo_angulo):
    distancia_label.config(text=f"Distancia: {ultima_distancia:.0f} cm")
    angulo_label.config(text=f"Angulo: {angulo_para_vista(ultimo_angulo):.1f} deg")

    if objetos_detectados:
        # Resalta el objeto mas cercano como referencia principal.
        objeto_principal = min(objetos_detectados, key=lambda objeto: objeto["distancia"])
        prediccion = objeto_principal["prediccion"]

        velocidad_label.config(text=f"Velocidad: {objeto_principal['velocidad']:.1f} cm/s")

        if prediccion is not None:
            futura_label.config(
                text=(
                    f"Futuro: {prediccion['distancia']:.0f} cm "
                    f"@ {angulo_para_vista(prediccion['angulo']):.0f} deg"
                )
            )
        else:
            futura_label.config(text="Futuro: calculando...")
    else:
        velocidad_label.config(text="Velocidad: -- cm/s")
        futura_label.config(text="Futuro: --")

    velocidad_promedio_label.config(text=f"Promedio: {velocidad_promedio:.1f} cm/s")
    cantidad_objetos_label.config(text=f"Objetos activos: {len(objetos_detectados)}")

    if objetos_detectados:
        lineas = []
        for objeto in objetos_detectados[:MAX_OBJETOS_PANEL]:
            if objeto["prediccion"] is not None:
                futuro = (
                    f" -> {angulo_para_vista(objeto['prediccion']['angulo']):.0f} deg/"
                    f"{objeto['prediccion']['distancia']:.0f} cm"
                )
            else:
                futuro = ""

            lineas.append(
                f"ID {objeto['id']}: {angulo_para_vista(objeto['angulo']):.0f} deg, "
                f"{objeto['distancia']:.0f} cm, {objeto['velocidad']:.1f} cm/s{futuro}"
            )

        detalle_objetos_label.config(text="\n".join(lineas))
    else:
        detalle_objetos_label.config(text="Sin objetos agrupados en el ultimo barrido.")


# Dibuja el radar, el barrido y los objetos en pantalla.
def draw_radar():
    canvas.delete("all")

    width = canvas.winfo_width()
    height = canvas.winfo_height()
    if width < 120 or height < 120:
        return

    centro_x = width // 2
    centro_y = height // 2
    radio_max = min(width, height) // 2 - 45
    if radio_max <= 20:
        return

    color_principal = "#00FF66"
    color_secundario = "#0C4F22"
    color_suave = "#083215"

    # Dibuja los anillos concentricos de distancia.
    for indice in range(1, 5):
        radio = radio_max * indice / 4
        canvas.create_oval(
            centro_x - radio,
            centro_y - radio,
            centro_x + radio,
            centro_y + radio,
            outline=color_principal if indice == 4 else color_secundario,
            width=2 if indice == 4 else 1,
        )

        valor_distancia = int(DISTANCIA_MAXIMA_CM * indice / 4)
        canvas.create_text(
            centro_x + 14,
            centro_y - radio + 12,
            text=str(valor_distancia),
            fill=color_principal if indice == 4 else color_secundario,
            font=("Consolas", max(7, radio_max // 22)),
        )

    # Dibuja la cuadricula angular base.
    for angulo in range(0, 360, 30):
        x_fin, y_fin = coordenadas_canvas(DISTANCIA_MAXIMA_CM, angulo, centro_x, centro_y, radio_max)
        canvas.create_line(centro_x, centro_y, x_fin, y_fin, fill=color_suave, width=1)

    # Marca los angulos visibles alrededor del radar.
    for angulo in range(0, 360, 45):
        angulo_visual = angulo_para_vista(angulo)
        x_texto, y_texto = coordenadas_canvas(DISTANCIA_MAXIMA_CM + 8, angulo, centro_x, centro_y, radio_max)
        canvas.create_text(
            x_texto,
            y_texto,
            text=f"{angulo_visual:.0f} deg",
            fill=color_principal if angulo_visual % 90 == 0 else color_secundario,
            font=("Consolas", max(7, radio_max // 25), "bold" if angulo_visual % 90 == 0 else "normal"),
        )

    # Marca el centro del radar.
    canvas.create_oval(
        centro_x - 4,
        centro_y - 4,
        centro_x + 4,
        centro_y + 4,
        fill=color_principal,
        outline=color_principal,
    )

    cola_barrido = [
        ("#00FF66", 3, 0),
        ("#00CC55", 2, -4),
        ("#009944", 2, -8),
        ("#006633", 1, -12),
    ]

    # Dibuja la cola del barrido para dar sensacion de movimiento.
    for color, grosor, offset in cola_barrido:
        angulo_barrido = normalizar_angulo(angulo_actual + offset)
        x_fin, y_fin = coordenadas_canvas(DISTANCIA_MAXIMA_CM, angulo_barrido, centro_x, centro_y, radio_max)
        canvas.create_line(centro_x, centro_y, x_fin, y_fin, fill=color, width=grosor)

    # Pinta el rastro reciente de detecciones.
    if historial_puntos:
        total_puntos = len(historial_puntos)
        for indice, muestra in enumerate(historial_puntos):
            pixel_x, pixel_y = coordenadas_canvas(
                muestra["distancia"],
                muestra["angulo"],
                centro_x,
                centro_y,
                radio_max,
            )
            intensidad = int(60 + 180 * ((indice + 1) / total_puntos))
            color = f"#00{intensidad:02X}22"
            radio_punto = 2 if indice < total_puntos - 30 else 3
            canvas.create_oval(
                pixel_x - radio_punto,
                pixel_y - radio_punto,
                pixel_x + radio_punto,
                pixel_y + radio_punto,
                fill=color,
                outline="",
            )

    # Resalta los objetos agrupados y sus datos principales.
    for objeto in objetos_detectados:
        pixel_x, pixel_y = coordenadas_canvas(objeto["distancia"], objeto["angulo"], centro_x, centro_y, radio_max)
        radio_punto = max(5, radio_max // 25)

        canvas.create_oval(
            pixel_x - radio_punto - 2,
            pixel_y - radio_punto - 2,
            pixel_x + radio_punto + 2,
            pixel_y + radio_punto + 2,
            outline="#FF3300",
            width=1,
        )
        canvas.create_oval(
            pixel_x - radio_punto,
            pixel_y - radio_punto,
            pixel_x + radio_punto,
            pixel_y + radio_punto,
            fill="#FF2200",
            outline="#FFF066",
            width=2,
        )

        canvas.create_text(
            pixel_x,
            pixel_y - radio_punto - 12,
            text=f"ID {objeto['id']}",
            fill="#FFD966",
            font=("Consolas", max(7, radio_max // 32), "bold"),
        )
        canvas.create_text(
            pixel_x + radio_punto + 10,
            pixel_y - radio_punto,
            text=f"{objeto['velocidad']:.1f} cm/s",
            fill="#00FF66",
            font=("Consolas", max(7, radio_max // 30), "bold"),
            anchor="w",
        )
        canvas.create_text(
            pixel_x + radio_punto + 10,
            pixel_y + radio_punto + 10,
            text=f"{objeto['distancia']:.0f} cm",
            fill="#00CC66",
            font=("Consolas", max(6, radio_max // 34)),
            anchor="w",
        )

        prediccion = objeto["prediccion"]
        if prediccion is not None:
            # Muestra hacia donde se moveria el objeto.
            pred_x, pred_y = coordenadas_canvas(
                prediccion["distancia"],
                prediccion["angulo"],
                centro_x,
                centro_y,
                radio_max,
            )
            canvas.create_line(
                pixel_x,
                pixel_y,
                pred_x,
                pred_y,
                fill="#6D5CFF",
                width=2,
                dash=(4, 4),
            )
            radio_prediccion = max(8, radio_max // 28)
            canvas.create_oval(
                pred_x - radio_prediccion,
                pred_y - radio_prediccion,
                pred_x + radio_prediccion,
                pred_y + radio_prediccion,
                fill="#7D5CFF",
                outline="#D7C8FF",
                width=2,
            )
            canvas.create_text(
                pred_x,
                pred_y,
                text=str(objeto["id"]),
                fill="#F7F1FF",
                font=("Consolas", max(7, radio_max // 36), "bold"),
            )
            canvas.create_text(
                pred_x + radio_prediccion + 8,
                pred_y - radio_prediccion - 6,
                text=f"Pred ID {objeto['id']}",
                fill="#C7B8FF",
                font=("Consolas", max(7, radio_max // 36), "bold"),
                anchor="w",
            )


# Coordina lectura serial, tracking y repintado de la GUI.
def actualizar_radar():
    global angulo_actual

    lecturas = leer_datos_arduino()

    if lecturas:
        tiempo_base = time.time()
        # Reparte un tiempo aproximado entre lecturas del mismo lote.
        paso_tiempo = max(0.001, (INTERVALO_ACTUALIZACION_MS / 1000.0) / max(1, len(lecturas)))

        for indice, (distancia, angulo) in enumerate(lecturas):
            tiempo_lectura = tiempo_base - (len(lecturas) - indice - 1) * paso_tiempo
            angulo_actual = angulo
            registrar_lectura(distancia, angulo, tiempo_lectura)

        distancia_final, angulo_final = lecturas[-1]
        actualizar_panel_objetos(distancia_final, angulo_final)

    reloj_label.config(text=datetime.datetime.now().strftime("%H:%M:%S"))
    refrescar_estado_conexion()
    draw_radar()
    window.after(INTERVALO_ACTUALIZACION_MS, actualizar_radar)


# Redibuja cuando cambia el tamano util del canvas.
def on_configure(_event):
    draw_radar()


# Libera el puerto serial antes de cerrar la ventana.
def al_cerrar():
    cerrar_arduino()
    window.destroy()


# Arranca conexion, estado inicial y primer ciclo del radar.
def inicializar():
    conectar_arduino()
    refrescar_estado_conexion()
    draw_radar()
    actualizar_radar()


# Construye la ventana principal del radar.
window = tk.Tk()
window.title("RADAR 360 Proyecto Fundamentos II")
window.geometry("1080x720")
window.minsize(860, 620)
window.configure(bg="#050505")

# Contenedor del canvas del radar.
frame_radar = tk.Frame(window, bg="#000000")
frame_radar.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)

# Area de dibujo del radar.
canvas = tk.Canvas(frame_radar, bg="#000000", highlightthickness=0)
canvas.pack(fill=tk.BOTH, expand=True)

# Panel lateral con lecturas, estado y resumen.
panel_info = tk.Frame(
    window,
    bg="#0A0A0A",
    width=300,
    highlightthickness=1,
    highlightbackground="#00FF66",
)
panel_info.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=10)
panel_info.pack_propagate(False)

# Titulo principal del panel.
titulo_label = tk.Label(
    panel_info,
    text="RADAR 360",
    font=("Consolas", 15, "bold"),
    fg="#00FF66",
    bg="#0A0A0A",
)
titulo_label.pack(pady=8)

# Separador visual superior.
separator = tk.Frame(panel_info, bg="#00FF66", height=1)
separator.pack(fill=tk.X, padx=12, pady=4)

# Datos principales de la lectura actual.
distancia_label = tk.Label(
    panel_info,
    text="Distancia: -- cm",
    font=("Consolas", 11),
    fg="#00FF66",
    bg="#0A0A0A",
)
distancia_label.pack(pady=4)

angulo_label = tk.Label(
    panel_info,
    text="Angulo: -- deg",
    font=("Consolas", 11),
    fg="#00FF66",
    bg="#0A0A0A",
)
angulo_label.pack(pady=4)

velocidad_label = tk.Label(
    panel_info,
    text="Velocidad: -- cm/s",
    font=("Consolas", 11),
    fg="#00FF66",
    bg="#0A0A0A",
)
velocidad_label.pack(pady=4)

futura_label = tk.Label(
    panel_info,
    text="Futuro: --",
    font=("Consolas", 11),
    fg="#A998FF",
    bg="#0A0A0A",
)
futura_label.pack(pady=4)

# Seccion de resumen general.
separator2 = tk.Frame(panel_info, bg="#003300", height=1)
separator2.pack(fill=tk.X, padx=12, pady=8)

velocidad_promedio_label = tk.Label(
    panel_info,
    text="Promedio: -- cm/s",
    font=("Consolas", 12, "bold"),
    fg="#00FF66",
    bg="#0A0A0A",
)
velocidad_promedio_label.pack(pady=5)

cantidad_objetos_label = tk.Label(
    panel_info,
    text="Objetos activos: 0",
    font=("Consolas", 11, "bold"),
    fg="#FFD966",
    bg="#0A0A0A",
)
cantidad_objetos_label.pack(pady=5)

reloj_label = tk.Label(
    panel_info,
    text="--:--:--",
    font=("Consolas", 10),
    fg="#00AA44",
    bg="#0A0A0A",
)
reloj_label.pack(pady=4)

# Seccion del estado de conexion.
separator3 = tk.Frame(panel_info, bg="#003300", height=1)
separator3.pack(fill=tk.X, padx=12, pady=8)

conexion_label = tk.Label(
    panel_info,
    text="INICIANDO",
    font=("Consolas", 10, "bold"),
    fg="#FFAA00",
    bg="#0A0A0A",
)
conexion_label.pack(pady=4)

puerto_label = tk.Label(
    panel_info,
    text=PUERTO_SERIAL_PREFERIDO,
    font=("Consolas", 9),
    fg="#00AA44",
    bg="#0A0A0A",
)
puerto_label.pack(pady=3)

estado_label = tk.Label(
    panel_info,
    text="Esperando conexion serial...",
    font=("Consolas", 9),
    fg="#55CC88",
    bg="#0A0A0A",
    justify=tk.LEFT,
    wraplength=250,
)
estado_label.pack(padx=14, pady=6)

# Seccion del detalle de objetos detectados.
separator4 = tk.Frame(panel_info, bg="#003300", height=1)
separator4.pack(fill=tk.X, padx=12, pady=8)

detalle_titulo_label = tk.Label(
    panel_info,
    text="Resumen de objetos",
    font=("Consolas", 11, "bold"),
    fg="#00FF66",
    bg="#0A0A0A",
)
detalle_titulo_label.pack(pady=4)

detalle_objetos_label = tk.Label(
    panel_info,
    text="Sin objetos agrupados en el ultimo barrido.",
    font=("Consolas", 9),
    fg="#B8FFCC",
    bg="#0A0A0A",
    justify=tk.LEFT,
    wraplength=250,
)
detalle_objetos_label.pack(padx=14, pady=6)

nota_label = tk.Label(
    panel_info,
    text="Formato serial esperado:\ndistancia,angulo",
    font=("Consolas", 9),
    fg="#33AA66",
    bg="#0A0A0A",
    justify=tk.LEFT,
)
nota_label.pack(padx=14, pady=10)

# Eventos de repintado y cierre controlado.
canvas.bind("<Configure>", on_configure)
window.bind("<Configure>", lambda _event: window.after_idle(draw_radar))
window.protocol("WM_DELETE_WINDOW", al_cerrar)

# Estado inicial y arranque del loop visual.
refrescar_estado_conexion()
window.after(100, inicializar)
window.mainloop()
