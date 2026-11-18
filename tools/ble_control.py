#!/usr/bin/env python3
"""
RD90 Console - Control BLE (GATT) completo
==================================================================

GUI para conectarse por BLE a la consola RD90 (WP_C_V9_UP, DEVICE_NAME=
"RD90Pro"): ve en vivo todo lo que la consola ya manda (tilt, bateria,
temperatura, encoder, eventos de captura, version) Y ademas permite
controlarla: brillo de luz, movimiento (adelante/atras/derecha/izquierda con
velocidad) y los 3 servos (cabeza, cuello, servo3).

Replica exactamente el protocolo BLE de main.c (ver comentarios junto a cada
UUID ahi, BLE_UUID128_INIT ~linea 395-470):

  LECTURA (Read + Notify, salvo c006 que es solo Read):
    c001 tilt     : 2x int16 LE  -> [pitch_raw, roll_raw]
    c002 battery  : 2x int16 LE  -> [consola_mV, robot_raw]
    c003 temp     : 1x int16 LE  -> temp_raw
    c004 encoder  : 1x int32 LE  -> conteo
    c005 capture  : 1x uint8     -> contador de pulsaciones
    c006 fwver    : 4x uint8     -> [proto_ver, major, minor, patch]
    c00C cam2     : 1x uint8     -> contador de comandos RL1 (CAM2)

  CONTROL (Write-only):
    c007 led      : 1x uint8     -> brillo, porcentaje 0-100
    c008 motor    : 2x uint8     -> [direccion 0-4, velocidad % 0-100]
                                     direccion: 0=stop 1=adelante 2=atras 3=derecha 4=izquierda
    c009 srv1     : 1x uint16 LE -> angulo cabeza, 0-180
    c00A srv2     : 1x uint16 LE -> angulo cuello, 0-180
    c00B srv3     : 1x uint16 LE -> angulo servo3, 0-270

IMPORTANTE -- prioridad del control fisico: en TODOS los casos (luz, motor,
servos), el joystick/botones fisicos de la consola tienen prioridad sobre
BLE. Si alguien toca el control fisico correspondiente mientras estas
controlando por BLE, el fisico retoma el control de inmediato. Esto es a
proposito (ver los comentarios en main.c junto a cada *ChrAccess()).

NOTA sobre pitch/roll/temp: el robot los manda por Modbus en grados x100
(TABLAS_MODBUS.md) y la consola los reenvia por BLE sin dividir (bug
conocido, pendiente). Por eso se muestra el crudo y el /100.

Requisitos:
    pip install bleak
"""

import asyncio
import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from datetime import datetime

try:
    from bleak import BleakScanner, BleakClient
except ImportError:
    raise SystemExit(
        "Falta bleak. Instala con: pip install bleak"
    )

# ============================================================
# UUIDs (128-bit) - mismos que main.c, invertidos a formato string
# estandar (NimBLE guarda los bytes en orden "de aire"). Se matchea
# ademas por sufijo (...c00N) como red de seguridad.
# ============================================================
DEVICE_NAME_FILTER = "RD90"

READ_CHAR_DEFS = {
    "tilt":    {"uuid": "0000bcea-5f78-2315-deef-1212a1f1c001", "suffix": "c001", "label": "Tilt (pitch/roll)"},
    "battery": {"uuid": "0000bcea-5f78-2315-deef-1212a1f1c002", "suffix": "c002", "label": "Bateria"},
    "temp":    {"uuid": "0000bcea-5f78-2315-deef-1212a1f1c003", "suffix": "c003", "label": "Temperatura"},
    "encoder": {"uuid": "0000bcea-5f78-2315-deef-1212a1f1c004", "suffix": "c004", "label": "Encoder"},
    "capture": {"uuid": "0000bcea-5f78-2315-deef-1212a1f1c005", "suffix": "c005", "label": "Captura (evento)"},
    "fwver":   {"uuid": "0000bcea-5f78-2315-deef-1212a1f1c006", "suffix": "c006", "label": "Version FW/protocolo"},
    "cam2":    {"uuid": "0000bcea-5f78-2315-deef-1212a1f1c00c", "suffix": "c00c", "label": "CAM2 (evento RL1)"},
}

WRITE_CHAR_DEFS = {
    "led":   {"uuid": "0000bcea-5f78-2315-deef-1212a1f1c007", "suffix": "c007", "label": "LED"},
    "motor": {"uuid": "0000bcea-5f78-2315-deef-1212a1f1c008", "suffix": "c008", "label": "Motor"},
    "srv1":  {"uuid": "0000bcea-5f78-2315-deef-1212a1f1c009", "suffix": "c009", "label": "Servo1 (cabeza)"},
    "srv2":  {"uuid": "0000bcea-5f78-2315-deef-1212a1f1c00a", "suffix": "c00a", "label": "Servo2 (cuello)"},
    "srv3":  {"uuid": "0000bcea-5f78-2315-deef-1212a1f1c00b", "suffix": "c00b", "label": "Servo3"},
}

MOTOR_DIR = {"stop": 0, "adelante": 1, "atras": 2, "derecha": 3, "izquierda": 4}


def decode_payload(key: str, data: bytes) -> str:
    """Decodifica el payload crudo de cada característica de LECTURA."""
    try:
        if key == "tilt":
            if len(data) != 4:
                return f"(tamaño inesperado {len(data)})"
            pitch_raw, roll_raw = struct.unpack("<hh", data)
            return (f"pitch_raw={pitch_raw:6d}  roll_raw={roll_raw:6d}   "
                    f"(/100 → pitch={pitch_raw/100:.2f}°  roll={roll_raw/100:.2f}°)")

        if key == "battery":
            if len(data) != 4:
                return f"(tamaño inesperado {len(data)})"
            console_mv, robot_raw = struct.unpack("<hh", data)
            return f"consola={console_mv} mV   robot={robot_raw} (raw)"

        if key == "temp":
            if len(data) != 2:
                return f"(tamaño inesperado {len(data)})"
            (temp_raw,) = struct.unpack("<h", data)
            return f"temp_raw={temp_raw}   (/100 → {temp_raw/100:.2f} °C)"

        if key == "encoder":
            if len(data) != 4:
                return f"(tamaño inesperado {len(data)})"
            (count,) = struct.unpack("<i", data)
            return f"count={count}"

        if key == "capture":
            if len(data) != 1:
                return f"(tamaño inesperado {len(data)})"
            (evt,) = struct.unpack("<B", data)
            return f"contador de pulsaciones={evt}"

        if key == "cam2":
            if len(data) != 1:
                return f"(tamaño inesperado {len(data)})"
            (evt,) = struct.unpack("<B", data)
            return f"contador de comandos CAM2={evt}"

        if key == "fwver":
            if len(data) != 4:
                return f"(tamaño inesperado {len(data)})"
            proto, major, minor, patch = struct.unpack("<BBBB", data)
            return f"protocolo v{proto}   firmware v{major}.{minor}.{patch}"

    except struct.error as e:
        return f"(error decodificando: {e})"

    return "(clave desconocida)"


# ============================================================
# ESTADO COMPARTIDO
# ============================================================
class BleState:
    def __init__(self):
        self.lock = threading.Lock()
        self.connected = False
        self.values = {k: "(sin datos aun)" for k in READ_CHAR_DEFS}
        self.last_update = {k: 0.0 for k in READ_CHAR_DEFS}
        self.log_lines = []

    def log(self, text: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        with self.lock:
            self.log_lines.append(f"[{ts}] {text}")
            if len(self.log_lines) > 500:
                self.log_lines = self.log_lines[-500:]

    def update_value(self, key: str, data: bytes):
        text = decode_payload(key, data)
        with self.lock:
            changed = (self.values[key] != text)
            self.values[key] = text
            self.last_update[key] = time.time()
        if changed:
            self.log(f"{READ_CHAR_DEFS[key]['label']}: {text}")


# ============================================================
# HILO ASYNCIO — conecta, se suscribe a lectura, y drena una cola
# de escrituras pendientes (control) mientras dure la conexion.
# ============================================================
class BleWorker(threading.Thread):
    def __init__(self, state: BleState, on_status):
        super().__init__(daemon=True)
        self.state = state
        self.on_status = on_status
        self.loop = None
        self._stop_flag = threading.Event()
        self._target_address = None
        self._connect_requested = threading.Event()
        self._disconnect_requested = threading.Event()
        self.write_queue = queue.Queue()
        self._matched_write_chars = {}  # key -> uuid real (solo valido mientras conectado)

    def run(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._main())

    def stop(self):
        self._stop_flag.set()

    def request_connect(self, address: str):
        self._target_address = address
        self._connect_requested.set()

    def request_disconnect(self):
        self._disconnect_requested.set()

    def queue_write(self, key: str, data: bytes, clear_pending: bool = False):
        """Llamable desde el hilo de Tkinter. No bloquea.
        clear_pending=True descarta lo que haya en cola antes de encolar esto
        (para comandos urgentes como STOP, que no deben esperar detrás de
        varios "adelante" acumulados por auto-repeat del teclado)."""
        if self.state.connected:
            if clear_pending:
                try:
                    while True:
                        self.write_queue.get_nowait()
                except queue.Empty:
                    pass
            self.write_queue.put((key, data))

    async def _main(self):
        while not self._stop_flag.is_set():
            if self._connect_requested.is_set():
                self._connect_requested.clear()
                await self._connect_and_run(self._target_address)
            await asyncio.sleep(0.1)

    async def _connect_and_run(self, address: str):
        self.on_status(f"🔄 Conectando a {address}...")
        try:
            async with BleakClient(address) as client:
                self.state.connected = True
                self.on_status(f"✅ Conectado a {address}")
                self.state.log(f"Conectado a {address}")

                matched_read = {}
                matched_write = {}
                for svc in client.services:
                    for ch in svc.characteristics:
                        ch_uuid = str(ch.uuid).lower()
                        for key, meta in READ_CHAR_DEFS.items():
                            if ch_uuid == meta["uuid"] or ch_uuid.endswith(meta["suffix"]):
                                matched_read[key] = ch.uuid
                        for key, meta in WRITE_CHAR_DEFS.items():
                            if ch_uuid == meta["uuid"] or ch_uuid.endswith(meta["suffix"]):
                                matched_write[key] = ch.uuid
                        self.state.log(f"Descubierto: {ch_uuid}  props={list(ch.properties)}")

                self._matched_write_chars = matched_write
                if not matched_read and not matched_write:
                    self.state.log("⚠️ No se encontro ninguna caracteristica conocida del servicio RD90")

                if "fwver" in matched_read:
                    try:
                        data = await client.read_gatt_char(matched_read["fwver"])
                        self.state.update_value("fwver", bytes(data))
                    except Exception as e:
                        self.state.log(f"Error leyendo fwver: {e}")

                def make_handler(key):
                    def handler(_sender, data: bytearray):
                        self.state.update_value(key, bytes(data))
                    return handler

                subscribed = []
                for key, ch_uuid in matched_read.items():
                    if key == "fwver":
                        continue
                    try:
                        await client.start_notify(ch_uuid, make_handler(key))
                        subscribed.append(key)
                    except Exception as e:
                        self.state.log(f"No se pudo suscribir a {key}: {e}")

                if subscribed:
                    self.state.log(f"Suscrito a notify: {', '.join(subscribed)}")
                if matched_write:
                    self.state.log(f"Control disponible: {', '.join(matched_write.keys())}")

                # --- Loop principal: espera desconexion, drena escrituras pendientes ---
                while client.is_connected and not self._disconnect_requested.is_set() and not self._stop_flag.is_set():
                    drained = 0
                    while drained < 10:
                        try:
                            key, data = self.write_queue.get_nowait()
                        except queue.Empty:
                            break
                        drained += 1
                        ch_uuid = self._matched_write_chars.get(key)
                        if not ch_uuid:
                            self.state.log(f"⚠️ Caracteristica '{key}' no disponible en este firmware")
                            continue
                        try:
                            await client.write_gatt_char(ch_uuid, data, response=True)
                        except Exception as e:
                            self.state.log(f"❌ Error escribiendo {key}: {e}")

                    await asyncio.sleep(0.05)

                self._disconnect_requested.clear()
                self.state.log("Desconectando...")

        except Exception as e:
            self.state.log(f"❌ Error de conexion: {e}")
            self.on_status(f"❌ Error: {e}")
        finally:
            self.state.connected = False
            self._matched_write_chars = {}
            self.on_status("⏹️ Desconectado")


# ============================================================
# INTERFAZ GRAFICA
# ============================================================
class RD90BleControlApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("RD90 Console — Control BLE")
        self.geometry("820x900")
        self.configure(bg="#1e1e1e")

        self.state = BleState()
        self.worker = BleWorker(self.state, self._set_status)
        self.worker.start()

        self.devices = []
        self._last_led_send = 0.0
        self._last_srv_send = {"srv1": 0.0, "srv2": 0.0, "srv3": 0.0}
        self._speed_var = tk.IntVar(value=50)

        self._build_ui()
        self._bind_keys()
        self.after(150, self._refresh_loop)

    # ---------------------------------------------------------
    def _build_ui(self):
        FG, BG = "#e6e6e6", "#1e1e1e"
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TLabel", background=BG, foreground=FG, font=("Segoe UI", 10))
        style.configure("Title.TLabel", background=BG, foreground=FG, font=("Segoe UI", 13, "bold"))
        style.configure("TFrame", background=BG)
        style.configure("TButton", font=("Segoe UI", 10))
        style.configure("Dir.TButton", font=("Segoe UI", 11, "bold"))
        style.configure("Treeview", font=("Segoe UI", 9), rowheight=22)
        style.configure("Treeview.Heading", font=("Segoe UI", 9, "bold"))
        style.configure("Horizontal.TScale", background=BG)

        title = ttk.Label(self, text="RD90 Console — Control BLE completo", style="Title.TLabel")
        title.pack(pady=(10, 4))

        # --- Conexion ---
        conn_frame = ttk.Frame(self)
        conn_frame.pack(pady=4, fill="x", padx=14)
        ttk.Button(conn_frame, text="Escanear", command=self._start_scan).pack(side="left")
        self.device_var = tk.StringVar()
        self.device_combo = ttk.Combobox(conn_frame, textvariable=self.device_var, width=40, state="readonly")
        self.device_combo.pack(side="left", padx=6)
        self.connect_btn = ttk.Button(conn_frame, text="Conectar", command=self._toggle_connection)
        self.connect_btn.pack(side="left", padx=4)

        self.status_label = ttk.Label(self, text="Desconectado", foreground="#e06666")
        self.status_label.pack(pady=(2, 8))

        # --- Control: Movimiento ---
        motor_frame = ttk.Frame(self)
        motor_frame.pack(fill="x", padx=14, pady=6)
        ttk.Label(motor_frame, text="Movimiento (mantener presionado)", style="Title.TLabel").pack(anchor="w")

        speed_row = ttk.Frame(motor_frame)
        speed_row.pack(fill="x", pady=(4, 8))
        ttk.Label(speed_row, text="Velocidad:").pack(side="left")
        speed_scale = ttk.Scale(speed_row, from_=0, to=100, orient="horizontal",
                                 variable=self._speed_var, length=300)
        speed_scale.pack(side="left", padx=8)
        self.speed_value_label = ttk.Label(speed_row, text="50%", width=5)
        self.speed_value_label.pack(side="left")
        speed_scale.configure(command=lambda v: self.speed_value_label.config(text=f"{int(float(v))}%"))

        pad_frame = ttk.Frame(motor_frame)
        pad_frame.pack(pady=4)
        btn_fwd = ttk.Button(pad_frame, text="▲\nAdelante", style="Dir.TButton", width=10)
        btn_fwd.grid(row=0, column=1, padx=4, pady=2)
        btn_left = ttk.Button(pad_frame, text="◄\nIzquierda", style="Dir.TButton", width=10)
        btn_left.grid(row=1, column=0, padx=4, pady=2)
        btn_stop = ttk.Button(pad_frame, text="STOP", style="Dir.TButton", width=10,
                               command=lambda: self._send_motor("stop"))
        btn_stop.grid(row=1, column=1, padx=4, pady=2)
        btn_right = ttk.Button(pad_frame, text="►\nDerecha", style="Dir.TButton", width=10)
        btn_right.grid(row=1, column=2, padx=4, pady=2)
        btn_back = ttk.Button(pad_frame, text="▼\nAtrás", style="Dir.TButton", width=10)
        btn_back.grid(row=2, column=1, padx=4, pady=2)

        for btn, direction in ((btn_fwd, "adelante"), (btn_back, "atras"),
                                (btn_right, "derecha"), (btn_left, "izquierda")):
            btn.bind("<ButtonPress-1>", lambda e, d=direction: self._send_motor(d))
            btn.bind("<ButtonRelease-1>", lambda e: self._send_motor("stop"))

        ttk.Label(motor_frame,
                  text="Teclado: W/A/S/D = mover (mantener) · Q/E = luz -/+ · "
                       "↑/↓ = cabeza · ←/→ = cuello   |   "
                       "El control físico de la consola tiene prioridad sobre esto.",
                  foreground="#999999", wraplength=760, justify="left").pack(anchor="w", pady=(4, 0))

        # --- Control: LED ---
        led_frame = ttk.Frame(self)
        led_frame.pack(fill="x", padx=14, pady=6)
        ttk.Label(led_frame, text="Luz del robot", style="Title.TLabel").pack(anchor="w")
        led_row = ttk.Frame(led_frame)
        led_row.pack(fill="x", pady=4)
        self.led_var = tk.IntVar(value=0)
        led_scale = ttk.Scale(led_row, from_=0, to=100, orient="horizontal",
                               variable=self.led_var, length=400, command=self._on_led_change)
        led_scale.pack(side="left", padx=(0, 8))
        self.led_value_label = ttk.Label(led_row, text="0%", width=5)
        self.led_value_label.pack(side="left")

        # --- Control: Servos ---
        srv_frame = ttk.Frame(self)
        srv_frame.pack(fill="x", padx=14, pady=6)
        ttk.Label(srv_frame, text="Servos", style="Title.TLabel").pack(anchor="w")

        self.srv_vars = {}
        self.srv_labels = {}
        self._srv_ranges = {}
        srv_specs = [("srv1", "Cabeza", 0, 180), ("srv2", "Cuello", 0, 180), ("srv3", "Servo3", 0, 270)]
        for key, label, lo, hi in srv_specs:
            self._srv_ranges[key] = (lo, hi)
            row = ttk.Frame(srv_frame)
            row.pack(fill="x", pady=3)
            ttk.Label(row, text=f"{label} ({lo}-{hi}°):", width=16).pack(side="left")
            var = tk.IntVar(value=(hi // 2))
            self.srv_vars[key] = var
            scale = ttk.Scale(row, from_=lo, to=hi, orient="horizontal", variable=var, length=320,
                               command=lambda v, k=key: self._on_srv_change(k, v))
            scale.pack(side="left", padx=8)
            lbl = ttk.Label(row, text=f"{hi // 2}°", width=6)
            lbl.pack(side="left")
            self.srv_labels[key] = lbl

        # --- Tabla de lectura en vivo ---
        table_frame = ttk.Frame(self)
        table_frame.pack(fill="x", padx=14, pady=6)
        ttk.Label(table_frame, text="Lectura en vivo", style="Title.TLabel").pack(anchor="w")

        columns = ("valor", "hace")
        self.tree = ttk.Treeview(table_frame, columns=columns, show="tree headings", height=6)
        self.tree.heading("#0", text="Característica")
        self.tree.heading("valor", text="Valor decodificado")
        self.tree.heading("hace", text="Última act.")
        self.tree.column("#0", width=160)
        self.tree.column("valor", width=460)
        self.tree.column("hace", width=90, anchor="center")
        self.tree.pack(fill="x", pady=4)

        self.row_ids = {}
        for key, meta in READ_CHAR_DEFS.items():
            self.row_ids[key] = self.tree.insert("", "end", text=meta["label"], values=("(sin datos aun)", "-"))

        # --- Log ---
        log_frame = ttk.Frame(self)
        log_frame.pack(fill="both", expand=True, padx=14, pady=(4, 10))
        ttk.Label(log_frame, text="Log", style="Title.TLabel").pack(anchor="w")
        self.log_text = tk.Text(log_frame, bg="#111111", fg="#9fd3a8", font=("Consolas", 9), height=8,
                                 insertbackground="#ffffff")
        self.log_text.pack(fill="both", expand=True, pady=4)
        self.log_text.configure(state="disabled")
        self._last_log_len = 0

    # ---------------------------------------------------------
    # ESCANEO / CONEXION
    # ---------------------------------------------------------
    def _start_scan(self):
        self.device_combo["values"] = ["(escaneando...)"]
        threading.Thread(target=self._scan_thread, daemon=True).start()

    def _scan_thread(self):
        async def scan():
            return await BleakScanner.discover(timeout=5.0)
        try:
            devices = asyncio.run(scan())
        except Exception as e:
            self.after(0, lambda: messagebox.showerror("Error de escaneo", str(e)))
            return

        self.devices = devices
        labels = []
        for d in devices:
            name = d.name or "(sin nombre)"
            marker = " ⭐" if DEVICE_NAME_FILTER.lower() in name.lower() else ""
            labels.append(f"{name}{marker}  [{d.address}]")

        def apply():
            self.device_combo["values"] = labels
            if labels:
                for i, d in enumerate(devices):
                    if DEVICE_NAME_FILTER.lower() in (d.name or "").lower():
                        self.device_combo.current(i)
                        break
                else:
                    self.device_combo.current(0)
            self._set_status(f"Escaneo terminado: {len(devices)} dispositivo(s) encontrado(s)")
        self.after(0, apply)

    def _toggle_connection(self):
        if self.state.connected:
            self.worker.request_disconnect()
            self.connect_btn.config(text="Conectar")
            return
        idx = self.device_combo.current()
        if idx < 0 or idx >= len(self.devices):
            messagebox.showwarning("Sin dispositivo", "Escaneá y seleccioná un dispositivo primero.")
            return
        address = self.devices[idx].address
        self.worker.request_connect(address)
        self.connect_btn.config(text="Desconectar")

    def _set_status(self, text):
        self.after(0, lambda: self.status_label.config(
            text=text,
            foreground="#6aab73" if text.startswith("✅") else
                       ("#e0a526" if text.startswith("🔄") else "#e06666"),
        ))

    # ---------------------------------------------------------
    # CONTROL: movimiento
    # ---------------------------------------------------------
    def _send_motor(self, direction: str):
        dir_val = MOTOR_DIR[direction]
        pct = 0 if direction == "stop" else self._speed_var.get()
        payload = bytes([dir_val, pct])
        # STOP es urgente: descarta cualquier "adelante/atras/..." que haya
        # quedado acumulado en la cola por el auto-repeat del teclado, para
        # que frene de inmediato en vez de esperar a que se vacíe la cola.
        self.worker.queue_write("motor", payload, clear_pending=(direction == "stop"))

    # ---------------------------------------------------------
    # TECLADO: W/A/S/D mueve (mantenido), Q/E baja/sube la luz
    # ---------------------------------------------------------
    def _bind_keys(self):
        self.bind("<KeyPress>", self._on_key_down)
        self.bind("<KeyRelease>", self._on_key_up)
        self.focus_set()

    _KEY_TO_DIR = {"w": "adelante", "s": "atras", "a": "izquierda", "d": "derecha"}
    _active_direction = None  # evita re-encolar la misma direccion en cada repeticion de tecla

    def _on_key_down(self, event):
        key = event.keysym.lower()
        if key in self._KEY_TO_DIR:
            direction = self._KEY_TO_DIR[key]
            if self._active_direction != direction:
                self._active_direction = direction
                self._send_motor(direction)
        elif key == "q":
            self._adjust_led(-5)
        elif key == "e":
            self._adjust_led(+5)
        elif key == "up":
            self._adjust_servo("srv1", +3)   # cabeza
        elif key == "down":
            self._adjust_servo("srv1", -3)   # cabeza
        elif key == "right":
            self._adjust_servo("srv2", +3)   # cuello
        elif key == "left":
            self._adjust_servo("srv2", -3)   # cuello

    def _on_key_up(self, event):
        key = event.keysym.lower()
        if key in self._KEY_TO_DIR:
            self._active_direction = None
            self._send_motor("stop")

    # ---------------------------------------------------------
    # CONTROL: LED (throttle ~100ms mientras se arrastra)
    # ---------------------------------------------------------
    def _adjust_led(self, delta: int):
        new_val = max(0, min(100, self.led_var.get() + delta))
        self.led_var.set(new_val)
        self.led_value_label.config(text=f"{new_val}%")
        self.worker.queue_write("led", bytes([new_val]))

    def _on_led_change(self, value_str):
        pct = int(float(value_str))
        self.led_value_label.config(text=f"{pct}%")
        now = time.time()
        if (now - self._last_led_send) > 0.1:
            self._last_led_send = now
            self.worker.queue_write("led", bytes([pct]))

    # ---------------------------------------------------------
    # CONTROL: servos (throttle ~100ms mientras se arrastra)
    # ---------------------------------------------------------
    def _adjust_servo(self, key: str, delta: int):
        lo, hi = self._srv_ranges[key]
        new_val = max(lo, min(hi, self.srv_vars[key].get() + delta))
        self.srv_vars[key].set(new_val)
        self.srv_labels[key].config(text=f"{new_val}°")
        now = time.time()
        if (now - self._last_srv_send[key]) > 0.1:
            self._last_srv_send[key] = now
            self.worker.queue_write(key, struct.pack("<H", new_val))

    def _on_srv_change(self, key: str, value_str: str):
        angle = int(float(value_str))
        self.srv_labels[key].config(text=f"{angle}°")
        now = time.time()
        if (now - self._last_srv_send[key]) > 0.1:
            self._last_srv_send[key] = now
            self.worker.queue_write(key, struct.pack("<H", angle))

    # ---------------------------------------------------------
    # REFRESCO PERIODICO
    # ---------------------------------------------------------
    def _refresh_loop(self):
        with self.state.lock:
            values = dict(self.state.values)
            last_update = dict(self.state.last_update)
            log_lines = list(self.state.log_lines)

        now = time.time()
        for key, row_id in self.row_ids.items():
            ago = now - last_update[key]
            ago_txt = "-" if last_update[key] == 0 else (f"{ago:.1f}s" if ago < 60 else "hace rato")
            self.tree.item(row_id, values=(values[key], ago_txt))

        if len(log_lines) != self._last_log_len:
            self.log_text.configure(state="normal")
            self.log_text.delete("1.0", "end")
            self.log_text.insert("end", "\n".join(log_lines))
            self.log_text.see("end")
            self.log_text.configure(state="disabled")
            self._last_log_len = len(log_lines)

        self.after(150, self._refresh_loop)

    def on_close(self):
        self.worker.stop()
        self.destroy()


if __name__ == "__main__":
    app = RD90BleControlApp()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
