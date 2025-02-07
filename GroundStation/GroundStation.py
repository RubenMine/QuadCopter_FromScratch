#!/usr/bin/env python3
"""
Ground Station refactoring:
  - Separazione in classi per le funzionalità di telemetria, gestione del dispositivo,
    server Flask, visualizzazione 3D e GUI.
"""

import tkinter as tk
from tkinter import ttk
import threading
import time
import requests
import numpy as np
import matplotlib
matplotlib.use("TkAgg")  # Forza l'uso di TkAgg come backend
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D  # Necessario per 3D

from flask import Flask, request, jsonify


# -----------------------------------------------------------------------------
# TelemetryData: memorizza ed aggiorna i dati di telemetria condivisi
# -----------------------------------------------------------------------------
class QuadCopterData:
    def __init__(self):
        self.state = "WAIT"
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.altitude = 0.0
        
        self.pwm1 = 0
        self.pwm2 = 0
        self.pwm3 = 0
        self.pwm4 = 0

    def update_telemetry(self, roll, pitch, yaw, altitude):
        self.roll = float(roll)
        self.pitch = float(pitch)
        self.yaw = float(yaw)
        self.altitude = float(altitude)

    def update_state(self, state):
        self.state = state

    def update_motor(self, m1, m2, m3, m4):
    	self.pwm1 = m1
    	self.pwm2 = m2
    	self.pwm3 = m3
    	self.pwm4 = m4

# -----------------------------------------------------------------------------
# DeviceManager: gestisce la registrazione e l'invio dei comandi al dispositivo
# -----------------------------------------------------------------------------
class DeviceManager:
    def __init__(self):
        self.connected_device = "http://192.168.1.192:5001"  # E.g. "http://172.20.10.2:5001"

    def register_device(self, ip: str):
        if self.connected_device is not None:
            raise ValueError("Device already connected")
        self.connected_device = ip
        print(f"[GroundStation] Registrato nuovo dispositivo: {self.connected_device}")

    def send_command(self, cmd_type, cmd_info, cmd_value):
        if self.connected_device is None:
            print("[GroundStation] Errore: nessun device connesso.")
            return

        payload = {
            "command_type": cmd_type,
            "command_info": cmd_info,
            "command_value": cmd_value
        }

        try:
            url = f"http://192.168.1.192:5001/incoming_data"
            response = requests.post(url, json=payload, timeout=5)
            if response.status_code == 200:
                print("[GroundStation] Comando inviato correttamente:", payload)
            else:
                print(f"[GroundStation] Errore nell'invio (status={response.status_code}): {response.text}")
        except Exception as e:
            print(f"[GroundStation] Eccezione durante l'invio: {e}")


# -----------------------------------------------------------------------------
# FlaskServer: incapsula il server Flask e definisce gli endpoint
# -----------------------------------------------------------------------------
class FlaskServer:
    def __init__(self, data: QuadCopterData, device_manager: DeviceManager, host="0.0.0.0", port=6001):
        self.realtime_data = data
        self.device_manager = device_manager
        self.host = host
        self.port = port
        self.app = Flask(__name__)
        self.setup_routes()

    def setup_routes(self):
        self.app.add_url_rule("/register", methods=["POST"], view_func=self.register)
        self.app.add_url_rule("/TELEMETRY", methods=["POST"], view_func=self.telemetry_endpoint)
        self.app.add_url_rule("/STATUS", methods=["POST"], view_func=self.status_endpoint)
        self.app.add_url_rule("/MOTOR", methods=["POST"], view_func=self.motor_endpoint)

    def register(self):
        """
        Endpoint per la registrazione del dispositivo.
        Attende un JSON del tipo: {"ip": "http://<IP_VEICOLO>:<PORTA>"}
        """
        data = request.get_json()
        if not data or "ip" not in data:
            return jsonify({"error": "Missing 'ip' field in JSON"}), 400

        try:
            self.device_manager.register_device(data["ip"])
        except ValueError as ve:
            return jsonify({"error": str(ve)}), 400

        return jsonify({"message": "Registration successful"}), 200

    def telemetry_endpoint(self):
        """
        Endpoint per aggiornare i valori di telemetria.
        Attende un JSON del tipo:
            {
              "roll": <float>,
              "pitch": <float>,
              "yaw": <float>,
              "altitude": <float>
            }
        """
        data = request.get_json(force=True)
        print(f"[GroundStation] Nuovi dati TELEMETRY: {data}")

        r = float(data.get("roll", 0.0))
        p = float(data.get("pitch", 0.0))
        y = float(data.get("yaw", 0.0))
        a = float(data.get("altitude", 0.0))
        self.realtime_data.update_telemetry(r, p, y, a)
        return jsonify({"status": "ok", "received": data}), 200

    def status_endpoint(self):
        """
        Endpoint per aggiornare lo stato.
        Attende un JSON del tipo: {"state": <string>}
        """
        data = request.get_json(force=True)
        print(f"[GroundStation] Nuovi dati STATUS: {data}")
        s = data.get("state", "WAIT")
        self.realtime_data.update_state(s)
        return jsonify({"status": "ok", "received": data}), 200
        
        
    def motor_endpoint(self):
        """
        Endpoint per aggiornare lo stato.
        Attende un JSON del tipo: {"state": <string>}
        """
        data = request.get_json(force=True)
        print(f"[GroundStation] Nuovi dati STATUS: {data}")
        
        m1 = float(data.get("M1", 0.0))
        m2 = float(data.get("M2", 0.0))
        m3 = float(data.get("M3", 0.0))
        m4 = float(data.get("M4", 0.0))
        self.realtime_data.update_motor(m1, m2, m3, m4)
        
        
        
        self.realtime_data.update_state(s)
        return jsonify({"status": "ok", "received": data}), 200


    def run(self):
        self.app.run(host=self.host, port=self.port, debug=False, use_reloader=False)


# -----------------------------------------------------------------------------
# DronePlotter: contiene le funzioni per il calcolo della rotazione e il disegno 3D
# -----------------------------------------------------------------------------
class DronePlotter:
    def __init__(self, ax, canvas):
        self.ax = ax
        self.canvas = canvas

    @staticmethod
    def rotation_about_axis(angle, axis):
        """
        Restituisce la matrice 3x3 di rotazione di 'angle' radianti intorno all'asse 'axis'.
        Usa la formula di Rodrigues.
        """
        axis = np.array(axis, dtype=float)
        norm_axis = np.linalg.norm(axis)
        if norm_axis < 1e-12:
            return np.eye(3)

        axis = axis / norm_axis
        x, y, z = axis
        c = np.cos(angle)
        s = np.sin(angle)
        C = 1 - c

        K = np.array([
            [0,   -z,   y],
            [z,    0,  -x],
            [-y,   x,   0]
        ], dtype=float)

        I = np.eye(3)
        R = I + s * K + C * (K @ K)
        return R

    @staticmethod
    def rotation_matrix_from_euler(roll, pitch, yaw, degrees=True):
        """
        Calcola la matrice di rotazione combinata dagli angoli roll, pitch e yaw.
        """
        if degrees:
            roll = np.deg2rad(roll)
            pitch = np.deg2rad(pitch)
            yaw = np.deg2rad(yaw)

        # Assi personalizzati
        yaw_axis = [0, 0, 1]
        pitch_axis = [1, -1, 0]
        roll_axis = [1, 1, 0]

        Rz = DronePlotter.rotation_about_axis(yaw, yaw_axis)
        Rp = DronePlotter.rotation_about_axis(pitch, pitch_axis)
        Rr = DronePlotter.rotation_about_axis(roll, roll_axis)

        return Rz @ Rp @ Rr

    def update_plot(self, roll, pitch, yaw):
        """
        Ridisegna il drone 3D usando roll, pitch, yaw.
        """
        self.ax.clear()
        arm_length = 1.0

        # Definizione dei bracci
        x_arm = np.array([[-arm_length, 0, 0],
                          [ arm_length, 0, 0]]).T
        y_arm = np.array([[0, -arm_length, 0],
                          [0,  arm_length, 0]]).T
        arms = [x_arm, y_arm]

        # Linea diagonale
        separator_line = np.array([
            [-arm_length/2, -arm_length/2, 0],
            [ arm_length/2,  arm_length/2, 0]
        ]).T

        R = self.rotation_matrix_from_euler(roll, pitch, yaw, degrees=True)

        for arm in arms:
            rotated_arm = R @ arm
            self.ax.plot(rotated_arm[0, :],
                         rotated_arm[1, :],
                         rotated_arm[2, :],
                         color='green',
                         linewidth=2,
                         marker='o')

        rotated_line = R @ separator_line
        self.ax.plot(rotated_line[0, :],
                     rotated_line[1, :],
                     rotated_line[2, :],
                     color='red',
                     linewidth=1,
                     marker='o')

        limit = 1.5
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_zlim(-limit, limit)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title(f"Drone 3D\nRoll={roll:.1f}°, Pitch={pitch:.1f}°, Yaw={yaw:.1f}°")
        self.ax.view_init(elev=20, azim=45)
        self.canvas.draw()


# -----------------------------------------------------------------------------
# GroundStationGUI: crea l'interfaccia grafica e gestisce gli aggiornamenti
# -----------------------------------------------------------------------------
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
# Assicurati di avere anche gli import necessari per la visualizzazione 3D, ad esempio:
# from mpl_toolkits.mplot3d import Axes3D

import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Assumiamo che QuadCopterData, DeviceManager e DronePlotter siano già definiti

import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Assumiamo che QuadCopterData, DeviceManager e DronePlotter siano già definiti altrove

class GroundStationGUI:
    def __init__(self, realtime_data: 'QuadCopterData', device_manager: 'DeviceManager'):
        self.realtime_data = realtime_data
        self.device_manager = device_manager
        self.root = tk.Tk()
        self.root.title("Ground Station - Single Device")
        self.root.minsize(800, 600)
        self._build_gui()  # La creazione dell'interfaccia include anche la creazione di drone_plotter

    def _build_gui(self):
        # Impostazione del tema per un aspetto più moderno
        style = ttk.Style()
        style.theme_use('clam')
        self.root.option_add("*tearOff", False)

        # Frame principale con tre colonne (sinistra, centrale, destra)
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky="nsew")
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        # Colonna sinistra: pulsanti START/STOP
        left_frame = ttk.Frame(main_frame, borderwidth=2, relief="groove", padding=5)
        left_frame.grid(row=0, column=0, sticky="ns", padx=5, pady=5)

        # Colonna centrale: slider, valori di stato e plot
        center_frame = ttk.Frame(main_frame, borderwidth=2, relief="groove", padding=5)
        center_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        main_frame.grid_columnconfigure(1, weight=1)

        # Colonna destra: sezione Variable Settings
        right_frame = ttk.Frame(main_frame, borderwidth=2, relief="groove", padding=5)
        right_frame.grid(row=0, column=2, sticky="ns", padx=5, pady=5)

        # ----------------- Left Frame: START/STOP -----------------
        self.start_button = ttk.Button(left_frame, text="START", command=self.on_start_click)
        self.start_button.pack(fill="x", padx=5, pady=(0, 10))
        self.stop_button = ttk.Button(left_frame, text="STOP", command=self.on_stop_click, state="disabled")
        self.stop_button.pack(fill="x", padx=5, pady=10)

        # ----------------- Center Frame: THROTTLE, Status e Plot -----------------
        # --- Slider THROTTLE (smooth, con tk.Scale e resolution=0.1) ---
        throttle_label = ttk.Label(center_frame, text="THROTTLE (0 - 100):")
        throttle_label.grid(row=0, column=0, padx=5, pady=(5, 0), sticky="w")
        self.throttle_scale = tk.Scale(center_frame, from_=0, to=100, orient="horizontal",
                                       command=self.on_throttle_change, resolution=0.1)
        self.throttle_scale.set(0)
        self.throttle_scale.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        center_frame.grid_columnconfigure(0, weight=1)

        # --- Pannello per i valori di stato ---
        values_frame = ttk.LabelFrame(center_frame, text="Status Values", padding=10)
        values_frame.grid(row=2, column=0, sticky="ew", padx=5, pady=5)
        self.yaw_value_label = ttk.Label(values_frame, text="Yaw: 0.00")
        self.yaw_value_label.grid(row=0, column=0, sticky="w")
        self.pitch_value_label = ttk.Label(values_frame, text="Pitch: 0.00")
        self.pitch_value_label.grid(row=1, column=0, sticky="w")
        self.roll_value_label = ttk.Label(values_frame, text="Roll: 0.00")
        self.roll_value_label.grid(row=2, column=0, sticky="w")
        self.altitude_value_label = ttk.Label(values_frame, text="Altitude: 0.00")
        self.altitude_value_label.grid(row=3, column=0, sticky="w")
        self.state_value_label = ttk.Label(values_frame, text="State: WAIT")
        self.state_value_label.grid(row=4, column=0, sticky="w")

        # --- Pannello per i plot (3D e istogramma) ---
        plots_frame = ttk.Frame(center_frame)
        plots_frame.grid(row=3, column=0, sticky="nsew", padx=5, pady=5)
        center_frame.grid_rowconfigure(3, weight=1)

        # Plot 3D del drone
        drone_3d_frame = ttk.Frame(plots_frame, borderwidth=2, relief="sunken")
        drone_3d_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        plots_frame.grid_rowconfigure(0, weight=1)
        plots_frame.grid_columnconfigure(0, weight=1)
        fig = plt.Figure(figsize=(5, 4))
        self.ax = fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(fig, master=drone_3d_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        # **Creiamo drone_plotter qui, prima di chiamare update_plot**
        self.drone_plotter = DronePlotter(self.ax, self.canvas)
        self.drone_plotter.update_plot(0, 0, 0)

        # Istogramma per i motori
        hist_frame = ttk.Frame(plots_frame, borderwidth=2, relief="sunken")
        hist_frame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        fig_hist = plt.Figure(figsize=(5, 2))
        self.ax_hist = fig_hist.add_subplot(111)
        self.hist_canvas = FigureCanvasTkAgg(fig_hist, master=hist_frame)
        self.hist_canvas.get_tk_widget().pack(fill="both", expand=True)

        # ----------------- Right Frame: Variable Settings -----------------
        settings_frame = ttk.LabelFrame(right_frame, text="Variable Settings", padding=10)
        settings_frame.pack(fill="both", expand=True, padx=5, pady=5)
        # Casella di testo per inserire il valore numerico
        value_label = ttk.Label(settings_frame, text="Value:")
        value_label.grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.variable_value_entry = ttk.Entry(settings_frame)
        self.variable_value_entry.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        # Menù a tendina per selezionare la variabile da modificare
        variable_label = ttk.Label(settings_frame, text="Variable:")
        variable_label.grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.variable_combobox = ttk.Combobox(settings_frame,
                                              values=["ALTITUDE", "MAXPWM", "THROTTLE", "SPEED", "BATTERY"])
        self.variable_combobox.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        self.variable_combobox.current(0)
        # Bottone per inviare il comando
        send_button = ttk.Button(settings_frame, text="Send Command", command=self.on_send_command)
        send_button.grid(row=2, column=0, columnspan=2, pady=10)
        settings_frame.columnconfigure(1, weight=1)

    def on_start_click(self):
        self.device_manager.send_command("STATE", "START", 0.0)
        self.start_button.config(state="disabled")
        self.stop_button.config(state="normal")

    def on_stop_click(self):
        self.device_manager.send_command("STATE", "STOP", 0.0)
        self.stop_button.config(state="disabled")
        self.start_button.config(state="normal")

    def on_throttle_change(self, value):
        try:
            val = float(value)
            self.device_manager.send_command("SET", "THROTTLE", val)
        except ValueError:
            pass

    def on_send_command(self):
        """Legge il valore e la variabile selezionata e invia il comando SET."""
        try:
            value = float(self.variable_value_entry.get())
        except ValueError:
            print("Valore numerico non valido")
            return
        variable = self.variable_combobox.get()
        self.device_manager.send_command("SET", variable, value)

    def update_histogram(self):
        """Aggiorna l'istogramma con i valori dei motori."""
        m1 = self.realtime_data.pwm1
        m2 = self.realtime_data.pwm2
        m3 = self.realtime_data.pwm3
        m4 = self.realtime_data.pwm4
        values = [m1, m2, m3, m4]

        self.ax_hist.cla()
        self.ax_hist.bar(['M1', 'M2', 'M3', 'M4'], values, color='blue')
        self.ax_hist.set_ylim(900, 2100)
        self.ax_hist.set_title('Valori Motori')
        self.ax_hist.set_ylabel('Valore')
        self.hist_canvas.draw()

    def poll_telemetry(self):
        """Aggiorna periodicamente la GUI con i dati di telemetria."""
        state_val = self.realtime_data.state
        roll_val = self.realtime_data.roll
        pitch_val = self.realtime_data.pitch
        yaw_val = self.realtime_data.yaw
        alt_val = self.realtime_data.altitude

        self.state_value_label.config(text=f"State: {state_val}")
        self.roll_value_label.config(text=f"Roll: {roll_val:.2f}")
        self.pitch_value_label.config(text=f"Pitch: {pitch_val:.2f}")
        self.yaw_value_label.config(text=f"Yaw: {yaw_val:.2f}")
        self.altitude_value_label.config(text=f"Altitude: {alt_val:.2f}")

        if state_val in ["WAIT", "READY"]:
            self.start_button.config(state="normal")
            self.stop_button.config(state="disabled")
        elif state_val == "FLYING":
            self.start_button.config(state="disabled")
            self.stop_button.config(state="normal")

        self.drone_plotter.update_plot(roll_val, pitch_val, yaw_val)
        #self.update_histogram()

        self.root.after(100, self.poll_telemetry)

    def run(self):
        self.root.after(100, self.poll_telemetry)
        self.root.mainloop()
        
# realtime_data = QuadCopterData(...)
# device_manager = DeviceManager(...)
# gui = GroundStationGUI(realtime_data, device_manager)
# gui.run()




# -----------------------------------------------------------------------------
# GroundStationApp: inizializza e avvia il server Flask e la GUI Tkinter
# -----------------------------------------------------------------------------
class GroundStationApp:
    def __init__(self):
        self.realtime_data = QuadCopterData()
        self.device_manager = DeviceManager()
        self.gui = GroundStationGUI(self.realtime_data, self.device_manager)
        self.flask_server = FlaskServer(self.realtime_data, self.device_manager)

    def start_flask(self):
        server_thread = threading.Thread(target=self.flask_server.run, daemon=True)
        server_thread.start()

    def run(self):
        self.start_flask()
        self.gui.run()


# -----------------------------------------------------------------------------
# Funzione main
# -----------------------------------------------------------------------------
def main():
    app = GroundStationApp()
    app.run()


if __name__ == "__main__":
    main()
