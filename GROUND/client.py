import tkinter as tk
from tkinter import ttk
import requests
import numpy as np
import matplotlib
matplotlib.use("TkAgg")  # Forza l'uso di TkAgg come backend
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D  # Import necessario per 3D


# URL della FlightStation 
FLIGHT_URL = "http://192.168.1.197:5001/command"
# URL della GroundStation 
GROUND_URL = "http://192.168.1.197:5001/command"


# ---- Dizionario per salvare i valori di telemetria ricevuti dal mini-server Flask ----
# Viene aggiornato dal thread Flask, e letto dalla GUI con 'root.after' periodico.
shared_telemetry = {
    "state": "WAIT",

    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "altitude": 0.0,

    "altitude_target": 1.0
}


# -----------------------------------------------------------------------------------
# Funzione generica per inviare comandi al server
# -----------------------------------------------------------------------------------
def send_command(cmd_type, cmd_info, cmd_value):
    data = {
        "command_type": cmd_type,
        "command_info": cmd_info,
        "command_value": cmd_value
    }
    try:
        response = requests.post(FLIGHT_URL, json=data)
        if response.status_code == 200:
            print("Comando inviato correttamente:", data)
        else:
            print(f"Errore nell'invio: {response.status_code} - {response.text}")
    except Exception as e:
        print(f"Eccezione durante l'invio: {e}")



# -----------------------------------------------------------------------------------
# Matrici di rotazione e funzione per disegno drone 3D
# -----------------------------------------------------------------------------------
import numpy as np

def rotation_about_axis(angle, axis):
    """
    Restituisce la matrice 3x3 di rotazione di 'angle' radianti
    intorno all'asse 'axis' (vettore [x,y,z]).
    Usa la formula di Rodrigues:
      R = I + sin(a)*K + (1 - cos(a))*K^2
    dove K è la matrice skew-symmetric associata ad 'axis' normalizzato.
    """
    axis = np.array(axis, dtype=float)
    norm_axis = np.linalg.norm(axis)
    if norm_axis < 1e-12:
        # Asse nullo -> matrice identità
        return np.eye(3)

    # Normalizziamo l'asse
    axis = axis / norm_axis
    x, y, z = axis
    c = np.cos(angle)
    s = np.sin(angle)
    C = 1 - c

    # Matrice skew-symmetric di axis
    K = np.array([
        [   0,  -z,   y],
        [   z,   0,  -x],
        [  -y,   x,   0]
    ], dtype=float)

    # R = I + s*K + C*K^2
    I = np.eye(3)
    K2 = K @ K
    R = I + s*K + C*K2
    return R


def rotation_matrix_from_euler(roll, pitch, yaw, degrees=True):
    """
    Rotazione 3D con:
      - yaw attorno a Z,
      - pitch attorno a (1,-1,0),
      - roll attorno a (1, 1, 0).
    Ordine: Rz(yaw) * Rp(pitch) * Rr(roll).
    """
    if degrees:
        roll  = np.deg2rad(roll)
        pitch = np.deg2rad(pitch)
        yaw   = np.deg2rad(yaw)

    # Asse Yaw (Z)
    yaw_axis = [0, 0, 1]
    # Asse Pitch (1, -1, 0)
    pitch_axis = [1, -1, 0]
    # Asse Roll (1, 1, 0)
    roll_axis = [1, 1, 0]

    # Creiamo le tre matrici di rotazione
    Rz = rotation_about_axis(yaw,   yaw_axis)   # Yaw intorno a Z
    Rp = rotation_about_axis(pitch, pitch_axis) # Pitch intorno a (1,-1,0)
    Rr = rotation_about_axis(roll,  roll_axis)  # Roll intorno a (1,1,0)

    # Componiamo (moltiplichiamo da sinistra a destra in quest'ordine)
    # => Rz * Rp * Rr
    R = Rz @ Rp @ Rr
    return R




def update_drone_plot(roll, pitch, yaw):
    """
    Ridisegna il drone 3D con:
      - due linee verdi ortogonali (assi X e Y) sul piano XY,
      - una linea rossa sul piano XY che taglia diagonalmente i bracci,
        considerandola come nuovo asse "frontale" per pitch/roll.
    Applichiamo le stesse rotazioni Rz(yaw)*Ry(pitch)*Rx(roll) a tutti i segmenti.
    """

    ax.clear()

    # --- Definizione dei bracci verdi (piano XY) ---
    arm_length = 1.0

    # Braccio X da (-1,0,0) a (1,0,0)
    x_arm = np.array([[-arm_length, 0, 0],
                      [ arm_length, 0, 0]]).T  # shape (3,2)

    # Braccio Y da (0,-1,0) a (0,1,0)
    y_arm = np.array([[0, -arm_length, 0],
                      [0,  arm_length, 0]]).T  # shape (3,2)

    arms = [x_arm, y_arm]

    # --- Definizione della linea rossa sul piano XY ---
    # La facciamo diagonale, da (-1,-1,0) a (1,1,0),
    # così "divide a metà" la croce formata da x_arm e y_arm.
    separator_line = np.array([
        [-arm_length/2, -arm_length/2, 0],
        [ arm_length/2,  arm_length/2, 0]
    ]).T  # shape (3,2)

    # --- Calcolo della matrice di rotazione ---
    R = rotation_matrix_from_euler(roll, pitch, yaw, degrees=True)

    # --- Disegno delle linee verdi (X e Y) ---
    for arm in arms:
        rotated_arm = R @ arm
        ax.plot(
            rotated_arm[0, :],
            rotated_arm[1, :],
            rotated_arm[2, :],
            color='green',
            linewidth=2,
            marker='o'
        )

    # --- Disegno della linea rossa (nuovo "asse") ---
    rotated_line = R @ separator_line
    ax.plot(
        rotated_line[0, :],
        rotated_line[1, :],
        rotated_line[2, :],
        color='red',
        linewidth=1,
        marker='o'
    )

    # --- Impostazioni finali del grafico ---
    limit = 1.5
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f"Drone 3D\nRoll={roll:.1f}°, Pitch={pitch:.1f}°, Yaw={yaw:.1f}°")

    # Vista 3D
    ax.view_init(elev=20, azim=45)

    canvas.draw()


# -----------------------------------------------------------------------------------
# Funzioni di callback per pulsanti START/STOP
# -----------------------------------------------------------------------------------
def on_start_click():
    send_command("STATE", "START", 0.0)
    start_button.config(state="disabled")
    stop_button.config(state="normal")

def on_stop_click():
    send_command("STATE", "STOP", 0.0)
    stop_button.config(state="disabled")
    start_button.config(state="normal")

# -----------------------------------------------------------------------------------
# Funzioni di callback slider THROTTLE (0-100) e ALTITUDE (1-5)
# -----------------------------------------------------------------------------------
def on_throttle_change(value):
    """Invia un comando SET -> THROTTLE = value"""
    try:
        val = float(value)
        send_command("SET", "THROTTLE", val)
    except ValueError:
        pass

def on_altitude_change(value):
    """
    Invio comando SET -> ALTITUDE = value
    e aggiorno l'etichetta 'Altitude'
    """
    global alt_val
    try:
        alt_val = float(value)
        send_command("SET", "ALTITUDE", alt_val)
        #altitude_value_label.config(text=f"Altitude: {alt_val:.2f}")
    except ValueError:
        pass


# -----------------------------------------------------------------------------------
# Creazione interfaccia grafica
# -----------------------------------------------------------------------------------
root = tk.Tk()
root.title("Ground Station Command Sender")
root.minsize(800, 600)

main_frame = ttk.Frame(root, padding="10 10 10 10")
main_frame.pack(fill="both", expand=True)

# Suddividiamo lo spazio in pannello sinistro (pulsanti) e pannello destro (slider + 3D + letture)
left_frame = ttk.Frame(main_frame, borderwidth=2, relief="groove", padding=5)
left_frame.pack(side="left", fill="y", padx=5, pady=5)

right_frame = ttk.Frame(main_frame, borderwidth=2, relief="groove", padding=5)
right_frame.pack(side="right", fill="both", expand=True, padx=5, pady=5)

# ----------------------------
# Sezione sinistra: START/STOP
# ----------------------------
start_button = ttk.Button(left_frame, text="START", command=on_start_click)
start_button.pack(fill="x", padx=5, pady=(0, 10))

stop_button = ttk.Button(left_frame, text="STOP", command=on_stop_click, state="disabled")
stop_button.pack(fill="x", padx=5, pady=10)

# ----------------------------
# Sezione destra: Slider THROTTLE, ALTITUDE, e i 3 slider (Yaw, Pitch, Roll)
# ----------------------------

# 1. Slider THROTTLE
slider_label = ttk.Label(right_frame, text="THROTTLE (0 - 100):")
slider_label.pack(pady=(0, 5))

throttle_scale = ttk.Scale(
    right_frame,
    from_=0, to=100,
    orient="horizontal",
    command=on_throttle_change,
    length=300
)
throttle_scale.pack(pady=5)

# 2. Slider ALTITUDE
altitude_label = ttk.Label(right_frame, text="ALTITUDE (1 - 5):")
altitude_label.pack(pady=(20, 5))

altitude_scale = ttk.Scale(
    right_frame,
    from_=1, to=5,
    orient="horizontal",
    command=on_altitude_change,
    length=300
)
altitude_scale.set(1)  # Valore iniziale
altitude_scale.pack(pady=5)

# 3. Slider YAW, PITCH, ROLL (per grafica 3D)


# -----------------------------
# Sezione lettura valori (Yaw, Pitch, Roll, Altitude)
# -----------------------------
values_frame = ttk.LabelFrame(right_frame, text="Status Values", padding=10)
values_frame.pack(fill="x", expand=False, padx=5, pady=15)

yaw_value_label = ttk.Label(values_frame, text="Yaw: 0.00")
yaw_value_label.pack(anchor="w")

pitch_value_label = ttk.Label(values_frame, text="Pitch: 0.00")
pitch_value_label.pack(anchor="w")

roll_value_label = ttk.Label(values_frame, text="Roll: 0.00")
roll_value_label.pack(anchor="w")

altitude_value_label = ttk.Label(values_frame, text="Altitude: 0.00")
altitude_value_label.pack(anchor="w")

state_value_label = ttk.Label(values_frame, text="State: WAIT")
state_value_label.pack(anchor="w")

# -------------------------------------------
# Sezione DRONE 3D (Matplotlib embedded)
# -------------------------------------------
drone_3d_frame = ttk.Frame(right_frame, borderwidth=2, relief="sunken")
drone_3d_frame.pack(fill="both", expand=True, padx=5, pady=5)

# Creiamo la figura Matplotlib e la inseriamo nel Frame
fig = plt.Figure(figsize=(5,5))
ax = fig.add_subplot(111, projection='3d')
canvas = FigureCanvasTkAgg(fig, master=drone_3d_frame)
canvas.get_tk_widget().pack(fill="both", expand=True)

# Disegno iniziale del drone
update_drone_plot(0, 0, 0)


 #-----------------------------------------------------------------------------------
# 6) Mini-server Flask per ricevere telemetria
# -----------------------------------------------------------------------------------
from flask import Flask, request, jsonify
import threading


flask_app = Flask(__name__)

@flask_app.route('/telemetry', methods=['POST'])
def telemetry_endpoint():
    """
    Riceve un JSON del tipo:
    {
      "roll": <float>,
      "pitch": <float>,
      "yaw": <float>,
      "altitude": <float>
    }
    e aggiorna la struttura shared_telemetry.
    """
    data = request.get_json(force=True)  # force=True per sicurezza
    # Estrai i valori (con default 0.0 se non presenti)
    #s = data.get("state", "WAIT")
    r = float(data.get("roll", 0.0))
    p = float(data.get("pitch", 0.0))
    y = float(data.get("yaw", 0.0))
    a = float(data.get("altitude", 0.0))
    at = float(data.get("altitude_target", 0.0))

    # Aggiorna il dizionario condiviso
    #shared_telemetry["state"] = s
    shared_telemetry["roll"] = r
    shared_telemetry["pitch"] = p
    shared_telemetry["yaw"] = y
    shared_telemetry["altitude"] = a
    #shared_telemetry["altitude_target"] = at

    #print(f"[Flask] Telemetria ricevuta: roll={r}, pitch={p}, yaw={y}, altitude={a}")
    return jsonify({"status": "ok", "received": data}), 200

@flask_app.route('/status', methods=['POST'])
def status_endpoint():
    """
    Riceve un JSON del tipo:
    {
      "state": <string>
    }
    e aggiorna la struttura shared_telemetry.
    """
    data = request.get_json(force=True)  # force=True per sicurezza 
    # Estrai i valori (con default 0.0 se non presenti)
    s = data.get("state", "WAIT")
    # Aggiorna il dizionario condiviso
    shared_telemetry["state"] = s

    if s == "WAIT":
        start_button.config(state="normal")
        stop_button.config(state="disabled")
    elif s == "READY":
        stop_button.config(state="disabled")
        start_button.config(state="normal")
    elif s == "FLYING":
        start_button.config(state="disabled")
        stop_button.config(state="normal")
    


    return jsonify({"status": "ok", "received": data}), 200

# -----------------------------------------------------------------------------------
# 7) Funzione per sincronizzare i dati di telemetria (thread-safe)
# -----------------------------------------------------------------------------------
roll_val, pitch_val, yaw_val, alt_val, target_alt_val = 0.0, 0.0, 0.0, 0.0, 0.0
state_val = "WAIT"

def poll_telemetry():
    """
    Questa funzione gira nel main thread Tkinter (grazie a root.after).
    Legge i valori in shared_telemetry e aggiorna la GUI (slider e label).
    Viene richiamata periodicamente (es. ogni 200 ms).
    """
    global roll_val, pitch_val, yaw_val, alt_val, state_val
    # Leggiamo i dati dalla struttura condivisa

    # Se ci sono cambiamenti significativi, aggiorniamo la GUI
    # (Oppure aggiorniamo sempre, come esempio)
    state_val = shared_telemetry["state"]
    roll_val = shared_telemetry["roll"]
    pitch_val = shared_telemetry["pitch"]
    yaw_val = shared_telemetry["yaw"]
    alt_val = shared_telemetry["altitude"]


    # Aggiorna slider (se vuoi che si muovano da soli) - attento al range
    # roll_scale.set(roll_val)  # se vogliamo far muovere lo slider
    # pitch_scale.set(pitch_val)
    # yaw_scale.set(yaw_val)
    # altitude_scale.set(alt_val)  # se alt_val è tra 1 e 5

    # Aggiorna label
    state_value_label.config(text=f"State: {state_val}")
    roll_value_label.config(text=f"Roll: {roll_val:.2f}")
    pitch_value_label.config(text=f"Pitch: {pitch_val:.2f}")
    yaw_value_label.config(text=f"Yaw: {yaw_val:.2f}")
    altitude_value_label.config(text=f"Altitude: {alt_val:.2f}")

    # Ridisegna il drone 3D
    update_drone_plot(roll_val, pitch_val, yaw_val)

    # Richiama questa funzione tra 200 ms
    root.after(200, poll_telemetry)

# -----------------------------------------------------------------------------------
# 8) Avvio del server Flask in un thread separato
# -----------------------------------------------------------------------------------
def run_flask_server():
    flask_app.run(host="0.0.0.0", port=6000, debug=False, use_reloader=False)

# Creiamo il thread e lo avviamo come daemon
server_thread = threading.Thread(target=run_flask_server, daemon=True)
server_thread.start()

# Pianifichiamo la prima chiamata a poll_telemetry
root.after(200, poll_telemetry)
# Avviamo il mainloop di Tkinter (bloccante)
root.mainloop()








