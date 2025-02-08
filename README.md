Questo repository contiene il codice per un progetto completo di quadricottero realizzato da zero. Il sistema è suddiviso in 3 componenti principali, ognuno collocato in una cartella dedicata:

## 📂 Struttura del Progetto

flightcontroller/ (Arduino/ESP32):Contiene il codice che gira sul microcontrollore del quadricottero. Gestisce la lettura dei sensori (IMU, sonar), l'elaborazione dei dati tramite PID, il controllo dei motori e la gestione degli stati di volo attraverso una macchina a stati (FSM). La comunicazione avviene con un protocollo personalizzato basato su pacchetti con start/stop e checksum.

FlightStation/ (Raspberry Pi): Attualmente composto da un singolo modulo intermedio, scritto in Python, si occupa della comunicazione tra il flightcontroller e la GroundStation. Legge i dati in ingresso dalla porta seriale (UART) e li trasforma in pacchetti HTTP, e viceversa: riceve comandi via HTTP dalla GroundStation, li impacchetta e li invia via seriale al flightcontroller.
In futuro ci saranno vari moduli.

groundstation/ (PC):La GroundStation è l’interfaccia utente per monitorare e controllare il quadricottero. Anch'essa è realizzata in Python e utilizza Flask per il server e Tkinter (insieme a matplotlib) per una GUI intuitiva. Qui vengono visualizzati in tempo reale i dati di telemetria (roll, pitch, yaw, altitudine), lo stato del volo e i segnali PWM dei motori, oltre ad offrire la possibilità di inviare comandi (ad es. START, STOP, modifiche di throttle e altre variabili).

# 🔧 Panoramica delle Funzionalità

## FlightController | # (Arduino/ESP32)

✅ Acquisizione dati da sensori IMU e sonar

✅ Algoritmi PID per stabilizzazione

✅ Macchina a stati (FSM) per il controllo del volo

✅ Comunicazione via protocollo seriale personalizzato

## FlightStation | # (Raspberry Pi)

✅ Gestione della comunicazione tra il flightcontroller e la GroundStation

✅ Conversione dati da seriale a HTTP e viceversa

✅ Ascolto e inoltro dei comandi ricevuti

## GroundStation | # (PC)

✅ GUI interattiva con monitoraggio in tempo reale

✅ Invio comandi di controllo (START, STOP, SET, ecc.)

✅ Ricezione e visualizzazione dati telemetrici

✅ Server Flask per la gestione della comunicazione

🔜 
## TODO

🛠 Implementazione AI Vision sul Raspberry Pi per elaborare immagini e migliorare il controllo del volo.

🔜  Implementazione controllo altitudine tramite pid

⚙️ Ottimizzazione del codice C per una maggiore efficienza e passaggio da Arduino a ESP32 usando multithread.

📡 Ampliamento della GroundStation con funzionalità avanzate e miglioramenti alla GUI.


## 📁 Struttura delle Cartelle

├── flightcontroller
│   ├── main.ino               # Codice principale per Arduino/ESP32
│   ├── command.h / command.cpp  # Gestione dei comandi e del protocollo seriale
│   ├── flight_fsm.h / flight_fsm.cpp  # Macchina a stati per la logica di volo
│   ├── motor.h / motor.cpp    # Gestione dei motori e del mixing dei segnali PID
│   ├── sensors.h / sensors.cpp  # Inizializzazione e lettura dei sensori (IMU e sonar)
│   └── pid.h / pid.cpp        # Implementazione dei controllori PID
│
├── FlightStation
│   ├── SerialReader.py        # Lettura e scrittura dati via porta seriale
│   ├── PacketHandler.py       # Gestione e impacchettamento dei dati (protocollo personalizzato)
│   ├── HTTPConnection.py      # Gestione della comunicazione HTTP (invia/recupera dati)
│   └── CommunicationModule.py # Integrazione dei moduli di comunicazione
│
└── groundstation
    ├── groundstation.py       # Avvio dell'applicazione principale
    ├── FlaskServer.py         # Server Flask per ricevere dati e comandi
    ├── GroundStationGUI.py    # Interfaccia grafica (Tkinter + matplotlib) per il monitoraggio e il controllo
    └── QuadCopterData.py      # Gestione e aggiornamento dei dati di telemetria del drone




📝 Licenza

Questo progetto è rilasciato sotto [inserire qui la licenza desiderata, ad es. MIT License].
