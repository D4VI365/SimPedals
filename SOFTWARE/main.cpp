#include <Joystick.h>
#include <HX711.h>

// ==========================================
// CONFIGURAZIONE PIN
// ==========================================
const int PIN_ACCEL  = A0;
const int PIN_CLUTCH = A1;
const int PIN_DT     = 2;
const int PIN_SCK    = 3;

// ==========================================
// INIZIALIZZAZIONE OGGETTI
// ==========================================
// Creiamo un Joystick con 3 assi (X, Y, Z) e nessun pulsante
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_MULTI_AXIS, 
                   0, 0,                   // Numero di pulsanti e Hat Switch
                   true, true, true,       // Attiva asse X, Y, Z
                   false, false, false,    // Disattiva Rx, Ry, Rz
                   false, false,           // Disattiva Timone e Throttle
                   false, false, false);   // Disattiva Acceleratore, Freno, Sterzo dedicati (usiamo X,Y,Z per massima compatibilità)

HX711 scale;

// ==========================================
// PARAMETRI DI CALIBRAZIONE (DA MODIFICARE)
// ==========================================
// I sensori SS49E a riposo (nessun campo magnetico) leggono circa 512.
// Quando avvicini il magnete, il valore sale (es. fino a 850) o scende (es. fino a 200) a seconda del polo (Nord/Sud).
// DOVRAI INSERIRE I TUOI VALORI REALI DOPO AVERLI LETTI SUL SERIAL MONITOR.
int accelMin = 512;  // Valore a pedale rilasciato
int accelMax = 850;  // Valore a pedale premuto al 100%

int clutchMin = 512; // Valore a pedale rilasciato
int clutchMax = 200; // Valore a pedale premuto al 100% (se usi l'altro polo magnetico, il numero scende!)

// Cella di carico (HX711)
float hx711_calibration_factor = 100.0; // Da calcolare durante la taratura
long brakeMaxRaw = 25000;               // Lettura grezza della cella a massima frenata desiderata

// ==========================================
// VARIABILI PER FILTRO SOFTWARE (EMA)
// ==========================================
// Aiuta a stabilizzare la lettura rimuovendo i micro-sfarfallii
float filteredAccel = 0;
float filteredClutch = 0;
float alpha = 0.2; // Forza del filtro (0.0 = massimo filtro/lento, 1.0 = nessun filtro/istantaneo)

void setup() {
  Serial.begin(115200);

  // Inizializza l'HX711
  scale.begin(PIN_DT, PIN_SCK);
  scale.set_scale(hx711_calibration_factor);
  scale.tare(); // Azzera la cella di carico all'avvio del Pro Micro

  // Configura i range della periferica USB (0-1023 per massima risoluzione a 10 bit)
  Joystick.setXAxisRange(0, 1023); // Acceleratore
  Joystick.setYAxisRange(0, 1023); // Frizione
  Joystick.setZAxisRange(0, 1023); // Freno
  
  Joystick.begin();
}

void loop() {
  // --------------------------------------------------
  // 1. LETTURA E FILTRAGGIO ACCELERATORE E FRIZIONE
  // --------------------------------------------------
  int rawAccel = analogRead(PIN_ACCEL);
  int rawClutch = analogRead(PIN_CLUTCH);

  // Applico il filtro passa-basso EMA
  filteredAccel = (alpha * rawAccel) + ((1.0 - alpha) * filteredAccel);
  filteredClutch = (alpha * rawClutch) + ((1.0 - alpha) * filteredClutch);

  // Mappatura e costrizione dei valori nel range USB (0-1023)
  // constrain() è vitale: impedisce che il valore vada sotto lo 0 se il pedale va leggermente oltre il limite meccanico
  int mappedAccel = map(filteredAccel, accelMin, accelMax, 0, 1023);
  mappedAccel = constrain(mappedAccel, 0, 1023);

  int mappedClutch = map(filteredClutch, clutchMin, clutchMax, 0, 1023);
  mappedClutch = constrain(mappedClutch, 0, 1023);

  // --------------------------------------------------
  // 2. LETTURA CELLA DI CARICO (FRENO)
  // --------------------------------------------------
  int mappedBrake = 0;
  
  // scale.is_ready() evita che l'Arduino si blocchi aspettando l'HX711
  if (scale.is_ready()) {
    long rawBrake = scale.get_units(1); // Legge 1 campione
    
    // Se la cella va in negativo (perché l'hai montata al contrario), usa abs(rawBrake)
    mappedBrake = map(rawBrake, 0, brakeMaxRaw, 0, 1023);
    mappedBrake = constrain(mappedBrake, 0, 1023);
  }

  // --------------------------------------------------
  // 3. INVIO DATI AL PC TRAMITE USB
  // --------------------------------------------------
  Joystick.setXAxis(mappedAccel);
  Joystick.setYAxis(mappedClutch);
  Joystick.setZAxis(mappedBrake);

  // (OPZIONALE) Stampa per la calibrazione - Decommenta per leggere i valori grezzi
  /*
  Serial.print("Raw Accel: "); Serial.print(rawAccel);
  Serial.print(" | Raw Clutch: "); Serial.print(rawClutch);
  Serial.print(" | Raw Brake: "); Serial.println(scale.get_units(1));
  */

  delay(10); // Piccolo delay per stabilità USB (10ms = 100Hz di aggiornamento, ottimo per i pedali)
}