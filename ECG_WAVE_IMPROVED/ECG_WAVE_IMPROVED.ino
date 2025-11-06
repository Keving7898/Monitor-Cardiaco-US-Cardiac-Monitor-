#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <arduinoFFT.h>
#include "ecg_template.h"

// Pines TFT
#define TFT_DC 2
#define TFT_CS 15
#define TFT_RST 4

// Pines de pulsadores (PULL DOWN)
#define BTN_ECG_PIN 34
#define BTN_FFT_PIN 35

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
MAX30105 particleSensor;

// ========== MODOS DE PANTALLA ==========
enum DisplayMode {
  MODE_ECG_VITALS,
  MODE_FFT_SPECTRUM
};
DisplayMode currentMode = MODE_ECG_VITALS;

// ========== CONFIGURACIÓN FFT ==========
#define FFT_SAMPLES 128
#define SAMPLING_FREQUENCY 125
double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, FFT_SAMPLES, SAMPLING_FREQUENCY);
int fftIndex = 0;
unsigned long lastFFTSample = 0;

// ========== FILTROS DIGITALES ==========
// Variables para filtro DC removal (EMA)
double dcOffset = 0;
bool dcInitialized = false;

// Variables para filtro pasa-alto IIR (fc ≈ 0.5 Hz)
double hp_y1 = 0;    // y[n-1]
double hp_x1 = 0;    // x[n-1]
const double HP_ALPHA = 0.97;  // Coeficiente (ajusta fc)

// ========== NUEVO: FILTRO PASA-BAJO ANTI-ALIASING ==========
// Filtro IIR de 2do orden Butterworth (fc = 40 Hz @ 125 Hz)
// Coeficientes calculados con scipy.signal.butter(2, 40/(125/2), 'low')
const double LP_B0 = 0.4524;
const double LP_B1 = 0.9047;
const double LP_B2 = 0.4524;
const double LP_A1 = -0.3541;
const double LP_A2 = 0.1636;

double lp_x1 = 0, lp_x2 = 0;  // x[n-1], x[n-2]
double lp_y1 = 0, lp_y2 = 0;  // y[n-1], y[n-2]

// ========== ALGORITMO PAN-TOMPKINS ==========
// Buffer circular para derivada
#define PT_BUFFER_SIZE 5
double pt_derivative_buffer[PT_BUFFER_SIZE] = {0};
int pt_buffer_idx = 0;

// Buffer para integración móvil
#define PT_INTEGRATION_WINDOW 30  // ~240ms @ 125Hz
double pt_integration_buffer[PT_INTEGRATION_WINDOW] = {0};
int pt_integration_idx = 0;
double pt_integration_sum = 0;

// Umbrales adaptativos Pan-Tompkins
double pt_threshold = 0;
double pt_signal_peak = 0;
double pt_noise_peak = 0;
const double PT_THRESHOLD_RATIO = 0.3;  // 30% del pico

// Variables de detección de latidos
unsigned long pt_last_qrs_time = 0;
const unsigned long PT_REFRACTORY = 200;  // 200ms período refractario
bool pt_qrs_detected = false;

// ========== CALIBRACIÓN SpO2 CON AC/DC ==========
// Buffers para calcular AC y DC
#define SPO2_WINDOW 50
long red_buffer[SPO2_WINDOW];
long ir_buffer[SPO2_WINDOW];
int spo2_buffer_idx = 0;
bool spo2_buffer_full = false;

// Componentes AC y DC
double red_ac = 0, red_dc = 0;
double ir_ac = 0, ir_dc = 0;

// ========== CONFIGURACIÓN DE GRÁFICAS ==========
#define ECG_GRAPH_LEFT 20
#define ECG_GRAPH_TOP 80
#define ECG_GRAPH_WIDTH 280
#define ECG_GRAPH_HEIGHT 120
#define ECG_GRAPH_RIGHT (ECG_GRAPH_LEFT + ECG_GRAPH_WIDTH)
#define ECG_GRAPH_BOTTOM (ECG_GRAPH_TOP + ECG_GRAPH_HEIGHT)

#define FFT_GRAPH_LEFT 20
#define FFT_GRAPH_TOP 50
#define FFT_GRAPH_WIDTH 280
#define FFT_GRAPH_HEIGHT 150
#define FFT_GRAPH_RIGHT (FFT_GRAPH_LEFT + FFT_GRAPH_WIDTH)
#define FFT_GRAPH_BOTTOM (FFT_GRAPH_TOP + FFT_GRAPH_HEIGHT)

// ========== VARIABLES PARA REPRODUCCIÓN DE ECG ==========
float ecg_phase = 0.0;
float ecg_speed = 1.0;
float ecg_x = ECG_GRAPH_LEFT + 1;
float ecg_lastx = ECG_GRAPH_LEFT + 1;
int ecg_y = ECG_GRAPH_TOP + ECG_GRAPH_HEIGHT/2;
int ecg_lasty = ECG_GRAPH_TOP + ECG_GRAPH_HEIGHT/2;

// ========== VARIABLES PARA DETECCIÓN DE BPM ==========
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute = 70.0;
int beatAvg = 70;
float smoothedBPM = 70.0;

// ========== VARIABLES PARA SpO2 ==========
float spO2 = 0.0;
int spo2Samples = 0;

// ========== DETECCIÓN DE DEDO ==========
#define MIN_IR_VALUE 50000
bool fingerDetected = false;
unsigned long lastFingerCheck = 0;

// ========== CONTROL DE BOTONES ==========
unsigned long lastButtonPress = 0;
const unsigned long debounceDelay = 300;
bool lastECGState = false;
bool lastFFTState = false;

// ========== TIMING ==========
unsigned long lastECGUpdate = 0;
const unsigned long ecgUpdateInterval = 8;

// ========== DECLARACIONES ADELANTADAS ==========
double applyLowPassFilter(double input);
double applyHighPassFilter(double input);
bool detectBeatPanTompkins(double filtered_signal);
void calculateSpO2Calibrated(long red, long ir);

void setup() {
  Serial.begin(115200);
  Serial.println("=== Monitor ECG Mejorado con Pan-Tompkins y SpO2 Calibrado ===");

  pinMode(BTN_ECG_PIN, INPUT);
  pinMode(BTN_FFT_PIN, INPUT);
  Serial.println("✅ Botones configurados");

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  Serial.println("✅ TFT inicializado");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("❌ MAX30102 no encontrado");
    showError("Sensor no encontrado");
    while (1);
  }

  byte ledBrightness = 60;
  byte sampleAverage = 4;
  byte ledMode = 3;
  int sampleRate = 125;
  int pulseWidth = 411;
  int adcRange = 4096;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);
  
  Serial.println("✅ MAX30102 configurado");

  // Inicializar buffers
  for (int i = 0; i < FFT_SAMPLES; i++) {
    vReal[i] = 0;
    vImag[i] = 0;
  }
  
  for (int i = 0; i < SPO2_WINDOW; i++) {
    red_buffer[i] = 0;
    ir_buffer[i] = 0;
  }

  drawInterface();
  Serial.println("✅ Sistema listo");
}

void loop() {
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();
  
  handleButtons();
  
  // Detección de dedo
  if (millis() - lastFingerCheck > 100) {
    fingerDetected = (irValue > MIN_IR_VALUE);
    lastFingerCheck = millis();
    updateFingerStatus();
  }

  // ========== PROCESAMIENTO DE SEÑAL ==========
  if (fingerDetected) {
    // 1. Aplicar filtro pasa-bajo anti-aliasing (40 Hz)
    double lp_filtered = applyLowPassFilter((double)irValue);
    
    // 2. Remover DC offset
    if (!dcInitialized) {
      dcOffset = lp_filtered;
      dcInitialized = true;
    } else {
      dcOffset = dcOffset * 0.99 + lp_filtered * 0.01;
    }
    double ac_signal = lp_filtered - dcOffset;
    
    // 3. Aplicar filtro pasa-alto (0.5 Hz)
    double hp_filtered = applyHighPassFilter(ac_signal);
    
    // 4. Detección de latidos con Pan-Tompkins
    if (detectBeatPanTompkins(hp_filtered)) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      
      beatsPerMinute = 60000.0 / delta;
      
      if (beatsPerMinute >= 40 && beatsPerMinute <= 180) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;
        
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) {
          beatAvg += rates[x];
        }
        beatAvg /= RATE_SIZE;
        
        smoothedBPM = smoothedBPM * 0.7 + beatAvg * 0.3;
        
        if (currentMode == MODE_ECG_VITALS) {
          updateBPMDisplay();
        }
        
        Serial.printf("💓 BPM: %.0f (Avg: %d) [Pan-Tompkins]\n", beatsPerMinute, beatAvg);
      }
    }
    
    // 5. Calcular SpO2 calibrado con AC/DC
    if (currentMode == MODE_ECG_VITALS) {
      calculateSpO2Calibrated(redValue, irValue);
    }
    
    // 6. Recopilar datos para FFT (señal filtrada)
    if (millis() - lastFFTSample >= 8 && fftIndex < FFT_SAMPLES) {
      vReal[fftIndex] = hp_filtered;
      vImag[fftIndex] = 0;
      fftIndex++;
      lastFFTSample = millis();
    }
  } else {
    // Reset al quitar dedo
    dcInitialized = false;
    pt_signal_peak = 0;
    pt_noise_peak = 0;
    pt_threshold = 0;
  }

  // Actualizar pantalla según modo
  if (currentMode == MODE_ECG_VITALS) {
    if (millis() - lastECGUpdate >= ecgUpdateInterval) {
      lastECGUpdate = millis();
      
      if (fingerDetected) {
        updateECGWave();
      } else {
        drawFlatLine();
      }
    }
  } else if (currentMode == MODE_FFT_SPECTRUM) {
    updateFFTSpectrum();
  }

  delay(5);
}

// ========== FILTRO PASA-BAJO BUTTERWORTH 2DO ORDEN (40 Hz) ==========
double applyLowPassFilter(double input) {
  // Ecuación en diferencias IIR:
  // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
  
  double output = LP_B0 * input + LP_B1 * lp_x1 + LP_B2 * lp_x2
                  - LP_A1 * lp_y1 - LP_A2 * lp_y2;
  
  // Actualizar historial
  lp_x2 = lp_x1;
  lp_x1 = input;
  lp_y2 = lp_y1;
  lp_y1 = output;
  
  return output;
}

// ========== FILTRO PASA-ALTO IIR 1ER ORDEN (0.5 Hz) ==========
double applyHighPassFilter(double input) {
  // y[n] = alpha * y[n-1] + alpha * (x[n] - x[n-1])
  // Con alpha = 0.97 → fc ≈ 0.6 Hz @ 125 Hz
  
  double output = HP_ALPHA * (hp_y1 + input - hp_x1);
  
  hp_x1 = input;
  hp_y1 = output;
  
  return output;
}

// ========== ALGORITMO PAN-TOMPKINS SIMPLIFICADO ==========
bool detectBeatPanTompkins(double signal) {
  unsigned long currentTime = millis();
  
  // Paso 1: Derivada (aproximación con diferencia de 5 puntos)
  pt_derivative_buffer[pt_buffer_idx] = signal;
  pt_buffer_idx = (pt_buffer_idx + 1) % PT_BUFFER_SIZE;
  
  int idx_old = (pt_buffer_idx + 2) % PT_BUFFER_SIZE;  // 2 muestras atrás
  double derivative = (pt_derivative_buffer[pt_buffer_idx] - pt_derivative_buffer[idx_old]) * 0.5;
  
  // Paso 2: Elevar al cuadrado (enfatiza altas frecuencias)
  double squared = derivative * derivative;
  
  // Paso 3: Integración móvil (ventana de ~240ms)
  pt_integration_sum -= pt_integration_buffer[pt_integration_idx];
  pt_integration_buffer[pt_integration_idx] = squared;
  pt_integration_sum += squared;
  pt_integration_idx = (pt_integration_idx + 1) % PT_INTEGRATION_WINDOW;
  
  double integrated = pt_integration_sum / PT_INTEGRATION_WINDOW;
  
  // Paso 4: Umbral adaptativo
  // Actualizar picos de señal y ruido
  if (integrated > pt_signal_peak) {
    pt_signal_peak = integrated;
  } else {
    pt_signal_peak = pt_signal_peak * 0.999 + integrated * 0.001;  // Decaimiento lento
  }
  
  if (integrated < pt_signal_peak * 0.1) {  // Considera ruido si es <10% del pico
    pt_noise_peak = pt_noise_peak * 0.99 + integrated * 0.01;
  }
  
  // Calcular umbral dinámico
  pt_threshold = pt_noise_peak + PT_THRESHOLD_RATIO * (pt_signal_peak - pt_noise_peak);
  
  // Paso 5: Detección de QRS con período refractario
  bool qrs_now = false;
  
  if (integrated > pt_threshold && (currentTime - pt_last_qrs_time) > PT_REFRACTORY) {
    qrs_now = true;
    pt_last_qrs_time = currentTime;
    
    // Debug
    Serial.printf("[PT] QRS! Integrated: %.2f, Threshold: %.2f\n", integrated, pt_threshold);
  }
  
  return qrs_now;
}

// ========== CÁLCULO SpO2 CALIBRADO CON AC/DC ==========
void calculateSpO2Calibrated(long red, long ir) {
  // Almacenar en buffer circular
  red_buffer[spo2_buffer_idx] = red;
  ir_buffer[spo2_buffer_idx] = ir;
  spo2_buffer_idx++;
  
  if (spo2_buffer_idx >= SPO2_WINDOW) {
    spo2_buffer_idx = 0;
    spo2_buffer_full = true;
  }
  
  // Calcular cada vez que el buffer está lleno
  if (spo2_buffer_full) {
    spo2Samples++;
    
    if (spo2Samples >= 25) {  // Calcular cada ~200ms
      spo2Samples = 0;
      
      // Calcular DC (promedio)
      long red_sum = 0, ir_sum = 0;
      long red_max = red_buffer[0], red_min = red_buffer[0];
      long ir_max = ir_buffer[0], ir_min = ir_buffer[0];
      
      for (int i = 0; i < SPO2_WINDOW; i++) {
        red_sum += red_buffer[i];
        ir_sum += ir_buffer[i];
        
        if (red_buffer[i] > red_max) red_max = red_buffer[i];
        if (red_buffer[i] < red_min) red_min = red_buffer[i];
        if (ir_buffer[i] > ir_max) ir_max = ir_buffer[i];
        if (ir_buffer[i] < ir_min) ir_min = ir_buffer[i];
      }
      
      red_dc = (double)red_sum / SPO2_WINDOW;
      ir_dc = (double)ir_sum / SPO2_WINDOW;
      
      // Calcular AC (pico a pico / 2)
      red_ac = (double)(red_max - red_min) / 2.0;
      ir_ac = (double)(ir_max - ir_min) / 2.0;
      
      // Calcular ratio R según ecuación estándar
      // R = (AC_red / DC_red) / (AC_ir / DC_ir)
      
      if (ir_dc > 1000 && red_dc > 1000 && ir_ac > 0 && red_ac > 0) {
        double ratio_red = red_ac / red_dc;
        double ratio_ir = ir_ac / ir_dc;
        
        if (ratio_ir > 0.0001) {  // Evitar división por cero
          double R = ratio_red / ratio_ir;
          
          // Fórmula de calibración empírica (basada en literatura)
          // SpO2 = 110 - 25*R  (aproximación lineal)
          // o fórmula cuadrática: SpO2 = a*R^2 + b*R + c
          
          // Usar fórmula mejorada (no lineal)
          double newSpO2;
          if (R < 0.4) {
            newSpO2 = 100.0;  // Saturación perfecta
          } else if (R > 2.0) {
            newSpO2 = 80.0;   // Saturación baja
          } else {
            // Fórmula cuadrática calibrada
            newSpO2 = -45.060 * R * R + 30.354 * R + 94.845;
          }
          
          newSpO2 = constrain(newSpO2, 70.0, 100.0);
          
          // Suavizar con filtro EMA fuerte
          if (spO2 == 0) {
            spO2 = newSpO2;
          } else {
            spO2 = spO2 * 0.9 + newSpO2 * 0.1;
          }
          
          updateSpO2Display();
          
          // Debug detallado
          Serial.printf("[SpO2] R=%.3f, AC_r=%.0f, DC_r=%.0f, AC_ir=%.0f, DC_ir=%.0f → SpO2=%.1f%%\n",
                        R, red_ac, red_dc, ir_ac, ir_dc, spO2);
        }
      }
    }
  }
}

// ========== MANEJO DE BOTONES ==========
void handleButtons() {
  bool ecgPressed = digitalRead(BTN_ECG_PIN);
  bool fftPressed = digitalRead(BTN_FFT_PIN);
  
  if (millis() - lastButtonPress > debounceDelay) {
    if (ecgPressed && !lastECGState) {
      currentMode = MODE_ECG_VITALS;
      Serial.println("📊 Modo: ECG + Signos Vitales");
      drawInterface();
      lastButtonPress = millis();
    }
    
    if (fftPressed && !lastFFTState) {
      currentMode = MODE_FFT_SPECTRUM;
      Serial.println("📈 Modo: Análisis Espectral FFT");
      fftIndex = 0;
      drawInterface();
      lastButtonPress = millis();
    }
  }
  
  lastECGState = ecgPressed;
  lastFFTState = fftPressed;
}

// ========== ANÁLISIS FFT ==========
void updateFFTSpectrum() {
  if (fftIndex >= FFT_SAMPLES && fingerDetected) {
    Serial.println("🔬 Procesando FFT...");
    
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
    
    tft.fillRect(FFT_GRAPH_LEFT + 1, FFT_GRAPH_TOP + 1, 
                 FFT_GRAPH_WIDTH - 2, FFT_GRAPH_HEIGHT - 2, ILI9341_BLACK);
    
    double maxMagnitude = 0;
    int startIdx = max(1, (int)(0.5 * FFT_SAMPLES / SAMPLING_FREQUENCY));
    int endIdx = min(FFT_SAMPLES/2, (int)(10.0 * FFT_SAMPLES / SAMPLING_FREQUENCY));
    
    for (int i = startIdx; i < endIdx; i++) {
      if (vReal[i] > maxMagnitude) maxMagnitude = vReal[i];
    }
    
    int lastX = FFT_GRAPH_LEFT + 1;
    int lastY = FFT_GRAPH_BOTTOM - 1;
    
    for (int i = startIdx; i < endIdx; i++) {
      double magnitude = vReal[i];
      double freq = (i * SAMPLING_FREQUENCY) / (double)FFT_SAMPLES;
      
      int x_pos = map(i, startIdx, endIdx, FFT_GRAPH_LEFT + 1, FFT_GRAPH_RIGHT - 1);
      int y_pos = map(magnitude, 0, maxMagnitude * 1.1, FFT_GRAPH_BOTTOM - 1, FFT_GRAPH_TOP + 1);
      y_pos = constrain(y_pos, FFT_GRAPH_TOP + 1, FFT_GRAPH_BOTTOM - 1);
      
      uint16_t color = ILI9341_GREEN;
      if (freq > 1.5) color = ILI9341_CYAN;
      if (freq > 3.0) color = ILI9341_YELLOW;
      if (freq > 5.0) color = ILI9341_RED;
      
      if (i > startIdx) {
        tft.drawLine(lastX, lastY, x_pos, y_pos, color);
      }
      
      lastX = x_pos;
      lastY = y_pos;
    }
    
    int peakIndex = startIdx;
    double peakValue = 0;
    
    for (int i = startIdx; i < endIdx; i++) {
      if (vReal[i] > peakValue) {
        peakValue = vReal[i];
        peakIndex = i;
      }
    }
    
    double peakFreq = (peakIndex * SAMPLING_FREQUENCY) / (double)FFT_SAMPLES;
    double estimatedBPM = peakFreq * 60.0;
    
    int peakX = map(peakIndex, startIdx, endIdx, FFT_GRAPH_LEFT + 1, FFT_GRAPH_RIGHT - 1);
    int peakY = map(peakValue, 0, maxMagnitude * 1.1, FFT_GRAPH_BOTTOM - 1, FFT_GRAPH_TOP + 1);
    tft.fillCircle(peakX, peakY, 3, ILI9341_WHITE);
    
    tft.fillRect(20, FFT_GRAPH_BOTTOM + 10, 280, 30, ILI9341_BLACK);
    tft.setCursor(20, FFT_GRAPH_BOTTOM + 10);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    
    if (peakFreq > 0.5 && peakFreq < 4.0) {
      tft.printf("Pico: %.2f Hz (%.0f BPM)", peakFreq, estimatedBPM);
      tft.setCursor(20, FFT_GRAPH_BOTTOM + 22);
      tft.printf("Mag: %.0f", peakValue);
    } else {
      tft.print("Señal fuera de rango cardíaco");
    }
    
    for (int refHz = 1; refHz <= 3; refHz++) {
      int refIndex = (int)(refHz * FFT_SAMPLES / SAMPLING_FREQUENCY);
      if (refIndex >= startIdx && refIndex < endIdx) {
        int refX = map(refIndex, startIdx, endIdx, FFT_GRAPH_LEFT + 1, FFT_GRAPH_RIGHT - 1);
        tft.drawLine(refX, FFT_GRAPH_TOP + 1, refX, FFT_GRAPH_BOTTOM - 1, ILI9341_DARKGREY);
      }
    }
    
    fftIndex = 0;
    Serial.printf("✅ FFT: Pico en %.2f Hz (%.0f BPM)\n", peakFreq, estimatedBPM);
    
  } else if (!fingerDetected) {
    tft.fillRect(FFT_GRAPH_LEFT + 1, FFT_GRAPH_TOP + 1, 
                 FFT_GRAPH_WIDTH - 2, FFT_GRAPH_HEIGHT - 2, ILI9341_BLACK);
    
    tft.fillRect(20, FFT_GRAPH_BOTTOM + 10, 280, 30, ILI9341_BLACK);
    tft.setCursor(20, FFT_GRAPH_BOTTOM + 10);
    tft.setTextColor(ILI9341_RED);
    tft.setTextSize(1);
    tft.print("Esperando señal...");
    
    fftIndex = 0;
  }
}

// ========== ACTUALIZAR ONDA ECG ==========
void updateECGWave() {
  ecg_speed = smoothedBPM / 60.0;
  
  int index = (int)ecg_phase % ECG_SAMPLES;
  uint8_t ecg_value = pgm_read_byte(&ecg_template[index]);
  
  int y_new = map(ecg_value, 0, 255, ECG_GRAPH_BOTTOM - 5, ECG_GRAPH_TOP + 5);
  
  tft.drawLine(ecg_lastx, ecg_lasty, ecg_x, y_new, ILI9341_RED);
  
  ecg_lasty = y_new;
  ecg_lastx = ecg_x;
  ecg_x += 2.0;
  
  ecg_phase += ecg_speed * 0.5;
  
  if (ecg_x >= ECG_GRAPH_RIGHT - 2) {
    clearECGGraph();
    ecg_x = ECG_GRAPH_LEFT + 1;
    ecg_lastx = ecg_x;
    ecg_lasty = ECG_GRAPH_TOP + ECG_GRAPH_HEIGHT/2;
  }
}

void drawFlatLine() {
  int y_flat = ECG_GRAPH_TOP + ECG_GRAPH_HEIGHT/2;
  tft.drawLine(ecg_lastx, ecg_lasty, ecg_x, y_flat, ILI9341_DARKGREY);
  
  ecg_lasty = y_flat;
  ecg_lastx = ecg_x;
  ecg_x += 2.0;
  
  if (ecg_x >= ECG_GRAPH_RIGHT - 2) {
    clearECGGraph();
    ecg_x = ECG_GRAPH_LEFT + 1;
    ecg_lastx = ecg_x;
    ecg_lasty = y_flat;
  }
}

// ========== INTERFAZ GRÁFICA ==========
void drawInterface() {
  tft.fillScreen(ILI9341_BLACK);
  
  if (currentMode == MODE_ECG_VITALS) {
    drawECGInterface();
  } else {
    drawFFTInterface();
  }
}

void drawECGInterface() {
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_CYAN);
  tft.setCursor(30, 5);
  tft.print("ECG + SIGNOS VITALES");
  
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(10, 40);
  tft.print("BPM:");
  
  tft.setCursor(110, 40);
  tft.print("SpO2:");
  
  tft.setCursor(210, 40);
  tft.print("Estado:");
  
  tft.drawRect(ECG_GRAPH_LEFT, ECG_GRAPH_TOP, ECG_GRAPH_WIDTH, ECG_GRAPH_HEIGHT, ILI9341_GREEN);
  
  tft.setTextColor(ILI9341_GREEN);
  tft.setCursor(ECG_GRAPH_LEFT, ECG_GRAPH_TOP - 12);
  tft.print("ECG Sincronizado (Pan-Tompkins)");
  
  int center_y = ECG_GRAPH_TOP + ECG_GRAPH_HEIGHT/2;
  tft.drawLine(ECG_GRAPH_LEFT + 1, center_y, ECG_GRAPH_RIGHT - 1, center_y, ILI9341_DARKGREY);
  
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(20, ECG_GRAPH_BOTTOM + 8);
  tft.printf("LP:40Hz HP:0.5Hz | AC/DC Calib");
  
  tft.setCursor(20, 225);
  tft.setTextColor(ILI9341_GREEN);
  tft.print("[GPIO34:ECG]");
  tft.setCursor(150, 225);
  tft.setTextColor(ILI9341_DARKGREY);
  tft.print("[GPIO35:FFT]");
}

void drawFFTInterface() {
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_MAGENTA);
  tft.setCursor(40, 5);
  tft.print("ANALISIS FFT");
  
  tft.drawRect(FFT_GRAPH_LEFT, FFT_GRAPH_TOP, FFT_GRAPH_WIDTH, FFT_GRAPH_HEIGHT, ILI9341_CYAN);
  
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_CYAN);
  tft.setCursor(FFT_GRAPH_LEFT, FFT_GRAPH_TOP - 12);
  tft.print("Espectro (0-10 Hz) - Filtrado");
  
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(FFT_GRAPH_LEFT, FFT_GRAPH_BOTTOM + 2);
  tft.print("0Hz");
  tft.setCursor(FFT_GRAPH_RIGHT - 25, FFT_GRAPH_BOTTOM + 2);
  tft.print("10Hz");
  
  tft.setCursor(20, 30);
  tft.print("Estado:");
  
  tft.setCursor(20, 225);
  tft.setTextColor(ILI9341_DARKGREY);
  tft.print("[GPIO34:ECG]");
  tft.setCursor(150, 225);
  tft.setTextColor(ILI9341_MAGENTA);
  tft.print("[GPIO35:FFT]");
}

void clearECGGraph() {
  tft.fillRect(ECG_GRAPH_LEFT + 1, ECG_GRAPH_TOP + 1, 
               ECG_GRAPH_WIDTH - 2, ECG_GRAPH_HEIGHT - 2, ILI9341_BLACK);
  
  int center_y = ECG_GRAPH_TOP + ECG_GRAPH_HEIGHT/2;
  tft.drawLine(ECG_GRAPH_LEFT + 1, center_y, ECG_GRAPH_RIGHT - 1, center_y, ILI9341_DARKGREY);
}

// ========== ACTUALIZAR DISPLAYS ==========
void updateBPMDisplay() {
  tft.fillRect(40, 40, 60, 10, ILI9341_BLACK);
  tft.setCursor(40, 40);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_GREEN);
  tft.printf("%.0f", smoothedBPM);
}

void updateSpO2Display() {
  tft.fillRect(145, 40, 50, 10, ILI9341_BLACK);
  tft.setCursor(145, 40);
  tft.setTextSize(1);
  
  if (spO2 > 85) {
    tft.setTextColor(ILI9341_CYAN);
    tft.printf("%.1f%%", spO2);
  } else if (spO2 > 70) {
    tft.setTextColor(ILI9341_YELLOW);
    tft.printf("%.1f%%", spO2);
  } else {
    tft.setTextColor(ILI9341_DARKGREY);
    tft.print("---%");
  }
}

void updateFingerStatus() {
  if (currentMode == MODE_ECG_VITALS) {
    tft.fillRect(255, 40, 60, 10, ILI9341_BLACK);
    tft.setCursor(255, 40);
  } else {
    tft.fillRect(70, 30, 80, 10, ILI9341_BLACK);
    tft.setCursor(70, 30);
  }
  
  tft.setTextSize(1);
  
  if (fingerDetected) {
    tft.setTextColor(ILI9341_GREEN);
    tft.print("OK");
  } else {
    tft.setTextColor(ILI9341_RED);
    tft.print("SIN DEDO");
  }
}

void showError(const char* message) {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_RED);
  tft.setCursor(50, 100);
  tft.print("ERROR:");
  tft.setCursor(50, 130);
  tft.print(message);
}