¡Claro! Con base en toda la información y el código proporcionados, aquí tienes una sección completa de **README** en formato Markdown, lista para tu repositorio de GitHub.

---

# 🩺 MONITOR CARDÍACO PORTÁTIL Y ANALIZADOR DE ESPECTROS (PPG/SpO2)

## 🌟 Resumen del Proyecto

[cite_start]Este proyecto consiste en el **diseño e implementación de un monitor cardíaco portátil de bajo costo** capaz de registrar, procesar y visualizar la actividad del pulso arterial y la saturación de oxígeno en sangre ($\text{SpO}_2$) en tiempo real[cite: 6]. [cite_start]Utiliza el principio de la fotopletismografía (PPG) a través del sensor óptico **MAX30102**[cite: 7].

[cite_start]El sistema integra el microcontrolador **ESP32** para el procesamiento digital de señales (PDS) y un **display LCD TFT de 2.4"** que permite la visualización gráfica avanzada en dos modos: dominio temporal y dominio de la frecuencia (análisis espectral)[cite: 7].

---

## ⚙️ Arquitectura del Sistema

### 1. **Hardware**

| Componente | Función Principal |
| :--- | :--- |
| **Microcontrolador** | ESP32 DEVKIT V1 | [cite_start]Procesamiento PDS, control de periféricos y comunicación[cite: 142]. |
| **Sensor Biomédico** | MAX30102 | [cite_start]Adquisición no invasiva de pulso (IR) y $\text{SpO}_2$ (RED/IR)[cite: 196, 199]. |
| **Display** | [cite_start]LCD TFT 2.4" SPI (ILI9341) | Interfaz gráfica para visualización en tiempo real de ondas y espectros[cite: 228]. |
| **Alimentación** | Fuente Regulable MB-102 + Batería 9V | [cite_start]Suministro regulado de $3.3 \text{ V}$ al ESP32[cite: 255, 136]. |

### 2. **Flujo de Procesamiento Digital de Señales (PDS)**

La señal infrarroja (IR) adquirida a $f_s = 125 \text{ Hz}$ sigue el siguiente pipeline para la detección de BPM :

1.  [cite_start]**Filtro Pasa-Bajo Anti-Aliasing (LPF):** IIR Butterworth de $2^{\circ}$ orden ($f_c = 40 \text{ Hz}$) para prevenir el *aliasing*[cite: 389].
2.  [cite_start]**Remoción de DC (EMA):** Filtro EMA con $\alpha=0.97$ para aislar la componente AC fisiológica[cite: 390].
3.  [cite_start]**Filtro Pasa-Alto (HPF):** IIR de $1^{\circ}$ orden ($f_c \approx 0.6 \text{ Hz}$) para eliminar la deriva de la línea de base[cite: 392].
4.  [cite_start]**Detección de QRS/BPM:** Algoritmo simplificado de **Pan-Tompkins**[cite: 395].

---

## ⚡ Algoritmo Pan-Tompkins Simplificado

El algoritmo es crucial para la detección precisa de latidos, operando con las siguientes transformaciones matemáticas:

1.  [cite_start]**Derivada:** Aproximación de la pendiente para enfatizar los picos QRS[cite: 397].
    $$\text{derivada}[n] = \frac{1}{2} \cdot \left(\text{pt\_derivative\_buffer}[n] - \text{pt\_derivative\_buffer}[n-2]\right)$$
2.  [cite_start]**Elevación al Cuadrado:** Magnifica la energía de los picos QRS[cite: 399].
    $$\text{cuadrado}[n] = (\text{derivada}[n])^2$$
3.  [cite_start]**Integración de Media Móvil:** Suaviza la energía en una ventana de $30$ muestras ($\sim 240 \text{ ms}$)[cite: 402].
    $$\text{integrada}[n] = \frac{\text{pt\_integration\_sum}}{30}$$
4.  [cite_start]**Umbral Adaptativo:** El latido se detecta si la señal integrada excede el umbral, ajustado dinámicamente con un *ratio* de $30\%$ ($\text{PT\_THRESHOLD\_RATIO}=0.3$)[cite: 407, 408].
    $$\text{pt\_threshold} = \text{Pico\_Ruido} + 0.3 \cdot (\text{Pico\_Señal} - \text{Pico\_Ruido})$$

---

## 📈 Modos de Visualización TFT

El sistema ofrece dos modos clave para el análisis de la señal:

### 1. **Modo Signos Vitales**
* Muestra la onda del pulso en el dominio temporal.
* Presenta los valores numéricos de **BPM** (suavizado con $\alpha=0.3$) y $\text{SpO}_2$.

### 2. **Modo Análisis Espectral (FFT)**
* Realiza una **Transformada Rápida de Fourier (FFT)** con **$128$ muestras** de la señal filtrada.
* [cite_start]Se aplica la **ventana de Hamming** para reducir el *leakage* espectral[cite: 429].
* El espectro de frecuencia se **codifica por color** para facilitar el diagnóstico (rango $0 \text{ Hz}$ a $10 \text{ Hz}$):
    * [cite_start]**Curva Azul:** Muy baja frecuencia ($0 \text{ Hz}$ a $1.5 \text{ Hz}$)[cite: 441].
    * [cite_start]**Curva Amarilla:** Rango cardíaco principal ($1.5 \text{ Hz}$ a $3.0 \text{ Hz}$)[cite: 441].
    * [cite_start]**Curva Roja:** Altas frecuencias/ruido (por encima de $5 \text{ Hz}$)[cite: 442].

---

## 🔗 Referencias Técnicas Clave

* J. Pan and W. J. Tompkins, "A Real-Time QRS Detection Algorithm," *IEEE Trans. Biomed. Eng.*, vol. BME-32, no. [cite_start]3, pp. 230–236, Mar. 1985[cite: 483, 484].
* A. V. Oppenheim and R. W. Schafer, *Discrete-Time Signal Processing*. [cite_start]Pearson Higher Education, 2014[cite: 473].

---
¿Necesitas que añada o modifique alguna sección en particular de este README?
