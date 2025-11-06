import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.fft import fft, fftfreq

# === 1. LEER EL ARCHIVO CSV ===
filename = 'ecg_chunk_1.csv'
print(f"Leyendo archivo: {filename}")

df = pd.read_csv(filename)
print(f"✅ Archivo cargado: {len(df)} muestras")
print(f"   Columnas: {df.columns.tolist()}\n")

# === 2. EXTRAER LA SEÑAL ECG ===
ecg_column = [col for col in df.columns if col != 'sample'][0]
ecg_signal = df[ecg_column].values

# === 3. PARÁMETROS DE LA SEÑAL ===
fs = 125  # Frecuencia de muestreo en Hz
N = len(ecg_signal)  # Número de muestras
duration = N / fs  # Duración en segundos
time = np.arange(N) / fs

print(f"📊 PARÁMETROS DE LA SEÑAL")
print(f"   Frecuencia de muestreo (fs): {fs} Hz")
print(f"   Número de muestras (N): {N}")
print(f"   Duración total: {duration:.2f} segundos")
print(f"   Resolución temporal: {1/fs*1000:.2f} ms/muestra")
print(f"   Frecuencia de Nyquist: {fs/2} Hz\n")

# === 4. VENTANA DE HAMMING ===
ventana_hamming = np.hamming(N)

# Aplicar ventana a la señal
ecg_windowed = ecg_signal * ventana_hamming

print(f"🪟 PARÁMETROS DE LA VENTANA DE HAMMING")
print(f"   Longitud de la ventana: {N} muestras")
print(f"   Factor de forma: 0.54 - 0.46*cos(2πn/(N-1))")
print(f"   Atenuación del lóbulo lateral: ~43 dB")
print(f"   Ancho del lóbulo principal: 8π/N radianes\n")

# === 5. FFT (Fast Fourier Transform) ===
# FFT de la señal original
fft_signal = fft(ecg_signal)
fft_magnitude = np.abs(fft_signal)
fft_phase = np.angle(fft_signal)

# FFT de la señal con ventana
fft_windowed = fft(ecg_windowed)
fft_windowed_magnitude = np.abs(fft_windowed)

# Frecuencias correspondientes
freqs = fftfreq(N, 1/fs)

# Solo tomar la mitad positiva del espectro
positive_freqs = freqs[:N//2]
fft_magnitude_positive = fft_magnitude[:N//2]
fft_windowed_magnitude_positive = fft_windowed_magnitude[:N//2]
fft_phase_positive = fft_phase[:N//2]

# Convertir a dB
fft_magnitude_db = 20 * np.log10(fft_magnitude_positive + 1e-10)
fft_windowed_magnitude_db = 20 * np.log10(fft_windowed_magnitude_positive + 1e-10)

# Resolución en frecuencia
freq_resolution = fs / N
print(f"🔍 PARÁMETROS DE LA FFT")
print(f"   Puntos de FFT: {N}")
print(f"   Resolución en frecuencia: {freq_resolution:.4f} Hz")
print(f"   Rango de frecuencias: 0 - {fs/2} Hz")
print(f"   Bins de frecuencia: {N//2}\n")

# === 6. ENCONTRAR FRECUENCIAS DOMINANTES ===
# Buscar los picos más importantes en el espectro
peak_indices = signal.find_peaks(fft_windowed_magnitude_positive, 
                                  height=np.max(fft_windowed_magnitude_positive)*0.1)[0]
peak_freqs = positive_freqs[peak_indices]
peak_magnitudes = fft_windowed_magnitude_positive[peak_indices]

# Ordenar por magnitud
sorted_indices = np.argsort(peak_magnitudes)[::-1][:5]  # Top 5
top_freqs = peak_freqs[sorted_indices]
top_magnitudes = peak_magnitudes[sorted_indices]

print(f"🎯 FRECUENCIAS DOMINANTES (Top 5)")
for i, (freq, mag) in enumerate(zip(top_freqs, top_magnitudes), 1):
    print(f"   {i}. {freq:.3f} Hz (Magnitud: {mag:.2f})")

# Calcular frecuencia cardíaca aproximada (entre 0.5-3 Hz = 30-180 bpm)
hr_range = (positive_freqs >= 0.5) & (positive_freqs <= 3.0)
hr_freqs = positive_freqs[hr_range]
hr_magnitudes = fft_windowed_magnitude_positive[hr_range]
hr_peak_idx = np.argmax(hr_magnitudes)
heart_rate_hz = hr_freqs[hr_peak_idx]
heart_rate_bpm = heart_rate_hz * 60

print(f"\n❤️ FRECUENCIA CARDÍACA ESTIMADA")
print(f"   Frecuencia: {heart_rate_hz:.3f} Hz")
print(f"   BPM: {heart_rate_bpm:.1f} latidos/minuto\n")

# === 7. PARÁMETROS DE ESCALA DEL MARCO DE FONDO ===
# Puedes ajustar estos valores para cambiar el tamaño de las figuras
FIGURA_ANCHO = 16  # pulgadas (cambiar según necesites)
FIGURA_ALTO = 10   # pulgadas (cambiar según necesites)
DPI = 100          # resolución en puntos por pulgada

print(f"📐 PARÁMETROS DE ESCALA DE GRÁFICAS")
print(f"   Ancho de figura: {FIGURA_ANCHO} pulgadas")
print(f"   Alto de figura: {FIGURA_ALTO} pulgadas")
print(f"   DPI: {DPI}")
print(f"   Resolución en píxeles: {FIGURA_ANCHO*DPI} x {FIGURA_ALTO*DPI}\n")

# === 8. GRAFICAR RESULTADOS ===
fig = plt.figure(figsize=(FIGURA_ANCHO, FIGURA_ALTO), dpi=DPI)
gs = fig.add_gridspec(3, 2, hspace=0.35, wspace=0.3)

# Subplot 1: Señal Original
ax1 = fig.add_subplot(gs[0, :])
ax1.plot(time, ecg_signal, linewidth=1, color='blue')
ax1.set_xlabel('Tiempo (s)', fontsize=11)
ax1.set_ylabel('Amplitud (mV)', fontsize=11)
ax1.set_title('Señal ECG Original', fontsize=13, fontweight='bold')
ax1.grid(True, alpha=0.3)
ax1.set_xlim([0, min(10, duration)])

# Subplot 2: Ventana de Hamming
ax2 = fig.add_subplot(gs[1, 0])
ax2.plot(ventana_hamming, linewidth=1.5, color='green')
ax2.set_xlabel('Muestras', fontsize=11)
ax2.set_ylabel('Amplitud', fontsize=11)
ax2.set_title('Ventana de Hamming', fontsize=13, fontweight='bold')
ax2.grid(True, alpha=0.3)

# Subplot 3: Señal con Ventana Aplicada
ax3 = fig.add_subplot(gs[1, 1])
ax3.plot(time, ecg_windowed, linewidth=1, color='purple')
ax3.set_xlabel('Tiempo (s)', fontsize=11)
ax3.set_ylabel('Amplitud (mV)', fontsize=11)
ax3.set_title('Señal ECG con Ventana Hamming', fontsize=13, fontweight='bold')
ax3.grid(True, alpha=0.3)
ax3.set_xlim([0, min(10, duration)])

# Subplot 4: Espectro de Magnitud (Comparación)
ax4 = fig.add_subplot(gs[2, :])
ax4.plot(positive_freqs, fft_magnitude_db, linewidth=1, color='red', 
         alpha=0.5, label='Sin ventana')
ax4.plot(positive_freqs, fft_windowed_magnitude_db, linewidth=1.5, color='blue', 
         label='Con ventana Hamming')
ax4.axvline(x=heart_rate_hz, color='green', linestyle='--', linewidth=2,
            label=f'FC: {heart_rate_bpm:.1f} BPM')
ax4.set_xlabel('Frecuencia (Hz)', fontsize=11)
ax4.set_ylabel('Magnitud (dB)', fontsize=11)
ax4.set_title('Espectro de Frecuencias (FFT)', fontsize=13, fontweight='bold')
ax4.set_xlim([0, 20])  # Mostrar hasta 20 Hz (rango típico ECG)
ax4.grid(True, alpha=0.3)
ax4.legend(loc='upper right', fontsize=10)

plt.suptitle('Análisis FFT de Señal ECG con Ventana Hamming', 
             fontsize=16, fontweight='bold', y=0.98)
plt.show()

# === 9. GRÁFICA ADICIONAL: ESPECTRO DE MAGNITUD Y FASE ===
# Aquí también puedes ajustar el tamaño
FIGURA2_ANCHO = 15
FIGURA2_ALTO = 8

fig2, axes = plt.subplots(2, 1, figsize=(FIGURA2_ANCHO, FIGURA2_ALTO), dpi=DPI)

# Espectro de Magnitud detallado
axes[0].plot(positive_freqs, fft_windowed_magnitude_positive, 
             linewidth=1.5, color='blue')
axes[0].axvline(x=heart_rate_hz, color='red', linestyle='--', linewidth=2,
                label=f'FC: {heart_rate_bpm:.1f} BPM')
for freq in top_freqs[:3]:
    axes[0].axvline(x=freq, color='orange', linestyle=':', linewidth=1, alpha=0.7)
axes[0].set_xlabel('Frecuencia (Hz)', fontsize=11)
axes[0].set_ylabel('Magnitud', fontsize=11)
axes[0].set_title('Espectro de Magnitud (Escala Lineal)', fontsize=13, fontweight='bold')
axes[0].set_xlim([0, 20])
axes[0].grid(True, alpha=0.3)
axes[0].legend(fontsize=10)

# Espectro de Fase
axes[1].plot(positive_freqs, fft_phase_positive, linewidth=1, color='green')
axes[1].set_xlabel('Frecuencia (Hz)', fontsize=11)
axes[1].set_ylabel('Fase (radianes)', fontsize=11)
axes[1].set_title('Espectro de Fase', fontsize=13, fontweight='bold')
axes[1].set_xlim([0, 20])
axes[1].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

print("✅ Análisis espectral completado")
print(f"\n💡 CONSEJO: Para ajustar el tamaño de las gráficas, modifica:")
print(f"   - FIGURA_ANCHO y FIGURA_ALTO (en pulgadas)")
print(f"   - DPI (resolución en puntos por pulgada)")
print(f"   - Ejemplo: figsize=(20, 12) y dpi=150 para gráficas más grandes")