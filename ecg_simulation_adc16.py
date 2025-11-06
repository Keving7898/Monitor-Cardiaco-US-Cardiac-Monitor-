import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# === 1. LEER EL ARCHIVO CSV ===
filename = 'ecg_chunk_1.csv'
print(f"Leyendo archivo: {filename}")

df = pd.read_csv(filename)
print(f"✅ Archivo cargado: {len(df)} muestras")
print(f"   Columnas: {df.columns.tolist()}\n")

# === 2. EXTRAER LA SEÑAL ECG (ANALÓGICA) ===
ecg_column = [col for col in df.columns if col != 'sample'][0]
señal_analogica = df[ecg_column].values

# === 3. PARÁMETROS DEL ADC ===
bits = 16
niveles = 2**bits  # 65536 niveles
print(f"📊 PARÁMETROS DEL ADC")
print(f"   Resolución: {bits} bits")
print(f"   Niveles de cuantización: {niveles}")

# Rango de la señal analógica
V_min = señal_analogica.min()
V_max = señal_analogica.max()
V_rango = V_max - V_min

# Paso de cuantización (LSB - Least Significant Bit)
LSB = V_rango / (niveles - 1)
print(f"   Rango de entrada: [{V_min:.4f}, {V_max:.4f}] mV")
print(f"   Paso de cuantización (LSB): {LSB:.6f} mV")
print(f"   Voltaje por bit: {LSB*1000:.3f} µV\n")

# === 4. PROCESO DE CUANTIZACIÓN ===
# Normalizar la señal al rango [0, niveles-1]
señal_normalizada = (señal_analogica - V_min) / V_rango * (niveles - 1)

# Cuantizar (redondear al nivel más cercano)
señal_cuantizada_digital = np.round(señal_normalizada).astype(int)

# Convertir de nuevo a valores analógicos
señal_cuantizada = señal_cuantizada_digital * LSB + V_min

# === 5. ERROR DE CUANTIZACIÓN ===
error_cuantizacion = señal_analogica - señal_cuantizada

# === 6. CÁLCULO DEL SNR (Signal-to-Noise Ratio) ===
# Potencia de la señal
potencia_señal = np.mean(señal_analogica**2)

# Potencia del ruido (error de cuantización)
potencia_ruido = np.mean(error_cuantizacion**2)

# SNR en escala lineal y en dB
SNR_lineal = potencia_señal / potencia_ruido
SNR_dB = 10 * np.log10(SNR_lineal)

# SNR teórico para un ADC de N bits
SNR_teorico_dB = 6.02 * bits + 1.76

print(f"🔊 RELACIÓN SEÑAL A RUIDO (SNR)")
print(f"   SNR medido: {SNR_dB:.2f} dB")
print(f"   SNR teórico (ADC {bits}-bit): {SNR_teorico_dB:.2f} dB")
print(f"   Diferencia: {SNR_dB - SNR_teorico_dB:.2f} dB\n")

print(f"📉 ESTADÍSTICAS DEL ERROR")
print(f"   Error RMS: {np.sqrt(potencia_ruido):.6f} mV")
print(f"   Error máximo: {np.abs(error_cuantizacion).max():.6f} mV")
print(f"   Error teórico máximo: ±{LSB/2:.6f} mV (LSB/2)\n")

# === 7. CREAR EJE DE TIEMPO ===
fs = 125  # Frecuencia de muestreo en Hz
time = np.arange(len(señal_analogica)) / fs

# === 8. GRAFICAR RESULTADOS ===
duration_to_plot = 5  # segundos
samples_to_plot = int(duration_to_plot * fs)

fig, axes = plt.subplots(4, 1, figsize=(15, 12))
fig.suptitle(f'Simulación ADC de {bits} bits - Señal ECG', 
             fontsize=16, fontweight='bold')

# Subplot 1: Señal Analógica Original
axes[0].plot(time[:samples_to_plot], señal_analogica[:samples_to_plot], 
             linewidth=1.5, color='blue', label='Señal Analógica Original')
axes[0].set_ylabel('Amplitud (mV)', fontsize=11)
axes[0].set_title('Señal Analógica Original', fontsize=12, fontweight='bold')
axes[0].grid(True, alpha=0.3)
axes[0].legend(loc='upper right')

# Subplot 2: Comparación Analógica vs Cuantizada
axes[1].plot(time[:samples_to_plot], señal_analogica[:samples_to_plot], 
             linewidth=1.5, color='blue', alpha=0.7, label='Señal Analógica')
axes[1].plot(time[:samples_to_plot], señal_cuantizada[:samples_to_plot], 
             linewidth=1.2, color='red', alpha=0.8, linestyle='--', 
             label=f'Señal Cuantizada ({bits} bits)')
axes[1].set_ylabel('Amplitud (mV)', fontsize=11)
axes[1].set_title('Comparación: Analógica vs Cuantizada', fontsize=12, fontweight='bold')
axes[1].grid(True, alpha=0.3)
axes[1].legend(loc='upper right')

# Subplot 3: Error de Cuantización
axes[2].plot(time[:samples_to_plot], error_cuantizacion[:samples_to_plot]*1000, 
             linewidth=1, color='purple', alpha=0.8)
axes[2].axhline(y=0, color='black', linestyle='-', linewidth=0.8)
axes[2].axhline(y=(LSB/2)*1000, color='red', linestyle='--', linewidth=0.8, 
                alpha=0.6, label=f'±LSB/2 = ±{(LSB/2)*1000:.3f} µV')
axes[2].axhline(y=-(LSB/2)*1000, color='red', linestyle='--', linewidth=0.8, alpha=0.6)
axes[2].set_ylabel('Error (µV)', fontsize=11)
axes[2].set_title(f'Error de Cuantización (RMS: {np.sqrt(potencia_ruido)*1000:.3f} µV)', 
                  fontsize=12, fontweight='bold')
axes[2].grid(True, alpha=0.3)
axes[2].legend(loc='upper right')

# Subplot 4: Histograma del Error
axes[3].hist(error_cuantizacion*1000, bins=50, color='green', alpha=0.7, edgecolor='black')
axes[3].axvline(x=0, color='red', linestyle='--', linewidth=2, label='Error = 0')
axes[3].set_xlabel('Error de Cuantización (µV)', fontsize=11)
axes[3].set_ylabel('Frecuencia', fontsize=11)
axes[3].set_title('Distribución del Error de Cuantización', fontsize=12, fontweight='bold')
axes[3].grid(True, alpha=0.3, axis='y')
axes[3].legend(loc='upper right')

plt.tight_layout()
plt.show()

# === 9. ZOOM EN UN LATIDO PARA VER MEJOR LA CUANTIZACIÓN ===
# Buscar un pico R (valor máximo local) para hacer zoom
peak_idx = np.argmax(señal_analogica[:samples_to_plot])
zoom_start = max(0, peak_idx - 50)
zoom_end = min(len(señal_analogica), peak_idx + 100)

fig2, axes2 = plt.subplots(2, 1, figsize=(15, 8))
fig2.suptitle(f'Zoom en un Latido - Detalle de Cuantización ADC {bits} bits', 
              fontsize=16, fontweight='bold')

# Comparación con zoom
axes2[0].plot(time[zoom_start:zoom_end], señal_analogica[zoom_start:zoom_end], 
              linewidth=2, color='blue', marker='o', markersize=3, 
              label='Señal Analógica', alpha=0.7)
axes2[0].plot(time[zoom_start:zoom_end], señal_cuantizada[zoom_start:zoom_end], 
              linewidth=2, color='red', marker='s', markersize=3, 
              label=f'Señal Cuantizada ({bits} bits)', alpha=0.7)
axes2[0].set_ylabel('Amplitud (mV)', fontsize=11)
axes2[0].set_title('Detalle de la Cuantización', fontsize=12, fontweight='bold')
axes2[0].grid(True, alpha=0.3)
axes2[0].legend(loc='upper right')

# Error en el zoom
axes2[1].plot(time[zoom_start:zoom_end], error_cuantizacion[zoom_start:zoom_end]*1000, 
              linewidth=1.5, color='purple', marker='o', markersize=3)
axes2[1].axhline(y=0, color='black', linestyle='-', linewidth=0.8)
axes2[1].set_xlabel('Tiempo (segundos)', fontsize=11)
axes2[1].set_ylabel('Error (µV)', fontsize=11)
axes2[1].set_title('Error de Cuantización en Detalle', fontsize=12, fontweight='bold')
axes2[1].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

print("✅ Simulación completada")