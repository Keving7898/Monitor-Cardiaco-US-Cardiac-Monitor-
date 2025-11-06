import wfdb
import numpy as np
import pandas as pd

# === 1. Leer el registro ===
record_name = '3141595_0001'
pn_dir = 'mimic3wdb/1.0/31/3141595/'
record = wfdb.rdrecord(record_name, pn_dir=pn_dir)

print("Canales disponibles:", record.sig_name)
print("Frecuencia de muestreo:", record.fs, "Hz")

# === 2. Buscar canal ECG (puede ser II, I, III, V, etc.) ===
ecg_channels = [ch for ch in record.sig_name if 'ECG' in ch or ch in ['II', 'I', 'III', 'V']]

if not ecg_channels:
    print("⚠️ No se encontró canal ECG en este registro")
    print("Canales disponibles:", record.sig_name)
    exit()

# Usar el primer canal ECG disponible (típicamente 'II' es el más limpio)
ecg_channel = ecg_channels[0]
print(f"✅ Usando canal: {ecg_channel}")

idx = record.sig_name.index(ecg_channel)
ecg_signal = record.p_signal[:, idx]

# === 3. Parámetros ===
fs = int(record.fs)        # Frecuencia de muestreo (ej: 125 Hz)
duration_seconds = 80      # Duración deseada
chunk_size = fs * duration_seconds  # muestras
num_chunks = 4

# === 4. Verificar que hay suficientes datos ===
total_samples = len(ecg_signal)
print(f"Total de muestras disponibles: {total_samples}")
print(f"Duración total: {total_samples/fs:.2f} segundos")

if total_samples < chunk_size * num_chunks:
    print(f"⚠️ Advertencia: Solo hay datos para {total_samples//(chunk_size)} chunks completos")
    num_chunks = min(num_chunks, total_samples // chunk_size)

# === 5. Cortar las primeras porciones ===
chunks = [ecg_signal[i*chunk_size:(i+1)*chunk_size] for i in range(num_chunks)]

# === 6. Guardar cada porción en CSV ===
for i, chunk in enumerate(chunks, 1):
    df = pd.DataFrame(chunk, columns=[f'ECG_{ecg_channel}'])
    df.to_csv(f'ecg_chunk_{i}.csv', index=False)
    print(f"✅ Guardado: ecg_chunk_{i}.csv ({len(chunk)} muestras, {len(chunk)/fs:.2f}s)")

print(f"\n🎉 Se guardaron {num_chunks} porciones de ECG")