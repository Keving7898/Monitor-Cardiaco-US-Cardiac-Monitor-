import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from matplotlib.patches import Circle

# ============ PARÁMETROS DEL FILTRO ============
fs = 125  # Frecuencia de muestreo (Hz)
fc = 40   # Frecuencia de corte (Hz)
order = 2  # Orden del filtro

# ============ DISEÑO DEL FILTRO BUTTERWORTH ============
# Normalizar frecuencia de corte (Nyquist)
wc = fc / (fs / 2)

# Diseñar filtro Butterworth digital
b, a = signal.butter(order, wc, btype='low', analog=False)

# Normalizar denominador (a0 = 1)
a_norm = a / a[0]
b_norm = b / a[0]

# Imprimir coeficientes
print("=" * 70)
print("COEFICIENTES DEL FILTRO BUTTERWORTH ORDEN 2")
print("=" * 70)
print(f"Frecuencia de muestreo: {fs} Hz")
print(f"Frecuencia de corte: {fc} Hz")
print(f"\nCoeficientes normalizados:")
print(f"  b0 = {b_norm[0]:.6f}")
print(f"  b1 = {b_norm[1]:.6f}")
print(f"  b2 = {b_norm[2]:.6f}")
print(f"  a1 = {a_norm[1]:.6f}")
print(f"  a2 = {a_norm[2]:.6f}")
print("\nEcuación en diferencias:")
print(f"y[n] = {b_norm[0]:.4f}*x[n] + {b_norm[1]:.4f}*x[n-1] + {b_norm[2]:.4f}*x[n-2]")
print(f"       - ({a_norm[1]:.4f})*y[n-1] - ({a_norm[2]:.4f})*y[n-2]")
print("=" * 70)

# ============ CALCULAR POLOS Y CEROS ============
zeros = np.roots(b)
poles = np.roots(a)

# ============ RESPUESTA EN FRECUENCIA ============
w, h = signal.freqz(b, a, worN=8000, fs=fs)
magnitude_db = 20 * np.log10(abs(h))

# Encontrar frecuencia de -3dB
idx_3db = np.argmin(np.abs(magnitude_db + 3))
f_3db = w[idx_3db]

# ============ RESPUESTAS AL IMPULSO Y ESCALÓN ============
impulse_length = 100
impulse = np.zeros(impulse_length)
impulse[0] = 1
t_impulse = np.arange(impulse_length) / fs
h_impulse = signal.lfilter(b, a, impulse)

step = np.ones(impulse_length)
h_step = signal.lfilter(b, a, step)

# ============ CREAR FIGURA CON 4 SUBPLOTS ============
fig = plt.figure(figsize=(14, 10))
fig.suptitle('ANÁLISIS DEL FILTRO BUTTERWORTH ORDEN 2 (Anti-Aliasing fc=40Hz, fs=125Hz)', 
             fontsize=14, fontweight='bold', y=0.98)

# -------- SUBPLOT 1: DIAGRAMA DE POLOS Y CEROS (PLANO Z) --------
ax1 = plt.subplot(2, 2, 1)
ax1.set_title('Diagrama de Polos y Ceros (Plano Z)', fontweight='bold', fontsize=12)

# Círculo unitario
circle = Circle((0, 0), 1, fill=False, color='black', linestyle='--', linewidth=1.5)
ax1.add_patch(circle)

# Ejes
ax1.axhline(y=0, color='k', linewidth=0.5)
ax1.axvline(x=0, color='k', linewidth=0.5)

# Graficar ceros (círculos)
ax1.plot(np.real(zeros), np.imag(zeros), 'o', markersize=12, 
         markerfacecolor='white', markeredgecolor='blue', markeredgewidth=2.5, label='Ceros')

# Graficar polos (cruces)
ax1.plot(np.real(poles), np.imag(poles), 'x', markersize=12, 
         color='red', markeredgewidth=2.5, label='Polos')

ax1.set_xlabel('Parte Real', fontsize=11)
ax1.set_ylabel('Parte Imaginaria', fontsize=11)
ax1.grid(True, alpha=0.3)
ax1.axis('equal')
ax1.set_xlim([-1.3, 1.3])
ax1.set_ylim([-1.3, 1.3])
ax1.legend(loc='upper right', fontsize=10)

# Anotar coordenadas de los polos
for i, p in enumerate(poles):
    ax1.annotate(f'  p{i+1}\n  ({np.real(p):.3f}, {np.imag(p):.3f})', 
                xy=(np.real(p), np.imag(p)), 
                xytext=(10, -10), textcoords='offset points', 
                fontsize=8, color='red',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.5))

# -------- SUBPLOT 2: RESPUESTA AL IMPULSO --------
ax2 = plt.subplot(2, 2, 2)
ax2.set_title('Respuesta al Impulso h[n]', fontweight='bold', fontsize=12)
markerline, stemlines, baseline = ax2.stem(t_impulse, h_impulse, linefmt='b-', 
                                           markerfmt='bo', basefmt='k-')
markerline.set_markerfacecolor('blue')
markerline.set_markersize(4)
stemlines.set_linewidth(1.5)
ax2.set_xlabel('Tiempo (s)', fontsize=11)
ax2.set_ylabel('Amplitud', fontsize=11)
ax2.grid(True, alpha=0.3)
ax2.axhline(y=0, color='k', linewidth=0.5)

# Añadir valores clave
max_impulse = np.max(h_impulse)
max_idx = np.argmax(h_impulse)
ax2.text(0.98, 0.95, f'Máximo: {max_impulse:.4f}\nen n={max_idx}', 
         transform=ax2.transAxes, fontsize=9,
         verticalalignment='top', horizontalalignment='right',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

# -------- SUBPLOT 3: RESPUESTA AL ESCALÓN UNITARIO --------
ax3 = plt.subplot(2, 2, 3)
ax3.set_title('Respuesta al Escalón Unitario', fontweight='bold', fontsize=12)
ax3.plot(t_impulse, h_step, 'g-', linewidth=2.5, label='Respuesta')
ax3.axhline(y=1, color='r', linestyle='--', linewidth=1.5, alpha=0.7, label='Valor final ideal')
ax3.set_xlabel('Tiempo (s)', fontsize=11)
ax3.set_ylabel('Amplitud', fontsize=11)
ax3.grid(True, alpha=0.3)
ax3.legend(loc='lower right', fontsize=10)

# Añadir información de settling time
settling_threshold = 0.98
settling_idx = np.where(h_step >= settling_threshold)[0]
if len(settling_idx) > 0:
    settling_time = t_impulse[settling_idx[0]]
    ax3.text(0.98, 0.05, f'Tiempo de establecimiento\n(98%): {settling_time:.3f} s', 
             transform=ax3.transAxes, fontsize=9,
             verticalalignment='bottom', horizontalalignment='right',
             bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.5))

# -------- SUBPLOT 4: RESPUESTA EN FRECUENCIA - MAGNITUD --------
ax4 = plt.subplot(2, 2, 4)
ax4.set_title('Respuesta en Frecuencia - Magnitud', fontweight='bold', fontsize=12)
ax4.plot(w, magnitude_db, 'b', linewidth=2.5, label='Magnitud')
ax4.axhline(y=-3, color='r', linestyle='--', linewidth=1.5, label='-3 dB')
ax4.axvline(x=fc, color='g', linestyle='--', linewidth=1.5, label=f'fc diseño = {fc} Hz')
ax4.axvline(x=f_3db, color='orange', linestyle=':', linewidth=2, label=f'fc real = {f_3db:.1f} Hz')
ax4.set_xlabel('Frecuencia (Hz)', fontsize=11)
ax4.set_ylabel('Magnitud (dB)', fontsize=11)
ax4.grid(True, alpha=0.3)
ax4.legend(loc='upper right', fontsize=9)
ax4.set_xlim([0, fs/2])
ax4.set_ylim([-60, 5])

# Añadir información de atenuación
ax4.text(0.02, 0.05, f'Atenuación en fc: {magnitude_db[np.argmin(np.abs(w - fc))]:.2f} dB\n' +
                     f'Pendiente: ~{order * 20} dB/década', 
         transform=ax4.transAxes, fontsize=9,
         verticalalignment='bottom',
         bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))

plt.tight_layout()
plt.savefig('butterworth_filter_analysis.png', dpi=300, bbox_inches='tight')
plt.show()

print(f"\nFrecuencia de corte real (-3dB): {f_3db:.2f} Hz")
print(f"Estabilidad: {'ESTABLE' if all(abs(p) < 1 for p in poles) else 'INESTABLE'}")
print(f"Magnitud máxima de polos: {max(abs(poles)):.6f}")
print("\n✅ Gráfica guardada como 'butterworth_filter_analysis.png'")