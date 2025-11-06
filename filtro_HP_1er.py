import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from matplotlib.patches import Circle

# ============ PARÁMETROS DEL FILTRO ============
fs = 125  # Frecuencia de muestreo (Hz)
HP_ALPHA = 0.97  # Coeficiente alpha del código Arduino

# ============ FILTRO PASA-ALTO CON ALPHA ============
# Ecuación: y[n] = α*y[n-1] + α*(x[n] - x[n-1])
# Función de transferencia: H(z) = α(1 - z^-1) / (1 - αz^-1)

# Coeficientes de la función de transferencia
b_alpha = HP_ALPHA * np.array([1, -1])  # Numerador: α(1 - z^-1)
a_alpha = np.array([1, -HP_ALPHA])      # Denominador: (1 - αz^-1)

# Frecuencia de corte aproximada
fc_alpha = fs * (1 - HP_ALPHA) / (2 * np.pi)

# Imprimir información del filtro
print("=" * 70)
print("FILTRO PASA-ALTO IIR CON ALPHA (Código Arduino)")
print("=" * 70)
print(f"Frecuencia de muestreo: {fs} Hz")
print(f"Coeficiente Alpha: {HP_ALPHA}")
print(f"Frecuencia de corte estimada: {fc_alpha:.3f} Hz")
print(f"\nEcuación en diferencias:")
print(f"y[n] = {HP_ALPHA}*y[n-1] + {HP_ALPHA}*(x[n] - x[n-1])")
print(f"\nFunción de transferencia H(z):")
print(f"H(z) = {HP_ALPHA}*(1 - z⁻¹) / (1 - {HP_ALPHA}*z⁻¹)")
print(f"\nCoeficientes:")
print(f"  Numerador b:  [{b_alpha[0]:.6f}, {b_alpha[1]:.6f}]")
print(f"  Denominador a: [{a_alpha[0]:.6f}, {a_alpha[1]:.6f}]")
print("=" * 70)

# ============ CALCULAR POLOS Y CEROS ============
zeros = np.roots(b_alpha)  # Cero en z = 1
poles = np.roots(a_alpha)  # Polo en z = α

print(f"\nPolos y Ceros:")
print(f"  Cero: z = {zeros[0]:.6f} (en z=1, bloquea DC)")
print(f"  Polo: z = {poles[0]:.6f} (en z=α={HP_ALPHA})")
print(f"  |Polo| = {abs(poles[0]):.6f} (< 1, sistema ESTABLE)")

# ============ RESPUESTA EN FRECUENCIA ============
w, h = signal.freqz(b_alpha, a_alpha, worN=8000, fs=fs)
magnitude = abs(h)
magnitude_db = 20 * np.log10(magnitude + 1e-10)
phase = np.angle(h, deg=True)

# Encontrar frecuencia de -3dB
idx_3db = np.argmin(np.abs(magnitude_db + 3))
f_3db = w[idx_3db]

print(f"\nFrecuencia de corte real (-3dB): {f_3db:.3f} Hz")
print(f"Atenuación en 0 Hz (DC): {magnitude_db[0]:.2f} dB")

# ============ RESPUESTAS AL IMPULSO Y ESCALÓN ============
impulse_length = 200
impulse = np.zeros(impulse_length)
impulse[0] = 1
t = np.arange(impulse_length) / fs

# Respuesta al impulso
h_impulse = signal.lfilter(b_alpha, a_alpha, impulse)

# Respuesta al escalón
step = np.ones(impulse_length)
h_step = signal.lfilter(b_alpha, a_alpha, step)

# ============ CREAR FIGURA CON 4 SUBPLOTS ============
fig = plt.figure(figsize=(14, 10))
fig.suptitle(f'ANÁLISIS DEL FILTRO PASA-ALTO IIR (α={HP_ALPHA}, fc≈{fc_alpha:.2f}Hz, fs={fs}Hz)', 
             fontsize=14, fontweight='bold', y=0.98)

# -------- SUBPLOT 1: DIAGRAMA DE POLOS Y CEROS (PLANO Z) --------
ax1 = plt.subplot(2, 2, 1)
ax1.set_title('Diagrama de Polos y Ceros (Plano Z)', fontweight='bold', fontsize=12)

# Círculo unitario
circle = Circle((0, 0), 1, fill=False, color='black', linestyle='--', linewidth=1.5, label='Círculo unitario')
ax1.add_patch(circle)

# Ejes
ax1.axhline(y=0, color='k', linewidth=0.5)
ax1.axvline(x=0, color='k', linewidth=0.5)

# Graficar cero en z=1
ax1.plot(np.real(zeros[0]), np.imag(zeros[0]), 'o', markersize=14, 
         markerfacecolor='white', markeredgecolor='blue', markeredgewidth=3, 
         label='Cero (z=1)')

# Graficar polo en z=α
ax1.plot(np.real(poles[0]), np.imag(poles[0]), 'x', markersize=14, 
         color='red', markeredgewidth=3, label=f'Polo (z={HP_ALPHA})')

ax1.set_xlabel('Parte Real', fontsize=11)
ax1.set_ylabel('Parte Imaginaria', fontsize=11)
ax1.grid(True, alpha=0.3)
ax1.axis('equal')
ax1.set_xlim([-1.3, 1.3])
ax1.set_ylim([-1.3, 1.3])
ax1.legend(loc='upper left', fontsize=10)

# Anotar coordenadas
ax1.annotate(f'Cero en z=1\n(Bloquea DC/0Hz)', 
            xy=(np.real(zeros[0]), np.imag(zeros[0])), 
            xytext=(20, 30), textcoords='offset points', 
            fontsize=9, color='blue',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='lightblue', alpha=0.7),
            arrowprops=dict(arrowstyle='->', color='blue', lw=2))

ax1.annotate(f'Polo en z={HP_ALPHA}\n|z|={abs(poles[0]):.4f} < 1\n(Estable)', 
            xy=(np.real(poles[0]), np.imag(poles[0])), 
            xytext=(-80, -40), textcoords='offset points', 
            fontsize=9, color='red',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='lightyellow', alpha=0.7),
            arrowprops=dict(arrowstyle='->', color='red', lw=2))

# -------- SUBPLOT 2: RESPUESTA AL IMPULSO --------
ax2 = plt.subplot(2, 2, 2)
ax2.set_title('Respuesta al Impulso h[n]', fontweight='bold', fontsize=12)

markerline, stemlines, baseline = ax2.stem(t, h_impulse, linefmt='b-', 
                                           markerfmt='bo', basefmt='k-')
markerline.set_markerfacecolor('blue')
markerline.set_markersize(3)
stemlines.set_linewidth(1.2)

ax2.set_xlabel('Tiempo (s)', fontsize=11)
ax2.set_ylabel('Amplitud', fontsize=11)
ax2.grid(True, alpha=0.3)
ax2.axhline(y=0, color='k', linewidth=0.5)
ax2.set_xlim([0, 0.8])

# Añadir valores clave
max_impulse = np.max(h_impulse)
min_impulse = np.min(h_impulse)
ax2.text(0.98, 0.95, f'Máximo: {max_impulse:.4f}\nMínimo: {min_impulse:.4f}\n\nDecaimiento exponencial\ncon τ = α = {HP_ALPHA}', 
         transform=ax2.transAxes, fontsize=9,
         verticalalignment='top', horizontalalignment='right',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.6))

# -------- SUBPLOT 3: RESPUESTA AL ESCALÓN UNITARIO --------
ax3 = plt.subplot(2, 2, 3)
ax3.set_title('Respuesta al Escalón Unitario', fontweight='bold', fontsize=12)

ax3.plot(t, h_step, 'g-', linewidth=2.5, label='Respuesta')
ax3.axhline(y=0, color='r', linestyle='--', linewidth=1.5, alpha=0.7, 
            label='Nivel DC bloqueado')

ax3.set_xlabel('Tiempo (s)', fontsize=11)
ax3.set_ylabel('Amplitud', fontsize=11)
ax3.grid(True, alpha=0.3)
ax3.legend(loc='upper right', fontsize=10)
ax3.set_xlim([0, 0.8])

# Añadir información
valor_inicial = h_step[0]
valor_final = h_step[-1]
ax3.text(0.02, 0.95, f'Valor inicial: {valor_inicial:.4f}\nValor final: {valor_final:.4f}\n\n' +
                     f'El filtro pasa-alto\nbloquea la componente DC\ny tiende a cero', 
         transform=ax3.transAxes, fontsize=9,
         verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.6))

# -------- SUBPLOT 4: RESPUESTA EN FRECUENCIA - MAGNITUD --------
ax4 = plt.subplot(2, 2, 4)
ax4.set_title('Respuesta en Frecuencia - Magnitud', fontweight='bold', fontsize=12)

ax4.plot(w, magnitude_db, 'b', linewidth=2.5, label='Magnitud')
ax4.axhline(y=-3, color='green', linestyle='--', linewidth=1.5, label='-3 dB')
ax4.axvline(x=f_3db, color='orange', linestyle='--', linewidth=2, 
            label=f'fc = {f_3db:.2f} Hz')

ax4.set_xlabel('Frecuencia (Hz)', fontsize=11)
ax4.set_ylabel('Magnitud (dB)', fontsize=11)
ax4.grid(True, alpha=0.3)
ax4.legend(loc='lower right', fontsize=10)
ax4.set_xlim([0, 10])  # Zoom en bajas frecuencias
ax4.set_ylim([-40, 5])

# Marcar atenuación en DC
ax4.plot(0, magnitude_db[0], 'ro', markersize=8, label=f'DC: {magnitude_db[0]:.1f} dB')
ax4.legend(loc='lower right', fontsize=10)

# Añadir información
ax4.text(0.98, 0.05, f'Tipo: Pasa-Alto IIR\n' +
                     f'Orden: 1\n' +
                     f'α = {HP_ALPHA}\n' +
                     f'fc ≈ {f_3db:.2f} Hz\n\n' +
                     f'Atenúa frecuencias\npor debajo de fc\n' +
                     f'Bloquea DC (~{magnitude_db[0]:.0f} dB)', 
         transform=ax4.transAxes, fontsize=9,
         verticalalignment='bottom', horizontalalignment='right',
         bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.6))

plt.tight_layout()
plt.savefig('highpass_filter_alpha_analysis.png', dpi=300, bbox_inches='tight')
plt.show()

print("\n" + "=" * 70)
print("CÓDIGO ARDUINO:")
print("=" * 70)
print(f"const double HP_ALPHA = {HP_ALPHA};")
print(f"\n// Ecuación en diferencias:")
print(f"// y[n] = HP_ALPHA * (y[n-1] + x[n] - x[n-1])")
print(f"\ndouble applyHighPassFilter(double input) {{")
print(f"  double output = HP_ALPHA * (hp_y1 + input - hp_x1);")
print(f"  hp_x1 = input;")
print(f"  hp_y1 = output;")
print(f"  return output;")
print(f"}}")
print("=" * 70)
print("\n✅ Gráfica guardada como 'highpass_filter_alpha_analysis.png'")