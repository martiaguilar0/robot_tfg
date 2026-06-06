import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import norm
from scipy.signal import welch


CSV_FILE = 'gyro.csv'
FS       = 100.0   # frecuencia de muestreo


df = pd.read_csv(CSV_FILE, names=['sample', 'gx', 'gy', 'gz'])
ejes    = ['gx', 'gy', 'gz']
colores = ['#e74c3c', '#2ecc71', '#3498db']
N = len(df)
t = np.arange(N) / FS 

print(f"\n{'='*60}")
print(f"{'RESUMEN — GIROSCOPIO EN REPOSO (ICM-20948)':^60}")
print(f"{'='*60}")
print(f"  Muestras : {N}   |   Fs : {FS} Hz   |   Duracion : {N/FS:.1f} s")
print(f"\n  {'Eje':<6} {'Bias (dps)':>12} {'sigma (dps)':>12} {'Densidad (dps/rtHz)':>21}")
print(f"  {'-'*53}")
for eje in ejes:
    d = df[eje]
    mu, sd = d.mean(), d.std()
    nd = sd / np.sqrt(FS / 2)
    print(f"  {eje.upper():<6} {mu:>12.5f} {sd:>12.5f} {nd:>21.6f}")
print(f"\n  Densidad espectral = sigma / sqrt(Fs/2)\n")


def autocorrelacion(x, max_lag):
    x = x - x.mean()
    c = np.correlate(x, x, mode='full')
    c = c[len(x) - 1:]
    c = c / c[0]
    return c[:max_lag]


def allan_deviation(data, fs, n_taus=150):
    max_m = len(data) // 3
    ms = np.unique(np.logspace(0, np.log10(max(max_m, 2)), n_taus).astype(int))
    taus, adevs = [], []
    for m in ms:
        n_clusters = len(data) // m
        if n_clusters < 2:
            continue
        clusters = data[:n_clusters * m].reshape(n_clusters, m).mean(axis=1)
        adev = np.sqrt(0.5 * np.mean(np.diff(clusters) ** 2))
        taus.append(m / fs)
        adevs.append(adev)
    return np.array(taus), np.array(adevs)


plt.style.use('seaborn-v0_8-muted')

# Figura 1
fig1, axs = plt.subplots(3, 2, figsize=(15, 12))
fig1.suptitle('Serie Temporal y Distribución de Ruido — ICM-20948', fontsize=15)

for i, eje in enumerate(ejes):
    datos = df[eje]
    mu, sd = datos.mean(), datos.std()

    ax = axs[i, 0]
    ax.plot(t, datos, color=colores[i], lw=0.5, alpha=0.8)
    ax.axhline(mu,        color='black', ls='--', lw=1.2, label=f'Bias: {mu:.4f} dps')
    ax.axhline(mu + sd,   color='gray',  ls=':',  lw=1.0, label=f'±1σ ({sd:.4f} dps)')
    ax.axhline(mu - sd,   color='gray',  ls=':',  lw=1.0)
    ax.axhline(mu + 3*sd, color='gray',  ls='-.', lw=0.8, alpha=0.6, label='±3σ')
    ax.axhline(mu - 3*sd, color='gray',  ls='-.', lw=0.8, alpha=0.6)
    ax.set_title(f'Serie Temporal — {eje.upper()}')
    ax.set_ylabel('Vel. Angular (dps)')
    ax.set_xlabel('Tiempo (s)')
    ax.legend(fontsize=8)

    ax = axs[i, 1]
    ax.hist(datos, bins=60, density=True, alpha=0.65, color=colores[i],
            label='Datos medidos')
    x_gauss = np.linspace(mu - 4.5*sd, mu + 4.5*sd, 300)
    ax.plot(x_gauss, norm.pdf(x_gauss, mu, sd), 'k', lw=2,
            label=f'N(μ={mu:.4f}, σ={sd:.4f})')
    for k, ls in [(1, ':'), (3, '-.')]:
        ax.axvline(mu + k*sd, color='gray', ls=ls, lw=0.9)
        ax.axvline(mu - k*sd, color='gray', ls=ls, lw=0.9)
    ax.set_title(f'Distribución — {eje.upper()}')
    ax.set_xlabel('Vel. Angular (dps)')
    ax.set_ylabel('Densidad de probabilidad')
    ax.legend(fontsize=8)

fig1.tight_layout(rect=[0, 0, 1, 0.96])
fig1.savefig('figura_gyro_serie_distribucion.pdf', bbox_inches='tight', dpi=300)
fig1.savefig('figura_gyro_serie_distribucion.png', bbox_inches='tight', dpi=300)

# Figura 2
fig2, axs2 = plt.subplots(3, 2, figsize=(15, 12))
fig2.suptitle('Análisis Frecuencial y Correlación del Ruido — ICM-20948', fontsize=15)

for i, eje in enumerate(ejes):
    datos_c = df[eje].values - df[eje].mean()

    nperseg = min(1024, max(N // 4, 64))
    f, Pxx = welch(datos_c, fs=FS, nperseg=nperseg)
    ax = axs2[i, 0]
    ax.semilogy(f[1:], np.sqrt(Pxx[1:]), color=colores[i], lw=1.0)
    ax.set_title(f'Densidad Espectral de Potencia — {eje.upper()}')
    ax.set_xlabel('Frecuencia (Hz)')
    ax.set_ylabel('Amplitud (dps/√Hz)')
    ax.grid(True, which='both', alpha=0.3)

    max_lag = min(500, N // 2)
    ac = autocorrelacion(datos_c, max_lag)
    lags_s = np.arange(max_lag) / FS
    ci = 1.96 / np.sqrt(N)
    ax = axs2[i, 1]
    ax.plot(lags_s, ac, color=colores[i], lw=0.8)
    ax.axhline(0,   color='black', lw=0.8)
    ax.axhline( ci, color='red',   ls='--', lw=0.9, label='IC 95%')
    ax.axhline(-ci, color='red',   ls='--', lw=0.9)
    ax.set_title(f'Autocorrelación — {eje.upper()}')
    ax.set_xlabel('Lag (s)')
    ax.set_ylabel('Correlación')
    ax.set_ylim(-0.25, 1.1)
    ax.legend(fontsize=8)

fig2.tight_layout(rect=[0, 0, 1, 0.96])
fig2.savefig('figura_gyro_psd_autocorrelacion.pdf', bbox_inches='tight', dpi=300)
fig2.savefig('figura_gyro_psd_autocorrelacion.png', bbox_inches='tight', dpi=300)

# Figura 3
fig3, ax3 = plt.subplots(figsize=(10, 6))
ax3.set_title('Desviación de Allan — Giroscopio en Reposo (ICM-20948)', fontsize=14)

for i, eje in enumerate(ejes):
    taus, adevs = allan_deviation(df[eje].values, FS)
    ax3.loglog(taus, adevs, color=colores[i], lw=1.8, label=eje.upper())
    if taus[0] <= 1.0 <= taus[-1]:
        idx = np.argmin(np.abs(taus - 1.0))
        ax3.scatter(taus[idx], adevs[idx], color=colores[i], zorder=5, s=50)
        ax3.annotate(f'  {adevs[idx]:.4f} dps', xy=(taus[idx], adevs[idx]),
                     fontsize=8, color=colores[i], va='center')

ax3.set_xlabel('Tiempo de integración τ (s)')
ax3.set_ylabel('ADEV (dps)')
ax3.legend()
ax3.grid(True, which='both', alpha=0.3)
fig3.tight_layout()
fig3.savefig('figura_gyro_allan.pdf', bbox_inches='tight', dpi=300)
fig3.savefig('figura_gyro_allan.png', bbox_inches='tight', dpi=300)

plt.show()

print("\nFiguras exportadas:")
print("  figura_gyro_serie_distribucion.pdf/.png")
print("  figura_gyro_psd_autocorrelacion.pdf/.png")
print("  figura_gyro_allan.pdf/.png")
