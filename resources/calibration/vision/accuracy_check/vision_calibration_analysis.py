import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=str, default='vision_data_155cm.csv',
                        help='CSV generado por grabar_vision.py')
    args = parser.parse_args()

    df = pd.read_csv(args.input)
    n  = len(df)

    x     = df['x'].values
    y     = df['y'].values
    theta = df['theta'].values

    mean_x, mean_y, mean_th = x.mean(), y.mean(), theta.mean()
    std_x,  std_y,  std_th  = x.std(),  y.std(),  theta.std()
    var_x,  var_y,  var_th  = x.var(),  y.var(),  theta.var()

    print(f"\n{'='*55}")
    print(f"{'ANÁLISIS R_vision — EKF':^55}")
    print(f"{'='*55}")
    print(f"  Muestras : {n}")
    print(f"\n  {'':6} {'Media':>12} {'Std dev':>12} {'Varianza':>12}")
    print(f"  {'-'*44}")
    print(f"  {'X (m)':6} {mean_x:>12.5f} {std_x:>12.5f} {var_x:>12.6f}")
    print(f"  {'Y (m)':6} {mean_y:>12.5f} {std_y:>12.5f} {var_y:>12.6f}")
    print(f"  {'θ (rad)':6} {mean_th:>12.5f} {std_th:>12.5f} {var_th:>12.6f}")

    print(f"\n  ── Valores para R_vision (diagonal) ──")
    print(f"  R_vision = np.diag([{var_x:.6f}, {var_y:.6f}, {var_th:.6f}])")

    print(f"\n  ── Interpretación ──")
    print(f"  Precisión X  : ±{std_x*100:.1f} cm  (1σ)")
    print(f"  Precisión Y  : ±{std_y*100:.1f} cm  (1σ)")
    print(f"  Precisión θ  : ±{np.degrees(std_th):.2f}°  (1σ)")
    print(f"{'='*55}\n")

    fig, axs = plt.subplots(1, 3, figsize=(14, 4))
    fig.suptitle('Dispersión de mediciones de visión en reposo', fontsize=13)

    for ax, datos, etiqueta, unidad in zip(
        axs,
        [x, y, np.degrees(theta)],
        ['X', 'Y', 'θ'],
        ['m', 'm', '°']
    ):
        media = datos.mean()
        std   = datos.std()
        ax.plot(datos, lw=0.7, alpha=0.8)
        ax.axhline(media,       color='black', lw=1.2, ls='--', label=f'Media: {media:.4f}')
        ax.axhline(media + std, color='gray',  lw=0.9, ls=':',  label=f'±1σ ({std:.4f})')
        ax.axhline(media - std, color='gray',  lw=0.9, ls=':')
        ax.set_title(f'{etiqueta}  (σ={std:.4f} {unidad})')
        ax.set_xlabel('Muestra')
        ax.set_ylabel(f'{etiqueta} ({unidad})')
        ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig('figura_vision_dispersion.pdf', bbox_inches='tight', dpi=300)
    fig.savefig('figura_vision_dispersion.png', bbox_inches='tight', dpi=300)
    print('Figuras guardadas: figura_vision_dispersion.pdf/.png')

    
    fig2, ax2 = plt.subplots(figsize=(5, 5))
    ax2.scatter(x, y, s=8, alpha=0.5, label='Mediciones')
    ax2.scatter(mean_x, mean_y, s=80, color='red', zorder=5, label='Media')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Nube de puntos XY — visión en reposo')
    ax2.legend()
    ax2.set_aspect('equal')
    fig2.tight_layout()
    fig2.savefig('figura_vision_xy.pdf', bbox_inches='tight', dpi=300)
    fig2.savefig('figura_vision_xy.png', bbox_inches='tight', dpi=300)
    print('Figuras guardadas: figura_vision_xy.pdf/.png')

    plt.show()


if __name__ == '__main__':
    main()
