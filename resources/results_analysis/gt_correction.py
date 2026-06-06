"""
Corrige el GT aplicando una transformación afín 2D calculada a partir de
los 4 puntos conocidos del cuadrado.

El GT ya viene en metros pero tiene errores de offset, rotación y escala.
Esta calibración aplica la transformación afín que convierte las
coordenadas erróneas del GT en coordenadas reales del sistema del robot.

Uso:
    python gt_correction.py --gt gt.csv --out gt_corregido.csv \
        --cal_gt -0.02 0.07  1.26 0.07  1.26 -1.26  -0.07 -1.27

Los 8 valores son las coordenadas que el GT REPORTÓ cuando el robot estaba
físicamente en cada esquina del cuadrado, en este orden:
    p1: GT(x0,y0) cuando robot en (0,    0)
    p2: GT(x1,y1) cuando robot en (1.20, 0)
    p3: GT(x2,y2) cuando robot en (1.20, -1.20)
    p4: GT(x3,y3) cuando robot en (0,    -1.20)
"""

import sys
import argparse
import numpy as np
import pandas as pd


def calcular_afin(src_pts, dst_pts):
    """
    Calcula transformación afín 2D por mínimos cuadrados.
    [x_real]   [a b tx] [x_gt]
    [y_real] = [c d ty] [y_gt]
                        [ 1  ]
    Resuelve los 6 parámetros (a,b,tx,c,d,ty) con 4 puntos de correspondencia
    (8 ecuaciones, sistema sobre-determinado).
    """
    A, b = [], []
    for (xs, ys), (xd, yd) in zip(src_pts, dst_pts):
        A.append([xs, ys, 1, 0,  0,  0])  
        A.append([0,  0,  0, xs, ys, 1])  
        b.append(xd)
        b.append(yd)

    A = np.array(A, dtype=np.float64)
    b = np.array(b, dtype=np.float64)
    params, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    M = params.reshape(2, 3)
    return M


def aplicar_afin(df, M):
    xy1 = np.column_stack([
        df['x_m'].values,
        df['y_m'].values,
        np.ones(len(df))
    ])
    xy_corr = (M @ xy1.T).T

    df_out = df.copy()
    df_out['x_m'] = xy_corr[:, 0]
    df_out['y_m'] = xy_corr[:, 1]
    return df_out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gt',  required=True, help='CSV GT original')
    parser.add_argument('--out', required=True, help='CSV GT corregido')
    parser.add_argument('--cal_gt', required=True, nargs=8, type=float,
                        metavar=('x0','y0','x1','y1','x2','y2','x3','y3'),
                        help='8 valores: lo que el GT midió en cada esquina')
    args = parser.parse_args()

    v = args.cal_gt
    src_pts = [
        [v[0], v[1]],   # GT cuando robot en (0,    0)
        [v[2], v[3]],   # GT cuando robot en (1.20, 0)
        [v[4], v[5]],   # GT cuando robot en (1.20, -1.20)
        [v[6], v[7]],   # GT cuando robot en (0,    -1.20)
    ]
    dst_pts = [
        [0.0,  0.0],
        [1.2,  0.0],
        [1.2, -1.2],
        [0.0, -1.2],
    ]

    print("=" * 60)
    print("DIAGNÓSTICO DE ERRORES DEL GT")
    print("=" * 60)
    d_p1p2 = np.sqrt((v[2]-v[0])**2 + (v[3]-v[1])**2)
    d_p2p3 = np.sqrt((v[4]-v[2])**2 + (v[5]-v[3])**2)
    d_p3p4 = np.sqrt((v[6]-v[4])**2 + (v[7]-v[5])**2)
    d_p4p1 = np.sqrt((v[0]-v[6])**2 + (v[1]-v[7])**2)
    print(f"  Lado P1→P2 (real=1.20m): GT mide {d_p1p2:.3f}m  "
          f"(error escala: {(d_p1p2-1.2)/1.2*100:+.1f}%)")
    print(f"  Lado P2→P3 (real=1.20m): GT mide {d_p2p3:.3f}m  "
          f"(error escala: {(d_p2p3-1.2)/1.2*100:+.1f}%)")
    print(f"  Lado P3→P4 (real=1.20m): GT mide {d_p3p4:.3f}m  "
          f"(error escala: {(d_p3p4-1.2)/1.2*100:+.1f}%)")
    print(f"  Lado P4→P1 (real=1.20m): GT mide {d_p4p1:.3f}m  "
          f"(error escala: {(d_p4p1-1.2)/1.2*100:+.1f}%)")

    # Calcular transformacion
    M = calcular_afin(src_pts, dst_pts)
    print(f"\n" + "=" * 60)
    print("TRANSFORMACIÓN AFÍN CALCULADA")
    print("=" * 60)
    print(f"  [a b tx]   [{M[0,0]:+.4f}  {M[0,1]:+.4f}  {M[0,2]:+.4f}]")
    print(f"  [c d ty] = [{M[1,0]:+.4f}  {M[1,1]:+.4f}  {M[1,2]:+.4f}]")

    print(f"\nVerificación en las 4 esquinas:")
    for src, dst in zip(src_pts, dst_pts):
        corr = M @ np.array([src[0], src[1], 1.0])
        err  = np.sqrt((corr[0]-dst[0])**2 + (corr[1]-dst[1])**2) * 100
        print(f"  GT=({src[0]:+.3f}, {src[1]:+.3f}) → "
              f"corregido=({corr[0]:+.4f}, {corr[1]:+.4f})  "
              f"real=({dst[0]:+.1f}, {dst[1]:+.1f})  "
              f"error={err:.2f}cm")

    print(f"\n" + "=" * 60)
    print(f"PROCESANDO CSV: {args.gt}")
    print("=" * 60)
    df = pd.read_csv(args.gt)
    print(f"  {len(df)} muestras")
    print(f"  Rango ORIGINAL:")
    print(f"    X: [{df['x_m'].min():+.3f}, {df['x_m'].max():+.3f}]")
    print(f"    Y: [{df['y_m'].min():+.3f}, {df['y_m'].max():+.3f}]")

    df_corr = aplicar_afin(df, M)

    print(f"  Rango CORREGIDO:")
    print(f"    X: [{df_corr['x_m'].min():+.3f}, {df_corr['x_m'].max():+.3f}]")
    print(f"    Y: [{df_corr['y_m'].min():+.3f}, {df_corr['y_m'].max():+.3f}]")

    df_corr.to_csv(args.out, index=False)
    print(f"\n[OK] GT corregido guardado: {args.out}")


if __name__ == '__main__':
    main()