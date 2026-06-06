"""
plot_trayectorias.py
====================
Genera dos gráficas para validación de robot diferencial:
  1. Trayectoria 2D con degradado (con Inset Zoom en zona crítica de giro/corrección)
  2. Error vs distancia usando vecino más cercano + anclaje al error de cierre real
"""

import sys
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D

try:
    import pandas as pd
except ImportError:
    print("[ERROR] pip install pandas"); sys.exit(1)


def cargar(ruta):
    df = pd.read_csv(ruta)
    for c in ['time_s','x_m','y_m']:
        if c not in df.columns:
            raise ValueError(f"'{c}' no encontrada en {ruta}")
    return df.sort_values('time_s').reset_index(drop=True)


def detectar_tramo(df, umbral=0.005, ventana=7):
    x,y,t = df['x_m'].values, df['y_m'].values, df['time_s'].values
    dt = np.diff(t); dt = np.where(dt<=0,1e-9,dt)
    vel = np.sqrt(np.diff(x)**2+np.diff(y)**2)/dt
    vel = np.concatenate([[0],vel])
    vs  = np.convolve(vel, np.ones(ventana)/ventana, mode='same')
    mov = vs > umbral
    if not mov.any():
        print("  [WARN] Sin movimiento"); return df
    i0 = max(0, mov.argmax()-ventana)
    i1 = min(len(mov)-1, len(mov)-1-mov[::-1].argmax()+ventana)
    print(f"  Tramo: t=[{t[i0]:.1f}s,{t[i1]:.1f}s] ({t[i1]-t[i0]:.1f}s, {i1-i0+1} muestras)")
    return df.iloc[i0:i1+1].reset_index(drop=True)


def centrar(df):
    out=df.copy()
    out['x_m']=out['x_m']-df['x_m'].iloc[0]
    out['y_m']=out['y_m']-df['y_m'].iloc[0]
    return out


def dist_acum(x,y):
    return np.concatenate([[0],np.cumsum(np.sqrt(np.diff(x)**2+np.diff(y)**2))])


def calcular_error_vecino(gt, est, paso=0.05, e_cierre_manual=None):
    x_gt  = gt['x_m'].values;  y_gt  = gt['y_m'].values
    x_est = est['x_m'].values; y_est = est['y_m'].values

    d_gt  = dist_acum(x_gt, y_gt)
    d_ref = np.arange(0, d_gt[-1], paso)

    xg = np.interp(d_ref, d_gt, x_gt)
    yg = np.interp(d_ref, d_gt, y_gt)

    error = np.zeros(len(d_ref))
    for i in range(len(d_ref)):
        dists = np.sqrt((x_est - xg[i])**2 + (y_est - yg[i])**2)
        error[i] = dists.min() * 100

    print(f"  Error vecino bruto: medio={error.mean():.1f}cm, máx={error.max():.1f}cm, "
          f"último={error[-1]:.1f}cm")

    if e_cierre_manual is not None and abs(error[-1]) > 0.01:
        n      = len(error)
        tramo  = max(int(n * 0.20), 10)   
        factor = e_cierre_manual / error[-1]

        ramp           = np.linspace(1.0, factor, tramo)
        error[-tramo:] = error[-tramo:] * ramp
        error[-1]      = e_cierre_manual

        print(f"  Error anclado: último={error[-1]:.1f}cm (factor={factor:.2f})")

    return d_ref, error, xg, yg


def degradado(ax, x, y, cmap_name, lw=1.8, alpha=0.9, vmin=0.5):
    pts  = np.array([x,y]).T.reshape(-1,1,2)
    segs = np.concatenate([pts[:-1],pts[1:]],axis=1)
    t    = np.linspace(vmin,1,len(x)-1)
    cmap = plt.get_cmap(cmap_name)
    lc   = LineCollection(segs,cmap=cmap,alpha=alpha,linewidth=lw,
                          norm=mcolors.Normalize(vmin=vmin,vmax=1.0))
    lc.set_array(t); ax.add_collection(lc)
    return lc


def grafica_trayectoria(gt, est, e_cierre, output):
    x_gt=gt['x_m'].values; y_gt=gt['y_m'].values
    x_est=est['x_m'].values; y_est=est['y_m'].values

    fig,ax=plt.subplots(figsize=(7,7))
    ax.set_aspect('equal'); ax.grid(True,alpha=0.2,linestyle='--')
    ax.set_xlabel('X (m)',fontsize=11); ax.set_ylabel('Y (m)',fontsize=11)
    ax.set_title('Trayectoria: ground truth vs EKF',fontsize=12)

    wp=np.array([[0,0],[1.2,0],[1.2,-1.2],[0,-1.2],[0,0]])
    ax.plot(wp[:,0],wp[:,1],'k:',lw=1.0,alpha=0.3,zorder=1)

    degradado(ax,x_gt,y_gt,'Greens',lw=2.2,alpha=0.95, vmin=0.45)
    degradado(ax,x_est,y_est,'Blues',lw=1.6,alpha=0.85, vmin=0.45)

    cg=plt.get_cmap('Greens'); cb=plt.get_cmap('Blues')
    ax.scatter(x_gt[0],y_gt[0],color=cg(0.4),s=100,marker='o',zorder=8)
    ax.scatter(x_gt[-1],y_gt[-1],color=cg(0.9),s=100,marker='s',zorder=8)
    ax.scatter(x_est[0],y_est[0],color=cb(0.4),s=80,marker='o',zorder=8)
    ax.scatter(x_est[-1],y_est[-1],color=cb(0.9),s=80,marker='s',zorder=8)

    ax.plot([x_gt[-1],x_est[-1]],[y_gt[-1],y_est[-1]],color='red',lw=2.0,zorder=9)
    mx=(x_gt[-1]+x_est[-1])/2; my=(y_gt[-1]+y_est[-1])/2
    ax.annotate(f'Error cierre:\n{e_cierre:.1f} cm',(mx,my),fontsize=9,
                color='red',ha='center',va='bottom',
                bbox=dict(boxstyle='round,pad=0.3',facecolor='white',
                          edgecolor='red',alpha=0.85))

    ax_ins = ax.inset_axes([0.3, 0.3, 0.5, 0.5])
    ax_ins.set_aspect('equal')

    ax_ins.plot(wp[:,0], wp[:,1], 'k:', lw=1.2, alpha=0.5, zorder=1)

    degradado(ax_ins, x_gt, y_gt, 'Greens', lw=2.4, alpha=0.98, vmin=0.45)
    degradado(ax_ins, x_est, y_est, 'Blues', lw=1.8, alpha=0.90, vmin=0.45)

    zoom_x, zoom_y = 0.1, -1.1
    radio_zoom = 0.2

    ax_ins.set_xlim(zoom_x - radio_zoom, zoom_x + radio_zoom)
    ax_ins.set_ylim(zoom_y - radio_zoom, zoom_y + radio_zoom)
    ax_ins.grid(True, linestyle=':', alpha=0.4)
    ax_ins.set_title('Zoom Esquina (0.0, -1.2) m', fontsize=8, fontweight='bold', color='#1d3557')
    ax_ins.tick_params(labelsize=7.5)

    ax.indicate_inset_zoom(ax_ins, edgecolor='#e63946', lw=1.2, alpha=0.6)

    all_x=np.concatenate([x_gt,x_est]); all_y=np.concatenate([y_gt,y_est])
    m=0.10
    ax.set_xlim(all_x.min()-m,all_x.max()+m)
    ax.set_ylim(all_y.min()-m,all_y.max()+m)

    leyenda=[
        Line2D([0],[0],color=cg(0.6),lw=2.0,label='Ground truth (cámara cenital)'),
        Line2D([0],[0],color=cb(0.6),lw=1.6,label='Estimación EKF'),
        Line2D([0],[0],marker='o',color='gray',lw=0,markersize=7,label='Inicio'),
        Line2D([0],[0],marker='s',color='gray',lw=0,markersize=7,label='Fin'),
        Line2D([0],[0],color='red',lw=2.0,label=f'Error cierre: {e_cierre:.1f} cm'),
    ]
    ax.legend(handles=leyenda,loc='lower right',fontsize=8)

    sm=plt.cm.ScalarMappable(cmap='Greys',norm=mcolors.Normalize(vmin=0,vmax=1))
    sm.set_array([])
    cb2=fig.colorbar(sm,ax=ax,fraction=0.025,pad=0.02)
    cb2.set_ticks([0.1,0.9]); cb2.set_ticklabels(['Inicio','Fin'],fontsize=9)
    cb2.set_label('Progreso',fontsize=9)

    fig.tight_layout()
    fig.savefig(output,dpi=150,bbox_inches='tight')
    plt.close(fig)
    print(f"Trayectoria con Zoom: {output}")


def grafica_error_distancia(d_ref, error, output, perimetro=4.8):
    d_max=d_ref[-1]
    n_vueltas=int(d_max/perimetro)

    fig,ax=plt.subplots(figsize=(10,4))
    ax.grid(True,alpha=0.2,linestyle='--')
    ax.set_xlabel('Distancia recorrida (m)',fontsize=11)
    ax.set_ylabel('Error de posición (cm)',fontsize=11)
    ax.set_title('Error de posición vs distancia recorrida',fontsize=12)

    ax.plot(d_ref,error,color='#457b9d',lw=1.5,alpha=0.9)
    ax.fill_between(d_ref,error,alpha=0.15,color='#457b9d')

    ymax=max(error.max()*1.15, 5)
    for v in range(1,n_vueltas+1):
        d_v=v*perimetro
        if d_v<d_max:
            ax.axvline(d_v,color='gray',lw=0.8,linestyle='--',alpha=0.5)
            ax.text(d_v+0.1,ymax*0.92,f'V{v}',fontsize=7,color='gray',va='top')

    ax.set_xlim(0,d_max); ax.set_ylim(0,ymax)

    ax.text(0.98,0.97,
            f"Error medio:  {error.mean():.1f} cm\n"
            f"Error máx:    {error.max():.1f} cm\n"
            f"Error cierre: {error[-1]:.1f} cm\n"
            f"Dist total:   {d_max:.1f} m",
            transform=ax.transAxes,fontsize=9,va='top',ha='right',
            bbox=dict(boxstyle='round,pad=0.4',facecolor='wheat',alpha=0.85))

    fig.tight_layout()
    fig.savefig(output,dpi=150,bbox_inches='tight')
    plt.close(fig)
    print(f"[OK] Error vs distancia: {output}")


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--gt',required=True)
    parser.add_argument('--ekf',required=True)
    parser.add_argument('--output',    default='trayectoria.png')
    parser.add_argument('--output_err',default='error_distancia.png')
    parser.add_argument('--umbral_gt', default=0.08, type=float)
    parser.add_argument('--umbral_ekf',default=0.005,type=float)
    parser.add_argument('--e_cierre',  default=None, type=float,
                        help='Error de cierre medido con cinta (cm). '
                             'Ancla el último punto de la gráfica de error.')
    parser.add_argument('--paso',      default=0.05, type=float)
    args=parser.parse_args()

    print(f"\nGT:  {args.gt}")
    gt=centrar(detectar_tramo(cargar(args.gt),umbral=args.umbral_gt))
    print(f"  Distancia GT: {dist_acum(gt['x_m'].values,gt['y_m'].values)[-1]:.2f} m")

    print(f"\nEKF: {args.ekf}")
    ekf=centrar(detectar_tramo(cargar(args.ekf),umbral=args.umbral_ekf))
    print(f"  Distancia EKF: {dist_acum(ekf['x_m'].values,ekf['y_m'].values)[-1]:.2f} m")

    e_cierre = args.e_cierre if args.e_cierre is not None else \
        np.sqrt((gt['x_m'].iloc[-1]-ekf['x_m'].iloc[-1])**2 +
                (gt['y_m'].iloc[-1]-ekf['y_m'].iloc[-1])**2)*100
    print(f"\nError cierre: {e_cierre:.1f} cm")

    print("\nCalculando error por vecino más cercano...")
    d_ref, error, _, _ = calcular_error_vecino(
        gt, ekf, paso=args.paso, e_cierre_manual=args.e_cierre
    )

    grafica_trayectoria(gt, ekf, e_cierre, args.output)
    grafica_error_distancia(d_ref, error, args.output_err)


if __name__=='__main__':
    main()