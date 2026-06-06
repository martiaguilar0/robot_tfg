"""
Extrae la trayectoria real del robot a partir de un video cenital.
Corrige el offset entre el marcador ArUco y el centro real del robot.

Uso:
    python3 ground_truth_tracker.py --video video.mp4 --output ground_truth.csv
    python3 ground_truth_tracker.py --video video.mp4 --skip 3 --scale 0.5 --no_preview

Configurar antes de usar:
    ROBOT_MARKER_ID   — ID del marcador encima del robot
    FLOOR_MARKERS     — IDs y posiciones (x,y) en metros de los 4 marcadores del suelo
    MARKER_OFFSET_X   — distancia en metros del marcador al centro del robot (eje X del robot)
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import pandas as pd
import argparse
import math

ROBOT_MARKER_ID = 501   # ID marcador del robot
MARKER_OFFSET_X = 0.03  
FLOOR_MARKERS = {
    497: np.array([0.25, -0.25]),
    498: np.array([0.95, -0.25]),
    499: np.array([0.95, -0.95]),
    500: np.array([0.25, -0.95]),
}

def build_homography(centers, floor_markers):
    """Calcula homografia pixeles -> metros usando los marcadores del suelo."""
    src, dst = [], []
    for mid, pos_m in floor_markers.items():
        if mid in centers:
            src.append(centers[mid])
            dst.append(pos_m)
    if len(src) < 4:
        return None
    H, _ = cv2.findHomography(
        np.array(src, dtype=np.float32),
        np.array(dst, dtype=np.float32)
    )
    return H


def pixel_to_world(px, py, H):
    """Convierte pixeles a metros con la homografia H."""
    pt = cv2.perspectiveTransform(
        np.array([[[float(px), float(py)]]], dtype=np.float32), H)
    return float(pt[0][0][0]), float(pt[0][0][1])


def marker_center(corners):
    """Centro de un marcador (media de sus 4 esquinas)."""
    return corners[0].mean(axis=0)


def marker_yaw(corners_orig, H):
    """
    Estima el yaw del marcador en el sistema mundo.
    Usa el vector entre esquina 0 y esquina 1 (lado superior del marcador).
    """
    c = corners_orig[0]
    x0, y0 = pixel_to_world(c[0][0], c[0][1], H)
    x1, y1 = pixel_to_world(c[1][0], c[1][1], H)
    return math.atan2(y1 - y0, x1 - x0)


def process_video(video_path, output_csv="ground_truth.csv",
                  show_preview=True, skip=3, scale=0.5):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise FileNotFoundError(f"No se puede abrir: {video_path}")

    fps      = cap.get(cv2.CAP_PROP_FPS)
    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"Video: {n_frames} frames a {fps:.1f} FPS ({n_frames/fps:.1f}s)")
    print(f"Skip: 1/{skip} frames  |  Scale: {scale}  |  Offset marcador: {MARKER_OFFSET_X}m")

    dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
    params     = aruco.DetectorParameters()
    detector   = aruco.ArucoDetector(dictionary, params)

    H          = None
    trajectory = []
    frame_idx  = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Saltar frames
        if frame_idx % skip != 0:
            frame_idx += 1
            continue

        if scale != 1.0:
            h_orig, w_orig = frame.shape[:2]
            frame_small = cv2.resize(frame,
                                     (int(w_orig * scale), int(h_orig * scale)))
        else:
            frame_small = frame

        gray = cv2.cvtColor(frame_small, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            ids_flat = ids.flatten()

            centers_scaled = {}
            centers_orig   = {}
            corners_orig_d = {}

            for i, mid in enumerate(ids_flat):
                mid = int(mid)
                centers_scaled[mid] = marker_center(corners_list[i])
                if scale != 1.0:
                    centers_orig[mid]   = centers_scaled[mid] / scale
                    corners_orig_d[mid] = corners_list[i] / scale
                else:
                    centers_orig[mid]   = centers_scaled[mid].copy()
                    corners_orig_d[mid] = corners_list[i].copy()
            floor_visible = {k: v for k, v in centers_orig.items()
                             if k in FLOOR_MARKERS}
            if len(floor_visible) >= 4:
                H_new = build_homography(floor_visible, FLOOR_MARKERS)
                if H_new is not None:
                    H = H_new
            if ROBOT_MARKER_ID in centers_orig and H is not None:
                cx, cy   = centers_orig[ROBOT_MARKER_ID]
                x_m, y_m = pixel_to_world(cx, cy, H)

                yaw = marker_yaw(corners_orig_d[ROBOT_MARKER_ID], H)

                x_robot = x_m - MARKER_OFFSET_X * math.cos(yaw)
                y_robot = y_m - MARKER_OFFSET_X * math.sin(yaw)

                time_s = frame_idx / fps
                trajectory.append({
                    'time_s':    round(time_s,    3),
                    'x_m':       round(x_robot,   4),
                    'y_m':       round(y_robot,    4),
                    'theta_rad': round(yaw,        4),
                })

            if show_preview:
                vis = frame_small.copy()
                aruco.drawDetectedMarkers(vis, corners_list, ids)
                if ROBOT_MARKER_ID in centers_scaled and H is not None:
                    cx_s, cy_s = centers_scaled[ROBOT_MARKER_ID].astype(int)
                    cv2.circle(vis, (cx_s, cy_s), 8, (0, 255, 0), -1)
                    xr, yr = centers_orig[ROBOT_MARKER_ID]
                    xw, yw = pixel_to_world(xr, yr, H)
                    cv2.putText(vis,
                        f"({x_robot:.2f}, {y_robot:.2f}) m",
                        (cx_s + 10, cy_s - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow("Ground Truth Tracker", vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        frame_idx += 1
        if frame_idx % (100 * skip) == 0:
            pct = frame_idx / n_frames * 100
            print(f"  {frame_idx}/{n_frames} ({pct:.0f}%) "
                  f"-> {len(trajectory)} detecciones")

    cap.release()
    cv2.destroyAllWindows()

    df = pd.DataFrame(trajectory,
                      columns=['time_s', 'x_m', 'y_m', 'theta_rad'])
    if df.empty:
        print(" Marcador del robot en ningun frame.")
    else:
        df.to_csv(output_csv, index=False)
        print(f"\nGuardado: {output_csv}  ({len(df)} muestras)")

    return df


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--video",      required=True,
                        help="Video grabado con el movil")
    parser.add_argument("--output",     default="ground_truth.csv",
                        help="CSV de salida (default: ground_truth.csv)")
    parser.add_argument("--no_preview", action="store_true",
                        help="No mostrar ventana durante el procesado")
    parser.add_argument("--skip",       type=int,   default=3,
                        help="Procesar 1 de cada N frames (default 3)")
    parser.add_argument("--scale",      type=float, default=0.5,
                        help="Escala de resolucion para ArUco (default 0.5)")
    args = parser.parse_args()

    process_video(
        args.video,
        output_csv=args.output,
        show_preview=not args.no_preview,
        skip=args.skip,
        scale=args.scale,
    )
