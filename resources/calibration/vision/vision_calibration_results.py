import cv2, numpy as np, glob

SQUARE_MM = 25

objp = np.zeros((6*9, 3), np.float32)
objp[:,:2] = np.mgrid[0:9, 0:6].T.reshape(-1,2) * (SQUARE_MM / 1000.0)

obj_pts, img_pts = [], []
for f in sorted(glob.glob('calib_imgs/*.jpg')):
    gray = cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (9,6), None)
    if ret:
        corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                  (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        obj_pts.append(objp); img_pts.append(corners)
        print(f"OK: {f}")
    else:
        print(f"SKIP: {f}")

err, K, D, _, _ = cv2.calibrateCamera(obj_pts, img_pts, (1280, 720), None, None)

print(f"\nError de reproyeccion: {err:.3f} px  (bueno si < 1.0)")
print(f"\ncamera_matrix = np.array([")
print(f"    [{K[0,0]:.4f}, 0,       {K[0,2]:.4f}],")
print(f"    [0,       {K[1,1]:.4f}, {K[1,2]:.4f}],")
print(f"    [0,       0,       1      ]")
print(f"], dtype=np.float32)")
print(f"\ndist_coeffs = np.array([[{D[0,0]:.6f}, {D[0,1]:.6f}, "
      f"{D[0,2]:.6f}, {D[0,3]:.6f}, {D[0,4]:.6f}]], dtype=np.float32)")