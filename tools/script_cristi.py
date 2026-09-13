import cv2
import numpy as np

# before using, run 'pip install opencv-contrib-python numpy'

# also run 'v4l2-ctl --list-formats-ext -d /dev/video0' to see available framerates and resolutions for the camera

# configs
CAMERA_INDEX = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
USE_MJPG = True  # in case camera doesn't support it, set to False.
SHOW_WINDOW = True
PRINT_COORDS = True
VERBOSE_MODE = False

# distance estimation configs
MARKER_SIZE_CM = 6.7             # real-world side length of the marker
DISTANCE_AT_FULL_FRAME_CM = 4.8   # distance at which the marker's width == full frame width
SHOW_DISTANCE = True

ARUCO_DICT, TARGET_MARKER_ID = cv2.aruco.DICT_4X4_50, 26  # DOES NOT NEED CHANGING UNLESS RULES CHANGE


def compute_focal_px(frame_width, marker_size_cm, distance_at_full_frame_cm):
    return (frame_width * distance_at_full_frame_cm) / marker_size_cm


def build_camera_matrix(focal_px, frame_width, frame_height):
    return np.array([
        [focal_px, 0, frame_width / 2],
        [0, focal_px, frame_height / 2],
        [0, 0, 1],
    ], dtype=np.float32)


def build_marker_object_points(marker_size_cm):
    half = marker_size_cm / 2.0
    return np.array([
        [-half,  half, 0],
        [ half,  half, 0],
        [ half, -half, 0],
        [-half, -half, 0],
    ], dtype=np.float32)


def main():
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if USE_MJPG:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    if USE_MJPG:
        cap.set(cv2.CAP_PROP_FPS, 60)

    if VERBOSE_MODE:
        print("Requested MJPG @ 1080p/60 — actual negotiated values:")
        print("FourCC:", "".join([chr((int(cap.get(cv2.CAP_PROP_FOURCC)) >> 8 * i) & 0xFF) for i in range(4)]))
        print("Resolution:", int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), "x", int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        print("FPS:", cap.get(cv2.CAP_PROP_FPS))

    if not cap.isOpened():
        raise RuntimeError(f"Vezi că index-ul {CAMERA_INDEX} e prost. Camera nu e acolo.")

    focal_px = compute_focal_px(FRAME_WIDTH, MARKER_SIZE_CM, DISTANCE_AT_FULL_FRAME_CM)
    camera_matrix = build_camera_matrix(focal_px, FRAME_WIDTH, FRAME_HEIGHT)
    dist_coeffs = np.zeros((5, 1), dtype=np.float32)  # assuming negligible lens distortion
    object_points = build_marker_object_points(MARKER_SIZE_CM)

    print("Pornire tracking. Apasă 'q' pentru a ieși.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("cap.read() a eșuat. Ieșire.")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners, ids, _rejected = detector.detectMarkers(gray)

            if ids is not None:
                for marker_corners, marker_id in zip(corners, ids.flatten()):
                    if marker_id != TARGET_MARKER_ID:
                        continue

                    pts = marker_corners.reshape((4, 2)).astype(np.float32)
                    center_x = int(np.mean(pts[:, 0]))
                    center_y = int(np.mean(pts[:, 1]))

                    ok, rvec, tvec = cv2.solvePnP(
                        object_points, pts, camera_matrix, dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE  # tailored for planar square markers
                    )
                    distance_cm = float(np.linalg.norm(tvec)) if ok else None

                    if PRINT_COORDS:
                        corner_str = ", ".join(f"({x:.1f}, {y:.1f})" for x, y in pts)
                        dist_str = f", distance={distance_cm:.1f}cm" if distance_cm is not None else ""
                        print(f"Marker ID {marker_id}: center=({center_x}, {center_y}), corners=[{corner_str}]{dist_str}")

                    if SHOW_WINDOW:
                        cv2.aruco.drawDetectedMarkers(frame, [marker_corners])
                        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                        cv2.putText(
                            frame,
                            f"ID {marker_id}: ({center_x}, {center_y})",
                            (center_x + 10, center_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 0),
                            2,
                        )

                        if SHOW_DISTANCE and distance_cm is not None:
                            dist_text = f"{distance_cm:.1f} cm"
                            (text_w, text_h), _ = cv2.getTextSize(
                                dist_text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2
                            )
                            margin = 15
                            x = FRAME_WIDTH - text_w - margin
                            y = margin + text_h
                            cv2.putText(
                                frame,
                                dist_text,
                                (x, y),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.8,
                                (255, 0, 0),
                                2,
                            )

            if SHOW_WINDOW:
                cv2.imshow("ArUco Tracker", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    finally:
        cap.release()
        if SHOW_WINDOW:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
