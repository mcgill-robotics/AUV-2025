#!/usr/bin/env python3
import cv2
import numpy as np
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description="Camera calibration using a chessboard pattern.")
    parser.add_argument("--frames", type=int, default=400, help="Number of valid frames to capture for calibration -> Larger is better")
    parser.add_argument("--board_width", type=int, default=9, help="Number of inner corners -1 per chessboard row")
    parser.add_argument("--board_height", type=int, default=6, help="Number of inner corners -1 per chessboard column")
    parser.add_argument("--square_size", type=float, default=0.0254, help="Size of a square in meters")
    parser.add_argument("--camera_id", type=int, default=0, help="ID of the camera: NEED TO BE SET TO THE CAMERA ID YOU WANT TO CALIBRATE")
    return parser.parse_args()

def main():
    args = parse_args()

    # prepare object points based on the chessboard dimensions
    objp = np.zeros((args.board_height * args.board_width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.board_width, 0:args.board_height].T.reshape(-1, 2)
    objp *= args.square_size

    objpoints = []  #3d points in real-world space
    imgpoints = []  #2d points in image plane

    cap = cv2.VideoCapture(args.camera_id)
    if not cap.isOpened():
        print(f"Error: Cannot open camera {args.camera_id}")
        return

    valid_frames = 0
    print("Starting capture. Press 'q' to quit early.")

    while valid_frames < args.frames:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        pattern_size = (args.board_width, args.board_height)
        found, corners = cv2.findChessboardCorners(gray, pattern_size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

        display = frame.copy()
        if found:
            corners_sub = cv2.cornerSubPix( gray, corners, (11, 11), (-1, -1),criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            objpoints.append(objp)
            imgpoints.append(corners_sub)
            valid_frames += 1
            cv2.drawChessboardCorners(display, pattern_size, corners_sub, found)
            cv2.putText(display, f"Captured {valid_frames}/{args.frames}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            cv2.putText(display, "Chessboard not found", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow('Calibration', display)
        key = cv2.waitKey(500) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    if len(objpoints) < 1:
        print("Not enough valid frames for calibration.")
        return

    # Perform camera calibration
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    if not ret:
        print("Calibration failed.")
        return
    
    # Compute and print mean reprojection error
    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(
            objpoints[i], rvecs[i], tvecs[i], mtx, dist
        )
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += error

    mean_error = total_error / len(objpoints)

    # Print intrinsics and distortion coefficients
    print(f"Intrinsics: {mtx.tolist()}")
    print(f"Distortion coefficients: {dist.ravel().tolist()}")
    print(f"Mean reprojection error: {mean_error:.4f} pixels")

if __name__ == "__main__":
    main()
