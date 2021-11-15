#Camera Calibration Code
#From ComputerVision CSCI437 CameraClibration Lecture
##########################################################
import numpy as np
import cv2
import glob
from picamera.array import PiRGBArray
from picamera import PiCamera
import picamera
import sys
import os

QUERY_IMAGE_DIRECTORY = "chessboard_images"

def cameraCalibration(fname):
    # This program reads images of a chessboard, finds corners, and calibrates the camera.
    # The chessboard is assumed to be 7 rows and 8 columns of squares.
    # Create points in target coordinates; i.e., (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0).
    # These are the inner corners of the squares. Note that it doesn't find the outer corners, so the actual
    # grid is 6 x 7 corners. Units don't matter because we are not interested in the absolute camera poses.
    target_pts = np.zeros((6 * 9, 3), np.float32)
    target_pts[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)
    # Arrays to store object points and image points from all the images.
    objpoints = []  # Collect all 3d points in target coordinates
    imgpoints = []  # Collect all 2d points in image plane
    
    assert (os.path.exists(QUERY_IMAGE_DIRECTORY))
    image_file_names = glob.glob(os.path.join(QUERY_IMAGE_DIRECTORY, "*.png"))
    assert (len(image_file_names) > 0)
    
    for fname in image_file_names:
        img = cv2.imread(fname)
        gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray_image.shape
        # Find the chess board corners
        ret_val, corners = cv2.findChessboardCorners(gray_image, (9, 6), None)
        #print("ret_val: ", ret_val, "\n")
        # If found, add object and image points.
        if ret_val == True:
            # Optionally refine corner locations.
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray_image, corners, (11, 11), (-1, -1), 
                                        criteria)
            # Collect the object and image points.
            objpoints.append(target_pts)
            imgpoints.append(corners2)
            # Draw and display the corners
            cv2.drawChessboardCorners(img, (9,6), corners2, ret_val)
            cv2.imshow('img', img)
            cv2.waitKey(60)
    cv2.destroyAllWindows()
    # Do the calibration.
    ret_val, K, dist, rvecs, tvecs = cv2.calibrateCamera(objectPoints=objpoints, imagePoints=imgpoints, imageSize=(w, h),cameraMatrix=None, distCoeffs=None)
    print("Camera matrix:")
    print(repr(K))
    print("Distortion coeffs:")
    print(repr(dist))
    print("rvecs:")
    print(repr(rvecs))
    print("tvecs: ")
    print(repr(tvecs))
    
    # Calculate re-projection error - should be close to zero.
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    print( "total error: {}".format(mean_error/len(objpoints)) )
    # Optionally undistort and display the images.
    for fname in images:
        img = cv2.imread(fname)
        cv2.imshow("distorted", img)
        undistorted_img = cv2.undistort(src=img, cameraMatrix=K, distCoeffs=dist)
        cv2.imshow("undistorted", undistorted_img)
        cv2.imwrite("undistorted_" + fname, undistorted_img)
        if cv2.waitKey(0) == 27:  # ESC is ascii code 27
            break
        
def get_chessboard_images(fname):
    # Initialize image capture from camera.
    video_capture = cv2.VideoCapture(fname)  # Open video capture object
    is_ok, bgr_image_input = video_capture.read()  # Make sure we can read video
    if not is_ok:
        print("Cannot read video source")
        sys.exit()
    print("Hit ESC to quit; any other key to capture an image ...")
    imageNum = 0
    while True:
        is_ok, bgr_image_input = video_capture.read()
        if not is_ok:
            break  # no camera, or reached end of video file
        gray_image = cv2.cvtColor(src=bgr_image_input, code=cv2.COLOR_BGR2GRAY)
        # Find the chess board corners.
        ret_val, corners = cv2.findChessboardCorners(image=gray_image, 
                                                    patternSize=(7,6))
        # Draw corners on a copy of the image.
        bgr_display = bgr_image_input.copy()
        cv2.drawChessboardCorners(image=bgr_display, patternSize=(7,6), 
                                    corners=corners, patternWasFound=ret_val)
        # Wait for xx msec (0 = wait till keypress).
        cv2.imshow('img', bgr_display)
        key_pressed = cv2.waitKey(30)
        if key_pressed == -1:
            continue
        elif key_pressed == 27:
            break
        else:
            fname = "img%02d.png" % imageNum
            cv2.imwrite(filename=fname, img=bgr_image_input)
            print("Saving image %s" % fname)
            imageNum += 1
    video_capture.release()
    cv2.destroyAllWindows()
        
def main():
    #with picamera.PiCamera() as camera:
        #camera.resolution = (640, 480)
        #camera.start_recording('chessboard.h264')
        #camera.wait_recording(10)
        #camera.stop_recording()
    
    #get_chessboard_images('chessboard.h264')
    
    cameraCalibration('chessboard_images_2')
    

if __name__ == "__main__":
    main()