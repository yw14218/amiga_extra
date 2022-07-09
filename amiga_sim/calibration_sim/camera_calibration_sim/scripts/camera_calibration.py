import sys

sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
import cv2

sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import sys
import numpy as np
import glob


def calibration_photo(photo_path, camera_name):
	size = 1.0 
	x_nums = 7 
	y_nums = 6 

	world_point = np.zeros((x_nums * y_nums, 3), np.float32) 
	world_point[:, :2] = np.mgrid[:x_nums, :y_nums].T.reshape(-1, 2) 
	world_point = world_point * size

	world_position = []
	image_position = []

	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
	images = glob.glob(photo_path) 

	for image_path in images:
		image = cv2.imread(image_path)
		gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

		ok, corners = cv2.findChessboardCorners(gray, (x_nums, y_nums), None) 

		if ok:
			world_position.append(world_point)  
			exact_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)  
			image_position.append(exact_corners) 

			# image = cv2.drawChessboardCorners(image,(x_nums,y_nums),exact_corners,ok)
			# cv2.imshow('image_corner',image)
			# cv2.waitKey(5000)

	ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(world_position, image_position, (1920, 1080), None, None)  

	np.savez("./" +camera_name + "intrinsic_parameters.npz", mtx=mtx, dist=dist) 

	print("*********************** intrinsic parameters of ", camera_name, "***********************")
	np.set_printoptions(suppress=True)
	print("mtx:\n", mtx, "\n")
	print("dist:\n", dist, "\n")

	mean_error = 0
	for i in range(len(world_position)):
		image_position2, _ = cv2.projectPoints(world_position[i], rvecs[i], tvecs[i], mtx, dist)
		error = cv2.norm(image_position[i], image_position2, cv2.NORM_L2) / len(image_position2)
		mean_error += error
	print("total error: ", mean_error / len(image_position))


RGB_photo_path = "../save_checkboard_img/RGB/*"  
IR_photo_path = "../save_checkboard_img/IR/*" 
if __name__ == "__main__":
	calibration_photo(RGB_photo_path, "RGB_camera")
	calibration_photo(IR_photo_path, "IR_camera")
