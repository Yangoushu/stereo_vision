import numpy as np
import cv2
# from openpyxl import Workbook  # Used for writing data into an Excel file
from sklearn.preprocessing import normalize

kernel = np.ones((3, 3), np.uint8)  # Filtering
black_pixel_count = 0
white_pixel_count = 0
white_pixels = 0


def coords_mouse_disp(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        print(x, y, disp[y, x], filteredImg[y, x])
        average = 0
        for u in range(-1, 2):
            for v in range(-1, 2):
                average += disp[y + u, x + v]
        average = average / 9
        distance = -593.97 * average ** 3 + 1506.8 * average ** 2 - 1373.1 * average + 522.06
        distance = np.around(distance * 0.01, decimals=2)
        print('Distance: ' + str(distance) + ' m')
        # ws.append([distance, average])  # Write the data in excel file
        print('Measure at ' + str(distance) + ' cm, the disparity is ' + str(average))
        if distance <= 85:
            distance += 3
        elif distance <= 120:
            distance += 5
        else:
            distance += 10
        print('Next distance to measure: ' + str(distance) + 'cm')


# Create instances to store the values in excel file
# wb = Workbook()
# ws = wb.active

# Parameters for Distortion calibration
# Termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
criteria_stereo = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points
objp = np.zeros((9 * 6, 3), np.float32)
objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

# Arrays to store object points and image points from all images
objpoints = []  # 3d points in real world space
imgpointsR = []  # 2d points in image plane
imgpointsL = []

# Start calibration from the camera
print('Starting calibration for the 2 cameras... ')
# Call all saved images
for i in range(1, 100):
    t = str(i)
    ChessImaR = cv2.imread('calibration_images2/left_' + t + '.ppm', 0)  # Right side
    ChessImaL = cv2.imread('calibration_images2/right_' + t + '.ppm', 0)  # Left side
    retR, cornersR = cv2.findChessboardCorners(ChessImaR, (9, 6), None)
    retL, cornersL = cv2.findChessboardCorners(ChessImaL, (9, 6), None)
    if (retR == True) and (retL == True):
        objpoints.append(objp)
        cv2.cornerSubPix(ChessImaR, cornersR, (11, 11), (-1, -1), criteria)
        cv2.cornerSubPix(ChessImaL, cornersL, (11, 11), (-1, -1), criteria)
        imgpointsR.append(cornersR)
        imgpointsL.append(cornersL)

# Determine the new values for different parameters
# Right Side
retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(objpoints, imgpointsR, ChessImaR.shape[::-1], None, None)
hR, wR = ChessImaR.shape[:2]
OmtxR, roiR = cv2.getOptimalNewCameraMatrix(mtxR, distR, (wR, hR), 1, (wR, hR))

#   Left Side
retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(objpoints, imgpointsL, ChessImaL.shape[::-1], None, None)
hL, wL = ChessImaL.shape[:2]
OmtxL, roiL = cv2.getOptimalNewCameraMatrix(mtxL, distL, (wL, hL), 1, (wL, hL))

print('Cameras are ready to use!')

# StereoCalibrate function
flags = 0
flags |= cv2.CALIB_FIX_INTRINSIC
# flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
# flags |= cv2.CALIB_USE_INTRINSIC_GUESS
# flags |= cv2.CALIB_FIX_FOCAL_LENGTH
# flags |= cv2.CALIB_FIX_ASPECT_RATIO
# flags |= cv2.CALIB_ZERO_TANGENT_DIST
# flags |= cv2.CALIB_RATIONAL_MODEL
# flags |= cv2.CALIB_SAME_FOCAL_LENGTH
# flags |= cv2.CALIB_FIX_K3
# flags |= cv2.CALIB_FIX_K4
# flags |= cv2.CALIB_FIX_K5
retS, MLS, dLS, MRS, dRS, R, T, E, F = cv2.stereoCalibrate(objpoints, imgpointsL, imgpointsR, mtxL, distL, mtxR, distR,
                                                           ChessImaR.shape[::-1], criteria_stereo, flags)

# StereoRectify function
rectify_scale = 0  # if 0 image cropped, if 1 image not cropped
RL, RR, PL, PR, Q, roiL, roiR = cv2.stereoRectify(MLS, dLS, MRS, dRS, ChessImaR.shape[::-1], R, T, rectify_scale,
                                                  (0, 0))
# initUndistortRectifyMap function gives pictures that has no distortion and can be used for calculating distortion
Left_Stereo_Map = cv2.initUndistortRectifyMap(MLS, dLS, RL, PL, ChessImaR.shape[::-1], cv2.CV_16SC2)
Right_Stereo_Map = cv2.initUndistortRectifyMap(MRS, dRS, RR, PR, ChessImaR.shape[::-1], cv2.CV_16SC2)

# Create StereoSGBM and prepare all parameters
window_size = 3
min_disp = 2
num_disp = 130 - min_disp
stereo = cv2.StereoSGBM_create(minDisparity=min_disp, numDisparities=num_disp, blockSize=window_size,
                               uniquenessRatio=10, speckleWindowSize=100, speckleRange=32, disp12MaxDiff=5,
                               P1=8 * 3 * window_size ** 2, P2=32 * 3 * window_size ** 2)

# Used for the filtered image
stereoR = cv2.ximgproc.createRightMatcher(stereo)  # Create another stereo for right this time

# WLS FILTER Parameters
lmbda = 80000
sigma = 1.8
visual_multiplier = 1.0

wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=stereo)
wls_filter.setLambda(lmbda)
wls_filter.setSigmaColor(sigma)

CamR = cv2.VideoCapture(0)
CamL = cv2.VideoCapture(1)

CamR.set(3, 320)
CamR.set(4, 240)

CamL.set(3, 320)
CamL.set(4, 240)

while True:
    # Start Reading Camera images
    retR, frameR = CamR.read()
    retL, frameL = CamL.read()

    # Rectify the images on rotation and alignment
    Left_nice = cv2.remap(frameL, Left_Stereo_Map[0], Left_Stereo_Map[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
    Right_nice = cv2.remap(frameR, Right_Stereo_Map[0], Right_Stereo_Map[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
    # Rectify the image using the calibration parameters founds during the initialisation

    # ##    # Draw Red lines
    #     for line in range(0, int(Right_nice.shape[0]/20)): # Draw the Lines on the images Then numer of line is defines by the image Size/20
    #        Left_nice[line*20,:]= (0,0,255)
    #        Right_nice[line*20,:]= (0,0,255)
    #
    #     for line in range(0, int(frameR.shape[0]/20)): # Draw the Lines on the images Then numer of line is defines by the image Size/20
    #        frameL[line*20,:]= (0,255,0)
    #        frameR[line*20,:]= (0,255,0)

    # Show the Undistorted imagesq
    cv2.imshow('Both Images', np.hstack([Left_nice, Right_nice]))
    cv2.imshow('Normal', np.hstack([frameL, frameR]))

    # Convert from color(BGR) to gray
    grayR = cv2.cvtColor(Right_nice, cv2.COLOR_BGR2GRAY)
    grayL = cv2.cvtColor(Left_nice, cv2.COLOR_BGR2GRAY)

    # Compute the 2 images for the Depth_image
    disp = stereo.compute(grayL, grayR)  # as type(np.float32)/ 16
    dispL = disp
    dispR = stereoR.compute(grayR, grayL)
    dispL = np.int16(dispL)
    dispR = np.int16(dispR)

    # Using the WLS filter
    filteredImg = wls_filter.filter(dispL, grayL, None, dispR)
    filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)
    filteredImg = np.uint8(filteredImg)
    cv2.imshow('Disparity Map', filteredImg)
    disp = ((disp.astype(np.float32) / 16) - min_disp) / num_disp

    # Resize the image for faster executions
    dispR = cv2.resize(disp, None, fx=0.7, fy=0.7, interpolation=cv2.INTER_AREA)

    # Filtering the Results with a closing filter
    # Apply an morphological filter for closing little "black" holes in the picture(Remove noise)
    closing = cv2.morphologyEx(disp, cv2.MORPH_CLOSE, kernel)

    # Colors map
    dispc = (closing - closing.min()) * 255
    # Convert the type of the matrix from float32 to uint8, to show the results with the function cv2.imshow()
    dispC = dispc.astype(np.uint8)
    disp_Color = cv2.applyColorMap(dispC, cv2.COLORMAP_OCEAN)  # Change the Color of the Picture into an Ocean Color_Map
    filt_Color = cv2.applyColorMap(filteredImg, cv2.COLORMAP_OCEAN)
    jet_color = cv2.applyColorMap(filteredImg, cv2.COLORMAP_JET)
    bone_color = cv2.applyColorMap(filteredImg, cv2.COLORMAP_BONE)

    # Show the result for the Depth_image
    # cv2.imshow('Disparity', disp)
    # cv2.imshow('Closing', closing)
    # cv2.imshow('Color Depth', disp_Color)
    # cv2.imshow('Filtered Color Depth', filt_Color)
    cv2.imshow('jet color', jet_color)
    cv2.imshow('bone color', bone_color)

    # Mouse click on filtered color depth image to get the distance of that clicked object
    # cv2.setMouseCallback("Filtered Color Depth", coords_mouse_disp, filt_Color)
    img = cv2.cvtColor(bone_color, cv2.COLOR_BGR2GRAY)
    cv2.imshow('grayscaled image', img)
    ##########################################################################################################
    # My Logic:
    for y in range(1, 240):
        for x in range(1, 320):
            px = img[y, x]  # access the particular pixel
            white_pixels = white_pixels + px

    # print('black pixels count =', black_pixel_count)
    print('white pixels =', white_pixels)
    print('Total pixels = ', 320*240*150)
    percent = (white_pixels/(320*240*150)) * 100
    print('covered space in % =', percent)
    white_pixels = 0
    ##########################################################################################################

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# wb.save("output_data.xlsx")  # Save excel file
CamR.release()
CamL.release()
cv2.destroyAllWindows()
