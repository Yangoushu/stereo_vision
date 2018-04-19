# Stereo Vision
Distance measurement using Stereo Vision. This project mesaures the distance of an object using opencv stereo vision and python.


Depth Map using Stereo Camera:
 <p align="center">
  <img src="1.Point Cloud Generation/stereo_rig.jpg" width=600 height=400>
 </p>
 
## 1. Installations and Setup:
Stereo Camera Rig:
 <p align="center">
  <img src="1.Point Cloud Generation/stereo_rig.jpg" width=600 height=400>
 </p>

 ``` bash
 # Add conda to python2 path:
 export PATH=~/anaconda2/bin:$PATH
 
 # Download opencv package (2.4.13):
 conda install -c dhaneshr opencv  
 
 # Install other dependencies:
 conda install progressbar
 conda install simplejson
 
 sudo add-apt-repository ppa:zarquon42/meshlab
 sudo apt-get update
 sudo apt-get install meshlab

 # Test Webcams:
 python show_webcams.py -h
 python show_webcams.py 0 1
 
 # Take images for calibration:
 python capture_chessboards -h
 python capture_chessboards.py --rows 6 --columns 9 --square-size 2.5 --calibration-folder calibration_files 0 1 50 calibration_images
 
 # Calibrate cameras using calibration_images:
 python calibrate_cameras.py -h
 python calibrate_cameras.py --rows 6 --columns 9 --square-size 2.5 calibration_images calibration_files
 # Error Note: If we do not hold the chessboard as 6x9 style(horizontally) it will not be able to calibrate the vertically captured image

# Generate Stereo Images for Point Cloud:
python stereo_image_generator.py --output_folder stereo_images --interval 5 0 1
python stereo_image_generator.py -h

# Tune Block Matching Algorithm:
python tune_blockmatcher.py -h

# Use StereoBM rather than StereoSGBM block matcher:
python tune_blockmatcher.py --use_stereobm --bm_settings settings.json calibration_files stereo_images
# (Note: This do not give satisfactory results as it uses StereoBM rather than StereoSGBM block matcher

# Lets use StereoSGBM block matcher and generate settings.json for block matcher:
python tune_blockmatcher.py --bm_settings settings.json calibration_files stereo_images

# Generate Point Cloud using stereo image and Block Matcher Settings:
python images_to_pointcloud.py -h
python images_to_pointcloud.py --bm_settings settings.json calibration_files left_1.ppm right_1.ppm output.ply
```
Python Version = 2.7.14

## 2. Installations and Setup:
```bash
sudo pip3 install openpyxl

# Install opencv3 (current version is 3.4.0)
sudo pip3 install opencv-python

sudo pip3 install sklearn
```

Distance between two camera lenses = 110 mm

Python Version = 3.5.2

Opencv Version = 3.4.0

Youtube Video Link: https://www.youtube.com/watch?v=2GG_30n2X8E&t=69s

