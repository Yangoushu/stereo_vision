import cv2          # Import open cv
video_file_path = 'big_buck_bunny.mp4'
cap = cv2.VideoCapture(0)
import time
# cap = cv2.VideoCapture(1)   # To read WebCam usb
ret, frame = cap.read()
count  = 0

while ret:
    ret, frame = cap.read()
    print count
    if count % 200 == 0:
    	cv2.imshow('Colored Video', frame)
    count += 1
    if cv2.waitKey(25) & 0xFF == ord('q'):   # If q is pressed; quit the window
        break
	
cap.release()
# out.release()   # Saves the video (works in windows OS)
cv2.destroyAllWindows()


