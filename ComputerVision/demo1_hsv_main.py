#Nash Neff - Computer Vision
#SEED Lab
######################################
import cv2
import numpy as np
import math as m
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import smbus
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import board
###############################################
bus = smbus.SMBus(1)
address = 0x04
lcd_columns = 16
lcd_rows = 2
i2c = board.I2C()
try:
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
    lcd.color = [50,50,50]
except:
    print("no device connected \n")

#Camera Parameters
K = np.array([[427, 0, 332],
              [0, 412, 272],
              [0, 0, 1]])
fx, fy = 427, 412
xx, yy = 332, 272
DIST_COEFFS = np.array([[-.010, .045, -.007, .003, -.163]])

#DEAD_SPACE = 6

###############7-12##12-18#18-24#24-30#30-36
INCH_PER_PX = [.178, .142, .146, .165, .181]
DISTANCE_OFFSET = 4

data123 = [1, 2, 3]

def find_target_hsv():
    #end_case = 0
    while True:
        camera = PiCamera()
        rawCapture = PiRGBArray(camera)
    
        time.sleep(.1)
        try:
            camera.resolution = (640,480)
            camera.capture(rawCapture, format='rgb')
            img = rawCapture.array
            #implement to put colors back normal __> change thresholds
            img = img[:,:,::-1]
            camera.close()
        except:
            print("Failed to capture image \n")
    
        height, width = img.shape[0], img.shape[1]
    
        #hsv threshold blue
        low_thresh = (80,0,0)
        high_thresh = (120,200,100)
    
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
        planes = cv2.split(hsv_img)
    
        thresh_img = np.full((height, width), 255, dtype=np.uint8)
        windowNames = ["hue", "sat", "intensity"]
    
        for i in range(3):
            _, low_img = cv2.threshold(planes[i], low_thresh[i], 255, cv2.THRESH_BINARY)
            _, high_img = cv2.threshold(planes[i], high_thresh[i], 255, cv2.THRESH_BINARY_INV)
                    
            thresh_band_img = cv2.bitwise_and(low_img, high_img)
            thresh_img = cv2.bitwise_and(thresh_img, thresh_band_img)
            #cv2.imshow("test", thresh_img)
        
        kernel = np.ones((5,5), np.uint8)
        filtered_img = cv2.morphologyEx(thresh_img, cv2.MORPH_OPEN, kernel)
        filtered_img = cv2.morphologyEx(thresh_img, cv2.MORPH_OPEN, kernel)
        filtered_img = cv2.morphologyEx(thresh_img, cv2.MORPH_CLOSE, kernel)
    
        num_labels, labels_img, stats, centroids = cv2.connectedComponentsWithStats(filtered_img)
        
        marker_img = cv2.cvtColor(filtered_img, cv2.COLOR_GRAY2BGR)
        #cv2.imshow("thresh", marker_img)
    
        marker_list = []
        markerFound = False
        for stat, centroid in zip(stats, centroids):
            area = stat[cv2.CC_STAT_AREA]
            x = stat[cv2.CC_STAT_LEFT]
            y = stat[cv2.CC_STAT_TOP]
            w = stat[cv2.CC_STAT_WIDTH]
            h = stat[cv2.CC_STAT_HEIGHT]
            
            Cx = x+int(w/2)
            Cy = y+int(h/2)
        
            #area threshold to filter out noise
            if area >= 50 and area <= 10000:
                if Cy > 150:
                    #find the middle of the line
                    Cx = x+int(w/2)
                    Cy = y+int(h/2)
                    print("Cx, Cy = ", Cx," ", Cy, "\n")
                    marker_list.append((Cx, Cy))
                    markerFound = True
            
                    #normalize object coordinates
                    norm_point = np.linalg.inv(K) @ np.array([Cx, Cy, 1])
                    unit_vec = norm_point
                    
                    norm_point[0] = (norm_point[0] * xx)
                    norm_point[1] = (norm_point[1] * yy)
                    
                    end_of_vector = np.array([width/2 + norm_point[0], width/2 + norm_point[1]])
                    
            
                    output_img = cv2.drawMarker(gray_img, (Cx,Cy), (0,0,255), cv2.MARKER_CROSS, 5)
                    cv2.putText(output_img, str(len(marker_list)), org=(x,y-15), color=(0,0,255), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.5, thickness=2)
                    cv2.line(gray_img, (int(width/2), int(height)), (Cx, Cy), (255, 255, 255))
                    cv2.line(gray_img, (int(width/2), int(height)), (int(end_of_vector[0]), int(end_of_vector[1])), (255, 255, 255))
                    cv2.line(gray_img, (int(width/2), int(height)), (int(width/2), 0), (255, 255, 255))
                    #print("area of marker ", len(marker_list), "is ", area, "\n")
                    
                    #convert angle to 0-255 to be sent in one byte
                    phi = m.atan(((width/2)- end_of_vector[0]) / (height - end_of_vector[1]))/2
                    angleData = (phi*180/m.pi)
                    print("angle: ", angleData, "\n")
                    
                    #if Cy >= 469 and end_case == 0:
                        #distance = DEAD_SPACE
                        #end_case += 1
                    if Cy >= 469: # and end_case > 0:
                        distance = 0
                    if Cy < 469 and Cy >= 372:
                        distance = int((height - end_of_vector[1])*INCH_PER_PX[0])# - DEAD_SPACE
                    if Cy < 372 and Cy >= 294:
                        distance = int((height - end_of_vector[1])*INCH_PER_PX[1])
                    if Cy < 294 and Cy >= 251:
                        distance = int((height - end_of_vector[1])*INCH_PER_PX[2]) 
                    if Cy < 251 and Cy >= 224:
                        #distance = int((height - end_of_vector[1])*INCH_PER_PX[3])
                        distance = 24
                    if Cy < 224 and Cy >= 205:
                        #distance = int((height - end_of_vector[1])*INCH_PER_PX[4])
                        distance = 24
                    if Cy < 205:
                        distance = 24
                    print("distance: ",distance, "\n")
                    
        if markerFound is False:
            lcd.message = "no marker"
            print("nothing to do \n")
        if markerFound is True:
            angleData = int(angleData)
            try:
                writedata = [int((angleData+90)*255/180), int(distance*255/24)]

                writeBlock(address, writedata)
                #bus.write_byte_data(address,0,69)
            except:
                print("error sending data \n")
            lcd.message = "phi is %s"%(angleData)
            
        #time.sleep(.25)
        lcd.clear()
        
        #cv2.imshow("test", img)
        cv2.imshow("gray", gray_img)
        cv2.waitKey(15)
        
def writeBlock(address,data):
    bus.write_i2c_block_data(address, 0, data)
    return

def main():
    while True:
        try:
            find_target_hsv()
        except:
            print("i2c line error \n")
            
if __name__ == "__main__":
    main()