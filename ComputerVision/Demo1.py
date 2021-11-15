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
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.color = [50,50,50]

def writeByte(value):
    bus.write_byte(address, value)
    print("writeByte done \n")
    if value == 0:
        lcd.message = "no marker"
    elif value > 0:
        lcd.message = "marker found"
    return

def find_target_hsv():
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
        low_thresh = (100,0,0)
        high_thresh = (120,255,255)
    
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
        planes = cv2.split(hsv_img)
    
        thresh_img = np.full((height, width), 255, dtype=np.uint8)
        windowNames = ["hue", "sat", "intensity"]
    
        for i in range(3):
            _, low_img = cv2.threshold(planes[i], low_thresh[i], 255, cv2.THRESH_BINARY)
            _, high_img = cv2.threshold(planes[i], high_thresh[i], 255, cv2.THRESH_BINARY_INV)
                    
            thresh_band_img = cv2.bitwise_and(low_img, high_img)
            thresh_img = cv2.bitwise_and(thresh_img, thresh_band_img)
        
        kernel = np.ones((5,5), np.uint8)
        filtered_img = cv2.morphologyEx(thresh_img, cv2.MORPH_OPEN, kernel)
        filtered_img = cv2.morphologyEx(thresh_img, cv2.MORPH_CLOSE, kernel)
    
        num_labels, labels_img, stats, centroids = cv2.connectedComponentsWithStats(filtered_img)
        
        marker_img = cv2.cvtColor(filtered_img, cv2.COLOR_GRAY2BGR)
    
        marker_list = []
        markerFound = False
        for stat, centroid in zip(stats, centroids):
            area = stat[cv2.CC_STAT_AREA]
            x = stat[cv2.CC_STAT_LEFT]
            y = stat[cv2.CC_STAT_TOP]
            w = stat[cv2.CC_STAT_WIDTH]
            h = stat[cv2.CC_STAT_HEIGHT]
        
            #area threshold to filter out noise
            if area >= 150 and area <= 10000:
                #find the middle of the line
                Cx = x+int(w/2)
                Cy = y+int(h/2)
                marker_list.append((Cx, Cy))
                markerFound = True
            
                #calculating distance
                distance_x = Cx - width/2
                distance_y = height - (height - Cy)
            
                #calculate the angle of the line from center
                phi = m.atan(distance_x/distance_y)
                print("phi: ", phi,"\n")
            
                #convert angle to 0-255 to be sent in one byte
                angleData = (phi*255)/m.pi
                print("angleData", angleData, "\n")
            
                output_img = cv2.drawMarker(marker_img, (Cx,Cy), (0,0,255), cv2.MARKER_CROSS, 5)
                cv2.putText(output_img, str(len(marker_list)), org=(x,y-15), color=(0,0,255), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.5, thickness=2)
                print("area of marker ", len(marker_list), "is ", area, "\n")
        
        if markerFound is False:
            lcd.message = "no marker"
        if markerFound is True:
            angleData = int(angleData)
            lcd.message = "phi is %s"%(angleData)
        
        #cv2.imshow("test", marker_img)
        #cv2.waitKey(30)

def main():
    find_target_hsv()

if __name__ == "__main__":
    main()