import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import YB_Pcb_Car   

car = YB_Pcb_Car.YB_Pcb_Car()
car.Ctrl_Servo(1,0)
time.sleep(1)
car.Ctrl_Servo(2,90)
time.sleep(1)

#Set the GPIO port to BIARD encoding mode
GPIO.setmode(GPIO.BOARD)

#Ignore the warning message
GPIO.setwarnings(False)


AvoidSensorLeft = 21    
AvoidSensorRight = 19    
Avoid_ON = 22  


EchoPin = 18
TrigPin = 16


GPIO.setup(AvoidSensorLeft,GPIO.IN)
GPIO.setup(AvoidSensorRight,GPIO.IN)
GPIO.setup(Avoid_ON,GPIO.OUT)
GPIO.setup(EchoPin,GPIO.IN)
GPIO.setup(TrigPin,GPIO.OUT)
GPIO.output(Avoid_ON,GPIO.HIGH)

def Distance():
    GPIO.output(TrigPin,GPIO.LOW)
    time.sleep(0.000002)
    GPIO.output(TrigPin,GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TrigPin,GPIO.LOW)

    t3 = time.time()

    while not GPIO.input(EchoPin):
        t4 = time.time()
        if (t4 - t3) > 0.03 :
            return -1
    t1 = time.time()
    while GPIO.input(EchoPin):
        t5 = time.time()
        if(t5 - t1) > 0.03 :
            return -1

    t2 = time.time()
 
    return ((t2 - t1)* 340 / 2) * 100

def Distance_test():
    num = 0
    ultrasonic = []
    while num < 5:
            distance = Distance()
            while int(distance) == -1 :
                distance = Distance()
            while (int(distance) >= 500 or int(distance) == 0) :
                distance = Distance()
            ultrasonic.append(distance)
            num = num + 1

    distance = (ultrasonic[1]+ultrasonic[2]+ultrasonic[3])/3
    return distance
def avoid():
    distance = Distance_test()
    LeftSensorValue  = GPIO.input(AvoidSensorLeft)
    RightSensorValue = GPIO.input(AvoidSensorRight)
    if distance < 15 :
        car.Car_Stop()
        time.sleep(1)
        car.Car_Back(90,90)
        time.sleep(0.1)

        if distance < 15 and LeftSensorValue == False and RightSensorValue == False :
            car.Car_Stop() 
            time.sleep(0.1)
            car.Car_Left(0,100) 
            time.sleep(1)
        elif distance < 15 and LeftSensorValue == True and RightSensorValue == False :
            car.Car_Stop()
            time.sleep(0.1)
            car.Car_Right(80,0) 
            time.sleep(1)
            if LeftSensorValue == False and RightSensorValue == True :
                car.Car_Stop()
                time.sleep(0.1)
                car.Car_Left(0,90) 
                time.sleep(2)
        elif distance < 15 and LeftSensorValue == False and RightSensorValue == True :
            car.Car_Stop() 
            time.sleep(0.1)
            car.Car_Left(0,80)
            time.sleep(1)
            if LeftSensorValue == True and RightSensorValue == False  :
                car.Car_Stop()
                time.sleep(0.1)
                car.Car_Right(90,0) 
                time.sleep(2)
        elif distance < 15 and LeftSensorValue == True and RightSensorValue == True :
            car.Car_Stop() 
            time.sleep(0.1)
            car.Car_Right(80,0) 
            time.sleep(0.5)
 

    else:
        car.Car_Run(100,100)



def colored_cube():
# for color 

    lower_green = np.array([40, 40, 40])
    upper_green = np.array([80, 255, 255])

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])


    while True:
        cap = cv2.VideoCapture(0)

        ret, frame = cap.read()

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask_green = cv2.inRange(hsv_frame, lower_green, upper_green)
        mask_red1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)

        mask_red = mask_red1 + mask_red2
        mask_combined = mask_green + mask_red

        # Calculate the percentage of green and red pixels in the frame
        total_pixels = frame.shape[0] * frame.shape[1]
        green_pixels = cv2.countNonZero(mask_green)
        red_pixels = cv2.countNonZero(mask_red)

        green_percentage = (green_pixels / total_pixels) * 100
        red_percentage = (red_pixels / total_pixels) * 100

    

        cv2.imshow("Original", frame)
        cv2.imshow("Green Mask", mask_green)
        cv2.imshow("Red Mask", mask_red)
        cv2.imshow("Combined Mask", mask_combined)




        if green_percentage>1.0:
            car.Car_Stop()
            time.sleep(0.5)
            car.Car_Left(80,80) 
            time.sleep(1)
            car.Car_Run(50,50)


        elif red_percentage>1.0:
            car.Car_Stop()
            time.sleep(0.5)
            car.Car_Right(80,80) 
            time.sleep(1)
            car.Car_Run(50,50)



        cap.release()
        cv2.destroyAllWindows()


    


try:
    while True:
        avoid()
        colored_cube()
except KeyboardInterrupt:
    pass
car.Car_Stop() 
del car
print("Ending")
GPIO.cleanup()
