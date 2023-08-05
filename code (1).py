import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import YB_Pcb_Car   

car = YB_Pcb_Car.YB_Pcb_Car()

#Set the GPIO port to BIARD encoding mode
GPIO.setmode(GPIO.BOARD)

#Ignore the warning message
GPIO.setwarnings(False)

#for color
def avoid_cube():

    lower_green = np.array([40, 40, 40])
    upper_green = np.array([80, 255, 255])

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    cap = cv2.VideoCapture(0)

    while True:
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

        # print("Green percentage:", green_percentage)
        # print("Red percentage:", red_percentage)

        cv2.imshow("Original", frame)
        cv2.imshow("Green Mask", mask_green)
        cv2.imshow("Red Mask", mask_red)
        cv2.imshow("Combined Mask", mask_combined)

        car.Car_Run(50,50)


        if green_percentage > 1.0:
        
            car.Car_Stop()
            time.sleep(0.1)
            car.Car_Spin_Left(90,90) 
            time.sleep(2)
            car.Car_Run(100,100) 

        elif red_percentage > 1.0:
            car.Car_Stop()
            time.sleep(0.1)
            car.Car_Spin_Right(90,90) 
            time.sleep(2)
            car.Car_Run(100,100) 
        else:
            car.Car_Run(50,50)


    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

    cap.release()
    cv2.destroyAllWindows()



#for avoid
AvoidSensorLeft = 21     #Left infrared obstacle avoidance sensor pin
AvoidSensorRight = 19    #Right infrared obstacle avoidance sensor pin
Avoid_ON = 22   #Infrared obstacle avoidance sensor switch pin

#Define the pins of the ultrasonic module
EchoPin = 18
TrigPin = 16

#Set pin mode
GPIO.setup(AvoidSensorLeft,GPIO.IN)
GPIO.setup(AvoidSensorRight,GPIO.IN)
GPIO.setup(Avoid_ON,GPIO.OUT)
GPIO.setup(EchoPin,GPIO.IN)
GPIO.setup(TrigPin,GPIO.OUT)
GPIO.output(Avoid_ON,GPIO.HIGH)

#Ultrasonic function
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
    #time.sleep(0.01)
    print ("distance_1 is %d " % (((t2 - t1)* 340 / 2) * 100))
    distance=((t2 - t1)* 340 / 2) * 100
    return distance



def avoid():
    distance =  Distance()
    LeftSensorValue  = GPIO.input(AvoidSensorLeft)
    RightSensorValue = GPIO.input(AvoidSensorRight)
    #With obstacle pin is low level, the indicator light is on, without obstacle, pin is high level, the indicator light is off
    if distance < 15 :
        car.Car_Stop()
        time.sleep(0.1)
        if  LeftSensorValue == False and RightSensorValue == False :
            car.Car_Spin_Right(100,100) 
            time.sleep(1)
        elif LeftSensorValue == True and RightSensorValue == False :
            car.Car_Spin_Left(80,80) 
            time.sleep(1)
            if GPIO.input(AvoidSensorLeft)==False and GPIO.input(AvoidSensorRight)==True:
                car.Car_Stop()
                time.sleep(0.1)
                car.Car_Spin_Right(90,90) 
                time.sleep(2)
            
        elif LeftSensorValue == False and RightSensorValue == True :
            car.Car_Spin_Right(80,80)
            time.sleep(1)
            if GPIO.input(AvoidSensorLeft)==True and GPIO.input(AvoidSensorRight)==False:
                car.Car_Stop()
                time.sleep(0.1)
                car.Car_Spin_Left(90,90) 
                time.sleep(2)
        elif  LeftSensorValue == True and RightSensorValue == True :
           
            car.Car_Spin_Right(80,80) 
            time.sleep(0.5)
        
        

try:
    while True:
        car.Car_Run(100,100) 
        avoid()
        avoid_cube()
except KeyboardInterrupt:
    pass
car.Car_Stop() 
del car
print("Ending")
GPIO.cleanup()

