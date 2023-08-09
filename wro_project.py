import RPi.GPIO as GPIO
import time
import YB_Pcb_Car  
import cv2
import numpy as np  

car = YB_Pcb_Car.YB_Pcb_Car()
car.Ctrl_Servo(1,0)
time.sleep(1)
car.Ctrl_Servo(2,90)
time.sleep(1)


GPIO.setmode(GPIO.BOARD)

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
  
    distance = (ultrasonic[1] + ultrasonic[2] + ultrasonic[3])/3
    return distance
def avoid():
    distance = Distance_test()
    LeftSensorValue  = GPIO.input(AvoidSensorLeft);
    RightSensorValue = GPIO.input(AvoidSensorRight);
    if distance < 15 and LeftSensorValue == False and RightSensorValue == False :
        car.Car_Stop() 
        time.sleep(0.1)
        car.Car_Spin_Right(100,100) 
        time.sleep(1)
    elif distance < 15 and LeftSensorValue == True and RightSensorValue == False :
        car.Car_Stop()
        time.sleep(0.1)
        car.Car_Spin_Left(80,80) 
        time.sleep(1)
        if LeftSensorValue == False and RightSensorValue == True :
            car.Car_Stop()
            time.sleep(0.1)
            car.Car_Spin_Right(90,90) 
            time.sleep(2)
    elif distance < 15 and LeftSensorValue == False and RightSensorValue == True :
        car.Car_Stop() 
        time.sleep(0.1)
        car.Car_Spin_Right(80,80)
        time.sleep(1)
        if LeftSensorValue == True and RightSensorValue == False  :
            car.Car_Stop()
            time.sleep(0.1)
            car.Car_Spin_Left(90,90) 
            time.sleep(2)
    elif distance < 15 and LeftSensorValue == True and RightSensorValue == True :
        car.Car_Stop() 
        time.sleep(0.1)
        car.Car_Spin_Right(80,80) 
        time.sleep(0.5)
    elif distance >= 15 and LeftSensorValue == False and RightSensorValue == False :
        car.Car_Stop() 
        time.sleep(0.1)
        car.Car_Spin_Right(90,90) 
        time.sleep(1)
    elif distance >= 15 and LeftSensorValue == False and RightSensorValue == True :
        car.Car_Stop() 
        time.sleep(0.1)
        car.Car_Spin_Right(80,80) 
        time.sleep(0.5)
    elif distance >= 15 and LeftSensorValue == True and RightSensorValue == False :
        car.Car_Stop() 
        time.sleep(0.1)
        car.Car_Spin_Left(80,80) 
        time.sleep(0.5)
    else:
        car.Car_Run(100,100) 

# stop car after 3 cycle 


def stop_car():
    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([130, 255, 255])

    cap = cv2.VideoCapture(0)

    blue_counter = 0

    while True:
        ret, frame = cap.read()

        if not ret:
            break

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask_blue = cv2.inRange(hsv_frame, lower_blue, upper_blue)

        contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)  

            
                blue_counter += 1
                print("Blue Line Counter:", blue_counter)

                if blue_counter >= 12:
                    car.Car_stop()

        cv2.imshow("Blue Line Detection", frame)
        
        if cv2.waitKey(1) & 0xFF == 27:  


        cap.release()
        cv2.destroyAllWindows()


try:
    while True:
        avoid()
        stop_car()
except KeyboardInterrupt:
    pass
car.Car_Stop() 
del car
print("Ending")
GPIO.cleanup()