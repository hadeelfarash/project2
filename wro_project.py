from gpiozero import DistanceSensor
import time

# Define GPIO pins for ultrasonic sensors
forward_sensor = DistanceSensor(echo=18, trigger=17)  # Forward sensor
right_sensor = DistanceSensor(echo=23, trigger=22)   # Right sensor
left_sensor = DistanceSensor(echo=27, trigger=24)    # Left sensor

# Function to check obstacle avoidance
def avoid_obstacle():
    while True:
        forward_distance = forward_sensor.distance * 100  # Convert to centimeters
        right_distance = right_sensor.distance * 100
        left_distance = left_sensor.distance * 100

        print(f"Forward Distance: {forward_distance:.2f} cm")
        print(f"Right Distance: {right_distance:.2f} cm")
        print(f"Left Distance: {left_distance:.2f} cm")

        if forward_distance < 20:
            print("Obstacle detected in front. Turning left.")
            # Add code here to turn left (e.g., control motors or wheels)
        elif right_distance < 20:
            print("Obstacle detected on the right. Turning left.")
            # Add code here to turn left
        elif left_distance < 20:
            print("Obstacle detected on the left. Turning right.")
            # Add code here to turn right
        else:
            print("No obstacles detected. Moving forward.")
            # Add code here to move forward

        time.sleep(0.1)  # Adjust the delay as needed

try:
    avoid_obstacle()
except KeyboardInterrupt:
    print("Program terminated by user.")
finally:
    forward_sensor.close()
    right_sensor.close()
    left_sensor.close()
