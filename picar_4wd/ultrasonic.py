import time
from picar_4wd.servo import Servo
from picar_4wd.pwm import PWM
from picar_4wd.pin import Pin


class Ultrasonic():
    ANGLE_RANGE = 180
    STEP = 18

    def __init__(self, trig, echo, timeout=0.01):
        self.timeout = timeout
        self.trig = trig
        self.echo = echo
        self.servo = Servo(PWM("P0"), offset=10)
        self.angle_distance = [0, 0]
        self.current_angle = 0
        self.max_angle = self.ANGLE_RANGE / 2
        self.min_angle = -self.ANGLE_RANGE / 2
        self.scan_list = []

    def get_distance(self):
        self.trig.low()
        time.sleep(0.01)
        self.trig.high()
        time.sleep(0.000015)
        self.trig.low()
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()
        while self.echo.value() == 0:
            pulse_start = time.time()
            if pulse_start - timeout_start > self.timeout:
                return -1
        while self.echo.value() == 1:
            pulse_end = time.time()
            if pulse_end - timeout_start > self.timeout:
                return -2
        during = pulse_end - pulse_start
        cm = round(during * 340 / 2 * 100, 2)
        return cm


# Initialize Ultrasonic Sensor
trig_pin = Pin(pin='D0')
echo_pin = Pin(pin='D1')
ultrasonic = Ultrasonic(trig=trig_pin, echo=echo_pin)

# Initialize Servo for Steering
steering_servo = Servo(PWM("P0"), offset=10)

# Define a function to stop and change direction


def avoid_obstacle():
    print("Obstacle detected! Stopping and changing direction.")

    # Stop the car by setting motor speed to 0
    from picar_4wd import forward, backward, stop
    stop()

    # Change direction by adjusting the servo angle
    # For example, turn the steering servo to the right
    steering_servo.set_angle(90)  # Adjust the angle as needed

    # Back up the car for a brief moment
    backward()
    time.sleep(1)  # Adjust the duration as needed

    # Resume forward motion
    forward()
    time.sleep(1)  # Continue forward for a brief moment

    # Restore the steering servo to its original position
    steering_servo.set_angle(0)  # Adjust the angle as needed

    # Sleep to allow time for the car to stop and change direction
    time.sleep(1)


try:
    while True:
        distance = ultrasonic.get_distance()
        if distance > 0 and distance < 20:  # Adjust the range as needed
            avoid_obstacle()
        time.sleep(0.1)

except KeyboardInterrupt:
    pass

    # The following commented-out functions are for more advanced functionality
    # def get_distance_at(self, angle):
    #     self.servo.set_angle(angle)
    #     time.sleep(0.04)
    #     distance = self.get_distance()
    #     self.angle_distance = [angle, distance]
    #     return distance

    # def get_status_at(self, angle, ref1=35, ref2=10):
    #     dist = self.get_distance_at(angle)
    #     if dist > ref1 or dist == -2:
    #         return 2
    #     elif dist > ref2:
    #         return 1
    #     else:
    #         return 0

    # def scan_step(self, ref):
    #     if self.current_angle >= self.max_angle:
    #         self.current_angle = self.max_angle
    #         us_step = -self.STEP
    #     elif self.current_angle <= self.min_angle:
    #         self.current_angle = self.min_angle
    #         us_step = self.STEP
    #     self.current_angle += us_step
    #     status = self.get_status_at(self.current_angle, ref1=ref)
    #     self.scan_list.append(status)
    #     if self.current_angle == self.min_angle or self.current_angle == self.max_angle:
    #         if us_step < 0:
    #             self.scan_list.reverse()
    #         self.scan_list = []
    #         return self.scan_list
    #     else:
    #         return False
