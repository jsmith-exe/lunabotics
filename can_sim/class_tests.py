""" Simple test file for checking some class functionality. """
from src.classes import Motor, Actuator
import threading
from time import sleep

motor = Motor(1)
actuator = Actuator(2)

def print_state():
    while True:
        print(f"{motor}  {actuator}")
        sleep(0.1)

def update_loop():
    dt = 0.01
    while True:
        motor.update(dt)
        actuator.update(dt)
        sleep(dt)

threading.Thread(target=print_state, daemon=True).start()
threading.Thread(target=update_loop, daemon=True).start()

def test_motor_duty():
    sleep(3)
    motor.set_duty(1)
    sleep(3)
    motor.set_duty(-1)
    sleep(3)
    motor.set_duty(0)
def test_motor_vel():
    sleep(3)
    motor.set_velocity(5000)
    sleep(3)
    motor.set_velocity(1000)
    sleep(3)
    motor.set_velocity(-3000)
    sleep(3)
    motor.set_velocity(0)
def test_actuator_duty():
    sleep(3)
    actuator.set_duty(1)
    sleep(3)
    actuator.set_duty(0)
    sleep(3)
    actuator.set_duty(-1)
    sleep(3)
    actuator.set_duty(0)

if __name__ == '__main__':
    # test_motor_duty()
    # test_motor_vel()
    test_actuator_duty()
    sleep(3)
