# DPP-Sesi-3

import RPi.GPIO as GPIO
import time
import math

# Inisialisasi pin servo
servo_pins = [2, 3, 4]  # Ganti dengan pin yang sesuai
GPIO.setmode(GPIO.BCM)
servos = []
for pin in servo_pins:
    GPIO.setup(pin, GPIO.OUT)
    servos.append(GPIO.PWM(pin, 50))  # 50 Hz PWM

# Konstanta
J2L = 57.0  # Panjang J2 (57 - 2.35)mm
J3L = 110.0  # Panjang J3 110mm
Y_Rest = 70.0
Z_Rest = -80.0
J3_LegAngle = 15.4

# Joint Variables
J1Act = 0.0
J2Act = 0.0
J3Act = 40.0

# Command Variables
started = False
ended = False
commandStep = 0

# Commands
lines = [[0.0, 0.0, 40.0, 1000],
         [-30.0, 40.0, 20.0, 200],
         [-30.0, 40.0, -20.0, 200],
         [60.0, 40.0, -20.0, 1600],
         [60.0, 60.0, 20.0, 200],
         # Masukkan baris berikutnya di sini
         {-30.0, 30.0, 20.0, 200},
                            {-30.0, 30.0, -20.0, 200},
                            {60.0, 30.0, -20.0, 1600},
                            {60.0, 50.0, 20.0, 200},

                            {-30.0, 20.0, 20.0, 200},
                            {-30.0, 20.0, -20.0, 200},
                            {60.0, 20.0, -20.0, 1600},
                            {60.0, 40.0, 20.0, 200},

                            {-30.0, 10.0, 20.0, 200},
                            {-30.0, 10.0, -20.0, 200},
                            {60.0, 10.0, -20.0, 1600},
                            {60.0, 30.0, 20.0, 200},

                            {-30.0, 0.0, 20.0, 200},
                            {-30.0, 0.0, -20.0, 200},
                            {60.0, 0.0, -20.0, 1600},
                            {60.0, 20.0, 20.0, 200},                                                      



                            {-30.0, 40.0, 20.0, 200},
                            {-30.0, 40.0, -20.0, 200},
                            {-30.0, 0.0, -20.0, 800},
                            {-30.0, 20.0, 20.0, 200},

                            {-20.0, 40.0, 20.0, 200},
                            {-20.0, 40.0, -20.0, 200},
                            {-20.0, 0.0, -20.0, 800},
                            {-20.0, 20.0, 20.0, 200},

                            {-10.0, 40.0, 20.0, 200},
                            {-10.0, 40.0, -20.0, 200},
                            {-10.0, 0.0, -20.0, 800},
                            {-10.0, 20.0, 20.0, 200},
                            
                            {0.0, 40.0, 20.0, 200},
                            {0.0, 40.0, -20.0, 200},
                            {0.0, 0.0, -20.0, 800},
                            {0.0, 20.0, 20.0, 200},

                            {10.0, 40.0, 20.0, 200},
                            {10.0, 40.0, -20.0, 200},
                            {10.0, 0.0, -20.0, 800},
                            {10.0, 20.0, 20.0, 200},

                            {20.0, 40.0, 0.0, 200},
                            {20.0, 40.0, -20.0, 200},
                            {20.0, 0.0, -20.0, 800},
                            {20.0, 20.0, 0.0, 200},

                            {30.0, 40.0, 0.0, 200},
                            {30.0, 40.0, -20.0, 200},
                            {30.0, 0.0, -20.0, 800},
                            {30.0, 20.0, 0.0, 200},

                            {40.0, 40.0, 0.0, 200},
                            {40.0, 40.0, -20.0, 200},
                            {40.0, 0.0, -20.0, 800},
                            {40.0, 20.0, 0.0, 200},

                            {50.0, 40.0, 0.0, 200},
                            {50.0, 40.0, -20.0, 200},
                            {50.0, 0.0, -20.0, 800},
                            {50.0, 20.0, 0.0, 200},

                            {60.0, 40.0, 0.0, 200},
                            {60.0, 40.0, -20.0, 200},
                            {60.0, 0.0, -20.0, 800},
                            {60.0, 20.0, 0.0, 200},

                            {0.0, 0.0, 40.0, 200}]

def move_servos(angle1, angle2, angle3):
    for i in range(3):
        servos[i].start(angle1 / 18.0 + 2.5)
    time.sleep(1)
    for servo in servos:
        servo.stop()

def cartesian_move(X, Y, Z):
    # OFFSET TO REST POSITION
    Y += Y_Rest
    Z += Z_Rest

    # CALCULATE INVERSE KINEMATIC SOLUTION
    J1 = math.atan(X / Y) * (180 / math.pi)
    H = math.sqrt((Y * Y) + (X * X))
    L = math.sqrt((H * H) + (Z * Z))
    J3 = math.acos(((J2L * J2L) + (J3L * J3L) - (L * L)) / (2 * J2L * J3L)) * (180 / math.pi)
    B = math.acos(((L * L) + (J2L * J2L) - (J3L * J3L)) / (2 * L * J2L)) * (180 / math.pi)
    A = math.atan(Z / H) * (180 / math.pi)  # BECAUSE Z REST IS NEGATIVE, THIS RETURNS A NEGATIVE VALUE
    J2 = (B + A)  # BECAUSE 'A' IS NEGATIVE AT REST WE NEED TO INVERT '-' TO '+'

    update_position(J1, J2, J3)

def update_position(J1, J2, J3):
    # MOVE TO POSITION
    servos[0].start(90 - J1 / 18.0 + 2.5)
    servos[1].start(90 - J2 / 18.0 + 2.5)
    servos[2].start(J3 + J3_LegAngle - 90)
    time.sleep(1)
    for servo in servos:
        servo.stop()

try:
    while True:
        # Update position
        J1Act += 1
        J2Act += 1
        J3Act += 1

        cartesian_move(J1Act, J2Act, J3Act)

        if not ended:
            # Check if command finished
            if not started or J1Act >= len(lines):
                commandStep += 1
                if commandStep >= len(lines):
                    ended = True

                xMove, yMove, zMove, duration = lines[commandStep]

                cartesian_move(xMove, yMove, zMove)

                started = True

except KeyboardInterrupt:
    pass

finally:
    for servo in servos:
        servo.stop()
    GPIO.cleanup()
