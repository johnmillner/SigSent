import pigpio
from ga import *

mcu = MCU()
# Left, 0 degrees at the top
# right, 0 degrees at the bottom

# front right is 0
# front left is 1
# middle right is 2
# middle left is 3
# back right is 4
# back left is 5
step1 = Step(
    legs=[
        Leg(states=[mcu.degrees_to_servo(100),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()])
    ]
)
"""
step2 = Step(
    legs=[
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()])
    ]
)

step3 = Step(
    legs=[
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()])
    ]
)

step4 = Step(
    legs=[
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()])
    ]
)

step5 = Step(
    legs=[
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()])
    ]
)

step6 = Step(
    legs=[
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()]),
        Leg(states=[mcu.degrees_to_servo(),mcu.degrees_to_servo(),mcu.degrees_to_servo()])
    ]
)


gait = Gait(steps=[step1, step2, step3, step4, step5, step6])
"""

gait = Gait(steps=[step1])

mcu.send_gait_mcu(gait)