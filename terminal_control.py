from motor_test import test_motor
import time
from pymavlink import mavutil
import socket
import ipaddress


def arm_rov(mav_connection):
    """
    Arm the ROV, wait for confirmation
    """
    mav_connection.arducopter_arm()
    print("Waiting for the vehicle to arm")
    mav_connection.motors_armed_wait()
    print("Armed!")


def disarm_rov(mav_connection):
    """
    Disarm the ROV, wait for confirmation
    """
    mav_connection.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    mav_connection.motors_disarmed_wait()
    print("Disarmed!")


def run_motors_timed(mav_connection, seconds: int, motor_settings: list) -> None:
    """
    Run the motors for a set time
    :param mav_connection: The mavlink connection
    :param time: The time to run the motors
    :param motor_settings: The motor settings, a list of 6 values -100 to 100
    :return: None
    """
    step = 0
    while step < seconds:
        for i in range(len(motor_settings)):
            test_motor(
                mav_connection=mav_connection, motor_id=i, power=motor_settings[i]
            )
        time.sleep(0.2)
        step += 0.2

    # while True:
    #     test_motor(mav_connection = mav_connection, motor_id = i, power = [50.0, 100.0, 50.0, 100.0, 0.0, 0.0])


def donut(mav_connection, seconds: int) -> None:
    powers = [50.0, 100, -50.0, 100.0]
    run_motors_timed(mav_connection, seconds, powers)


def forward(mav_connection, seconds: int, coeff: float):
    powers = [100.0, 100.0, -100.0, -100.0]
    run_motors_timed(mav_connection, seconds, coeff)


if __name__ == "__main__":
    ####
    # Initialize ROV
    ####
    mav_connection = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
    mav_connection.wait_heartbeat()
    # Arm the ROV and wait for confirmation
    arm_rov(mav_connection)

    ####
    # Initialize Python server
    ####

    HOST = "10.29.86.90"
    PORT = 8000

    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind((HOST, PORT))
            data = s.recv(1024).decode()
            arm_rov(mav_connection)

            if data.strip() == "DONUT":
                donut(mav_connection, 10)
            elif data.strip().isdigit():
                forward(mav_connection, 5, data)
            elif len(data.strip().split()) > 1:
                run = True

                try:
                    params = [float(x) for x in data.split()]
                    if params[0] <= 0 or len(params) > 7:
                        run = False
                except:
                    run = False

                if run:
                    run_motors_timed(mav_connection, params[0], params[1:])
            elif data.strip() == "STOP":
                disarm_rov(mav_connection)
                break
            else:
                print("Bad Command.")
                # s.sendto(b"Bad Command", ("10.29.86.90", 8000))

    ####
    # Run choreography
    ####
    """
    Call sequence of calls to run_timed_motors to execute choreography
    Motors power ranges from -100 to 100
    """
    # run_motors_timed(
    #     mav_connection, seconds=5, motor_settings=[100, -100, 100, -100, 0, 0]
    # )
    # run_motors_timed(
    #     mav_connection, seconds=5, motor_settings=[-100, 100, -100, 100, 0, 0]
    # )
    # # stop
    # run_motors_timed(mav_connection, seconds=5, motor_settings=[0, 0, 0, 0, 0, 0])

    # donut(
    #     mav_connection=mav_connection,
    #     seconds=40,
    #     motor_settings=[50.0, 100.0, -50.0, 100.0, 0.0, 0.0],
    # )

    ####
    # Disarm ROV and exit
    ####
    # disarm_rov(mav_connection)
