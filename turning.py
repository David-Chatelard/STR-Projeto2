try:
    import sim
except:
    print("--------------------------------------------------------------")
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print("Make sure both are in the same folder as this file,")
    print('or appropriately adjust the file "sim.py"')
    print("--------------------------------------------------------------")
    print("")

import time

# Global variables
wanderVelocity = 1
nearCubeVelocity = 0.4
fasterVelocityToTurn = 0.8
slowerVelocityToTurn = 0.3


def stop(clientID, right_motor_handle, left_motor_handle):
    sim.simxPauseCommunication(clientID, True)
    sim.simxSetJointTargetVelocity(
        clientID, right_motor_handle, 0, sim.simx_opmode_oneshot
    )
    sim.simxSetJointTargetVelocity(
        clientID, left_motor_handle, 0, sim.simx_opmode_oneshot
    )
    sim.simxPauseCommunication(clientID, False)
    time.sleep(0.5)


def free_spin(clientID, direction, velocity, right_motor_handle, left_motor_handle):
    # direction = 1 , anti clock wise
    # direction = -1 , clock wise
    # velocity = velocity
    sim.simxPauseCommunication(clientID, True)
    if direction == 1:
        sim.simxSetJointTargetVelocity(
            clientID, right_motor_handle, fasterVelocityToTurn, sim.simx_opmode_oneshot
        )
        sim.simxSetJointTargetVelocity(
            clientID, left_motor_handle, slowerVelocityToTurn, sim.simx_opmode_oneshot
        )
    elif direction == -1:
        sim.simxSetJointTargetVelocity(
            clientID, right_motor_handle, slowerVelocityToTurn, sim.simx_opmode_oneshot
        )
        sim.simxSetJointTargetVelocity(
            clientID, left_motor_handle, fasterVelocityToTurn, sim.simx_opmode_oneshot
        )
    # sim.simxSetJointTargetVelocity(
    #     clientID,
    #     right_motor_handle,
    #     direction * velocity,
    #     sim.simx_opmode_oneshot,
    # )
    # sim.simxSetJointTargetVelocity(
    #     clientID,
    #     left_motor_handle,
    #     direction * velocity * (-1),
    #     sim.simx_opmode_oneshot,
    # )
    sim.simxPauseCommunication(clientID, False)


def get_angle_that_makes_sense(clientID, robot_handle):
    erro, euler_angles = sim.simxGetObjectOrientation(
        clientID, robot_handle, -1, sim.simx_opmode_streaming
    )
    while erro != 0:
        erro, euler_angles = sim.simxGetObjectOrientation(
            clientID, robot_handle, -1, sim.simx_opmode_streaming
        )
    factor = 270
    # print(factor)
    # print(factor)
    finalAngle = (60 * euler_angles[2]) + factor
    if finalAngle > 360:
        finalAngle -= 360
    return finalAngle


def turn_90_degrees(
    clientID, direction, robot_handle, right_motor_handle, left_motor_handle
):
    # direction = 1 , anti clock wise, left
    # direction = -1 , clock wise, right

    velocity = 0.5
    starting_angle = get_angle_that_makes_sense(clientID, robot_handle)
    # print(f"starting angle: {starting_angle}")

    free_spin(clientID, direction, velocity, right_motor_handle, left_motor_handle)
    while True:
        final_angle = get_angle_that_makes_sense(clientID, robot_handle)

        turned_angle = final_angle - starting_angle

        if direction == 1:  # anti clock wise
            if turned_angle < 0:  # 4th to 1st quadrant
                turned_angle += 360  # transform into angle between 0 and 360 degrees
        elif direction == -1:  # clock wise
            # since the robot is turning clockwise, the angle will be negative, so multiply by -1
            turned_angle *= -1
            if turned_angle < 0:  # 1st to 4th quadrant
                turned_angle += 360  # transform into angle between 0 and 360 degrees

        if (88) < turned_angle < (94):
            # print(f"final angle: {final_angle}")
            # print(f"turned angle: {turned_angle}")
            break
    stop(clientID, right_motor_handle, left_motor_handle)
