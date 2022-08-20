# Make sure to have the server side running in CoppeliaSim:
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

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
import pyRTOS

# ---------------------------------------------------

# PyRTOS definitions for messages ----------


# ---------------------------------------------------

# Name for objects handles -----------
ROBOT = "/PioneerP3DX"
RIGHT_MOTOR = f"{ROBOT}/rightMotor"
LEFT_MOTOR = f"{ROBOT}/leftMotor"
ULTRASONIC = "/ultrasonicSensor"

# ---------------------------------------------------

print("Program started")
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart("127.0.0.1", 19999, True, True, 5000, 5)
if clientID != -1:
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)
    print("Connected to remote API server")
    sim.simxAddStatusbarMessage(clientID, "Starting...", sim.simx_opmode_oneshot_wait)
    time.sleep(0.02)

    # Getting handles ------------
    erro, robot = sim.simxGetObjectHandle(clientID, ROBOT, sim.simx_opmode_blocking)
    if erro != 0:
        print("Error getting handles")
        sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
        sim.simxFinish(-1)
        exit()

    erro, rightMotor = sim.simxGetObjectHandle(
        clientID, RIGHT_MOTOR, sim.simx_opmode_blocking
    )
    if erro != 0:
        print("Error getting handles")
        sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
        sim.simxFinish(-1)
        exit()

    erro, leftMotor = sim.simxGetObjectHandle(
        clientID, LEFT_MOTOR, sim.simx_opmode_blocking
    )
    if erro != 0:
        print("Error getting handles")
        sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
        sim.simxFinish(-1)
        exit()

    ultrasonic = []
    for i in range(0, 16):
        erro, aux = sim.simxGetObjectHandle(
            clientID, f"{ULTRASONIC}[{i}]", sim.simx_opmode_blocking
        )
        if erro != 0:
            print("Error getting handles")
            sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
            sim.simxFinish(-1)
            exit()
        ultrasonic.append(aux)

    # ---------------------------------------------------

    execution_time = []
    # Tasks definition ---------------------------------------
    def task_wander(self):
        # Setup Code

        wander_velocity = 1

        # End Setup code

        # Pass control back to RTOS
        yield

        # Thread loop
        while True:
            sim.simxSetJointTargetVelocity(
                clientID, rightMotor, wander_velocity, sim.simx_opmode_oneshot
            )
            sim.simxSetJointTargetVelocity(
                clientID, leftMotor, wander_velocity, sim.simx_opmode_oneshot
            )

    # ------------------------------------------------------------------------------

    # Adding tasks -------------------------------------------
    pyRTOS.add_task(
        pyRTOS.Task(
            task_wander,
            priority=20,
            name="wander",
            notifications=None,
            mailbox=False,
        )
    )
    # pyRTOS.add_task(
    #     pyRTOS.Task(
    #         task_liga_ventoinha,
    #         priority=2,
    #         name="liga_ventoinha",
    #         notifications=None,
    #         mailbox=True,
    #     )
    # )
    # pyRTOS.add_task(
    #     pyRTOS.Task(task_LED, priority=6, name="LED", notifications=None, mailbox=True)
    # )

    # -------------------------------------------------------------------------------

    # Start RTOS
    pyRTOS.start()

    # Now close the connection to CoppeliaSim --------------------------------------
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
    sim.simxAddStatusbarMessage(clientID, "Finishing...", sim.simx_opmode_blocking)
    sim.simxFinish(-1)
else:
    sim.simxFinish(-1)
    print("Failed connecting to remote API server")
print("Program ended")
