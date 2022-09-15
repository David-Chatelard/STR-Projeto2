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

# Documentation: https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm

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
import numpy as np
import pyRTOS

# ---------------------------------------------------

# Global variables
FIRST_ULTRASONIC = 0
LAST_ULTRASONIC = 4  # last ultrasonic I want to read + 1, because range() doesn't include the last number
wanderVelocity = 1
nearCubeVelocity = 0.4
fasterVelocityToTurn = 0.8
slowerVelocityToTurn = 0.3
ultrasonicValue = 0
minDistance = 0.5
maxDistance = 1
colorSensorValue = ""
timeTurning = 1
isTurning = False


# PyRTOS definitions for messages ----------
HAS_READ_ULSTRASONIC = 128
HAS_READ_COLOR = 129
HAS_TURNED = 130

# ---------------------------------------------------

# Name for objects handles -----------
ROBOT = "/PioneerP3DX"
RIGHT_MOTOR = f"{ROBOT}/rightMotor"
LEFT_MOTOR = f"{ROBOT}/leftMotor"
ULTRASONIC = "/ultrasonicSensor"
COLOR_SENSOR = "/colorSensor"

# ---------------------------------------------------

print("Program started")
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart("127.0.0.1", 19999, True, True, 5000, 5)
if clientID != -1:
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)
    print("Connected to remote API server")
    sim.simxAddStatusbarMessage(clientID, "Starting...", sim.simx_opmode_oneshot_wait)
    time.sleep(0.02)

    # Getting handles -------------------------------------------------------------------------------------------
    # Robot handle
    error, robot = sim.simxGetObjectHandle(clientID, ROBOT, sim.simx_opmode_blocking)
    if error != 0:
        print("Error getting handles")
        sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
        sim.simxFinish(-1)
        exit()

    # Right motor handles
    error, rightMotor = sim.simxGetObjectHandle(
        clientID, RIGHT_MOTOR, sim.simx_opmode_blocking
    )
    if error != 0:
        print("Error getting rightMotor handles")
        sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
        sim.simxFinish(-1)
        exit()

    # Left motor handles
    error, leftMotor = sim.simxGetObjectHandle(
        clientID, LEFT_MOTOR, sim.simx_opmode_blocking
    )
    if error != 0:
        print("Error getting leftMotor handles")
        sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
        sim.simxFinish(-1)
        exit()

    # Ultrasonics handles
    ultrasonic = []
    for i in range(FIRST_ULTRASONIC, LAST_ULTRASONIC):
        error, aux = sim.simxGetObjectHandle(
            clientID, f"{ULTRASONIC}[{i}]", sim.simx_opmode_blocking
        )
        if error != 0:
            print(f"Error getting ultrasonic[{i}] handles")
            sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
            sim.simxFinish(-1)
            exit()
        ultrasonic.append(aux)

    # Color sensor handle
    error, colorSensor = sim.simxGetObjectHandle(
        clientID, f"{COLOR_SENSOR}", sim.simx_opmode_blocking
    )
    if error != 0:
        print("Error getting colorSensor handles")
        sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
        sim.simxFinish(-1)
        exit()

    # ----------------------------------------------------------------------------------

    # Tasks definition -----------------------------------------------------------------
    def task_wander(self):
        # Setup Code

        # End Setup code

        # Pass control back to RTOS
        yield

        # Thread loop
        while True:
            # Work code

            # Setting wandering velocity
            sim.simxPauseCommunication(clientID, True)
            sim.simxSetJointTargetVelocity(
                clientID, rightMotor, wanderVelocity, sim.simx_opmode_oneshot
            )
            sim.simxSetJointTargetVelocity(
                clientID, leftMotor, wanderVelocity, sim.simx_opmode_oneshot
            )
            sim.simxPauseCommunication(clientID, False)
            time.sleep(0.1)
            # End Work code

            yield [pyRTOS.wait_for_message(self)]

    def task_ultrasonic(self):
        # Setup Code
        global ultrasonicValue

        # for i in range(len(ultrasonic)):
        #     (
        #         error,
        #         detectionState,
        #         distancePoint,
        #         detectedObjectHandle,
        #         detectedSurface,
        #     ) = sim.simxReadProximitySensor(
        #         clientID, ultrasonic[i], sim.simx_opmode_streaming
        #     )
        #     while error != 0:
        #         (
        #             error,
        #             detectionState,
        #             distancePoint,
        #             detectedObjectHandle,
        #             detectedSurface,
        #         ) = sim.simxReadProximitySensor(
        #             clientID, ultrasonic[i], sim.simx_opmode_buffer
        #         )
        #         # print("Error reading proximity sensor")
        #         # sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
        #         # sim.simxFinish(-1)
        #         # exit()
        #     if detectionState == False:
        #         ultrasonicValue = maxDistance
        #     else:
        #         ultrasonicValue = distancePoint[2]

        # End Setup code

        # Pass control back to RTOS
        yield

        # Thread loop
        while True:
            # Work code
            for i in range(len(ultrasonic)):
                # Reading all ultrasonic sensors
                (
                    error,
                    detectionState,
                    distancePoint,
                    detectedObjectHandle,
                    detectedSurface,
                ) = sim.simxReadProximitySensor(
                    clientID, ultrasonic[i], sim.simx_opmode_streaming
                )
                while error != 0:
                    (
                        error,
                        detectionState,
                        distancePoint,
                        detectedObjectHandle,
                        detectedSurface,
                    ) = sim.simxReadProximitySensor(
                        clientID, ultrasonic[i], sim.simx_opmode_buffer
                    )
                    # print("Error reading proximity sensor")
                    # sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
                    # sim.simxFinish(-1)
                    # exit()

                # If didn't detect anything, set distance to maxDistance
                if detectionState == False:
                    # print(f"Sensor {i} No obstacle detected")
                    ultrasonicValue = maxDistance
                else:
                    # print(f"Sensor {i} Obstacle detected at {distancePoint[2]}")
                    ultrasonicValue = distancePoint[2]
                    # If obstacle is too close, slow down the robot and send message
                    if ultrasonicValue <= minDistance:
                        sim.simxPauseCommunication(clientID, True)
                        sim.simxSetJointTargetVelocity(
                            clientID,
                            rightMotor,
                            nearCubeVelocity,
                            sim.simx_opmode_oneshot,
                        )
                        sim.simxSetJointTargetVelocity(
                            clientID,
                            leftMotor,
                            nearCubeVelocity,
                            sim.simx_opmode_oneshot,
                        )
                        sim.simxPauseCommunication(clientID, False)
                        time.sleep(0.1)

                        self.send(
                            pyRTOS.Message(
                                HAS_READ_ULSTRASONIC,
                                self,
                                "color_sensor",
                                ultrasonicValue,
                            )
                        )
            # End Work code

            yield [pyRTOS.timeout(0.05)]

    def task_color_sensor(self):
        # Setup Code
        global colorSensorValue
        # End Setup code

        # Pass control back to RTOS
        yield

        # Thread loop
        while True:
            # Work code

            # If distance is less than minDistance, read color sensor
            if ultrasonicValue < minDistance:
                print(
                    f"ultrasonicValue: {ultrasonicValue} and minDistance: {minDistance}"
                )
                erro, resolution, Image = sim.simxGetVisionSensorImage(
                    clientID, colorSensor, 0, sim.simx_opmode_streaming
                )
                while erro != 0:
                    erro, resolution, Image = sim.simxGetVisionSensorImage(
                        clientID, colorSensor, 0, sim.simx_opmode_buffer
                    )

                # Getting color value from sensor reading
                img = np.array(Image, dtype=np.uint8)
                minColorValues = 130
                rgbColor = 0
                if img[0] > minColorValues:
                    rgbColor += 100
                if img[1] > minColorValues:
                    rgbColor += 10
                if img[2] > minColorValues:
                    rgbColor += 1

                if rgbColor == 1:
                    colorSensorValue = "BLUE"
                elif rgbColor == 10:
                    colorSensorValue = "GREEN"
                elif rgbColor == 100:
                    colorSensorValue = "RED"
                elif rgbColor == 110:
                    colorSensorValue = "YELLOW"
                elif rgbColor == 111:
                    colorSensorValue = "WHITE"
                else:
                    colorSensorValue = "BLACK"
                # print(f"img0: {img[0]}")
                # print(f"img1: {img[1]}")
                # print(f"img2: {img[2]}")
                # print(f"rgbColor: {rgbColor}")
                print(f"Color Sensor: {colorSensorValue}")
                self.send(
                    pyRTOS.Message(
                        HAS_READ_COLOR,
                        self,
                        "route_manager",
                        colorSensorValue,
                    )
                )

            # End Work code
            yield [pyRTOS.timeout(0.05)]

    def task_route_manager(self):
        # Setup Code
        global isTurning
        global colorSensorValue
        # End Setup code

        # Pass control back to RTOS
        yield

        # Thread loop
        while True:
            # Work code

            # If color is RED, turn left
            if colorSensorValue == "RED":
                colorSensorValue = ""
                isTurning = True

                # TODO Blink left LED

                # Making right motor faster than left motor, in order to turn left
                print("STARTED TURNING LEFT")
                sim.simxPauseCommunication(clientID, True)
                sim.simxSetJointTargetVelocity(
                    clientID, rightMotor, fasterVelocityToTurn, sim.simx_opmode_oneshot
                )
                sim.simxSetJointTargetVelocity(
                    clientID, leftMotor, slowerVelocityToTurn, sim.simx_opmode_oneshot
                )
                sim.simxPauseCommunication(clientID, False)
                time.sleep(timeTurning)

                isTurning = False

                # Going back to normal velocity
                print("STOPED TURNING LEFT")
                sim.simxPauseCommunication(clientID, True)
                sim.simxSetJointTargetVelocity(
                    clientID, rightMotor, wanderVelocity, sim.simx_opmode_oneshot
                )
                sim.simxSetJointTargetVelocity(
                    clientID, leftMotor, wanderVelocity, sim.simx_opmode_oneshot
                )
                sim.simxPauseCommunication(clientID, False)
                time.sleep(0.1)

                self.send(
                    pyRTOS.Message(
                        HAS_TURNED,
                        self,
                        "wander",
                        colorSensorValue,
                    )
                )

            # If color is BLUE, turn right
            elif colorSensorValue == "BLUE":
                colorSensorValue = ""
                isTurning = True

                # TODO Blink right LED

                # Making left motor faster than right motor, in order to turn right
                print("STARTED TURNING RIGHT")
                sim.simxPauseCommunication(clientID, True)
                sim.simxSetJointTargetVelocity(
                    clientID, rightMotor, slowerVelocityToTurn, sim.simx_opmode_oneshot
                )
                sim.simxSetJointTargetVelocity(
                    clientID, leftMotor, fasterVelocityToTurn, sim.simx_opmode_oneshot
                )
                sim.simxPauseCommunication(clientID, False)
                time.sleep(timeTurning)

                isTurning = False

                # Going back to normal velocity
                print("STOPED TURNING RIGHT")
                sim.simxPauseCommunication(clientID, True)
                sim.simxSetJointTargetVelocity(
                    clientID, rightMotor, wanderVelocity, sim.simx_opmode_oneshot
                )
                sim.simxSetJointTargetVelocity(
                    clientID, leftMotor, wanderVelocity, sim.simx_opmode_oneshot
                )
                sim.simxPauseCommunication(clientID, False)
                time.sleep(0.1)

                self.send(
                    pyRTOS.Message(
                        HAS_TURNED,
                        self,
                        "wander",
                        colorSensorValue,
                    )
                )
            # End Work code

            yield [pyRTOS.wait_for_message(self)]

    # ----------------------------------------------------------------------------------------

    # Adding tasks ---------------------------------------------------------------------------
    pyRTOS.add_task(
        pyRTOS.Task(
            task_wander,
            priority=20,
            name="wander",
            notifications=None,
            mailbox=True,
        )
    )
    pyRTOS.add_task(
        pyRTOS.Task(
            task_ultrasonic,
            priority=1,
            name="ultrasonic",
            notifications=None,
            mailbox=True,
        )
    )
    pyRTOS.add_task(
        pyRTOS.Task(
            task_color_sensor,
            priority=1,
            name="color_sensor",
            notifications=None,
            mailbox=True,
        )
    )
    pyRTOS.add_task(
        pyRTOS.Task(
            task_route_manager,
            priority=5,
            name="route_manager",
            notifications=None,
            mailbox=True,
        )
    )
    # pyRTOS.add_task(
    #     pyRTOS.Task(task_LED, priority=6, name="LED", notifications=None, mailbox=True)
    # )

    # ----------------------------------------------------------------------------------------

    # Start RTOS
    pyRTOS.start()

    # Now close the connection to CoppeliaSim ------------------------------------------------
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
    sim.simxAddStatusbarMessage(clientID, "Finishing...", sim.simx_opmode_blocking)
    sim.simxFinish(-1)
else:
    sim.simxFinish(-1)
    print("Failed connecting to remote API server")
print("Program ended")
