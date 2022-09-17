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
import turning

# ---------------------------------------------------

# Global variables-----------------------------------
# Number of sensors
FIRST_ULTRASONIC = 0
LAST_ULTRASONIC = 4  # last ultrasonic I want to read + 1, because range() doesn't include the last number
FIRST_COLOR_SENSOR = 0
LAST_COLOR_SENSOR = 3  # last color sensor I want to read + 1, because range() doesn't include the last number

# Velocity
wanderVelocity = 1
nearCubeVelocity = 0.4
fasterVelocityToTurn = 0.8
slowerVelocityToTurn = 0.3

# Sensors values
ultrasonicValue = 100
colorSensorValue = ""

# Control variables
minDistance = 0.7
maxDistance = 1
timeTurning = 1
isTurning = False
ultrasonicDetected = [False, False, False, False]

# LED positions
position_OFF_LED = [0, 0, 0]
position_ON_LEFT_LED = [-0.185, 0.097, 0.00519]
position_ON_RIGHT_LED = [-0.185, -0.097, 0.00519]

# Array for execution times
executionTimes = []


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
LEFT_LED = "leftLED"
RIGHT_LED = "rightLED"

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

    # Color sensors handle
    colorSensor = []
    for i in range(FIRST_COLOR_SENSOR, LAST_COLOR_SENSOR):
        error, aux = sim.simxGetObjectHandle(
            clientID, f"{COLOR_SENSOR}{i}", sim.simx_opmode_blocking
        )
        if error != 0:
            print(f"Error getting colorSensor{i} handles")
            sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
            sim.simxFinish(-1)
            exit()
        colorSensor.append(aux)

    # Lights handles
    error, leftLED = sim.simxGetObjectHandle(
        clientID, LEFT_LED, sim.simx_opmode_blocking
    )
    error, rightLED = sim.simxGetObjectHandle(
        clientID, RIGHT_LED, sim.simx_opmode_blocking
    )

    # ----------------------------------------------------------------------------------

    # Tasks definition -----------------------------------------------------------------
    def task_wander(self):
        # Setup Code

        # End Setup code

        # Pass control back to RTOS
        yield

        # Thread loop
        while True:
            startingTime = time.time_ns()

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

            executionTimes.append(time.time_ns() - startingTime)

            yield [pyRTOS.wait_for_message(self)]

    def task_ultrasonic(self):
        # Setup Code
        global ultrasonicValue
        global ultrasonicDetected

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

                # If didn't detect anything, set distance to maxDistance
                if detectionState == False:
                    # print(f"Sensor {i} No obstacle detected")
                    ultrasonicValue = maxDistance
                else:
                    # print(f"Sensor {i} Obstacle detected at {distancePoint[2]}")
                    ultrasonicValue = distancePoint[2]
                    # If obstacle is too close, slow down the robot and send message
                    if ultrasonicValue <= minDistance:
                        # print(f"Detected obstacle at sensor{i}")
                        # Specify wich ultrasonic detected the obstacle
                        ultrasonicDetected[i] = True

                        # Setting slower velocity
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
                        if i != 2:
                            break
            # End Work code
            print(f"Execution Times: {executionTimes}")

            yield [pyRTOS.timeout(0.05)]

    def task_color_sensor(self):
        # print("ENTERED COLOR SENSOR")
        # Setup Code
        global colorSensorValue
        global ultrasonicValue
        global ultrasonicDetected
        # End Setup code

        # Pass control back to RTOS
        yield

        # Thread loop
        while True:
            # Work code

            # If distance is less than minDistance, read color sensor
            if ultrasonicValue <= minDistance:
                # print("ENTERED IN IF AT COLOR SENSOR")
                ultrasonicValue = maxDistance
                # print(
                #     f"ultrasonicValue: {ultrasonicValue} and minDistance: {minDistance}"
                # )

                if ultrasonicDetected[0] == True:
                    index = 0
                elif ultrasonicDetected[3] == True:
                    index = 2
                # elif ultrasonicDetected[1] == True and ultrasonicDetected[2] == True:
                else:
                    index = 1
                ultrasonicDetected = [False, False, False, False]
                erro, resolution, Image = sim.simxGetVisionSensorImage(
                    clientID, colorSensor[index], 0, sim.simx_opmode_streaming
                )
                while erro != 0:
                    erro, resolution, Image = sim.simxGetVisionSensorImage(
                        clientID, colorSensor[index], 0, sim.simx_opmode_buffer
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
                # print(f"Color Sensor{index} detected: {colorSensorValue}")
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

                # Turn on left LED
                # print("STARTED TURNING LEFT")
                sim.simxSetObjectPosition(
                    clientID,
                    leftLED,
                    robot,
                    position_ON_LEFT_LED,
                    sim.simx_opmode_oneshot,
                )

                turning.turn_90_degrees(clientID, 1, robot, rightMotor, leftMotor)

                # Turn off left LED
                sim.simxSetObjectPosition(
                    clientID,
                    leftLED,
                    robot,
                    position_OFF_LED,
                    sim.simx_opmode_oneshot,
                )

                # Turn on right LED
                sim.simxSetObjectPosition(
                    clientID,
                    rightLED,
                    robot,
                    position_ON_RIGHT_LED,
                    sim.simx_opmode_oneshot,
                )

                turning.turn_90_degrees(clientID, -1, robot, rightMotor, leftMotor)

                # Turn off right LED
                sim.simxSetObjectPosition(
                    clientID,
                    rightLED,
                    robot,
                    position_OFF_LED,
                    sim.simx_opmode_oneshot,
                )
                # print("STOPED TURNING LEFT")

                # Going back to normal velocity
                sim.simxPauseCommunication(clientID, True)
                sim.simxSetJointTargetVelocity(
                    clientID, rightMotor, wanderVelocity, sim.simx_opmode_oneshot
                )
                sim.simxSetJointTargetVelocity(
                    clientID, leftMotor, wanderVelocity, sim.simx_opmode_oneshot
                )
                sim.simxPauseCommunication(clientID, False)
                time.sleep(0.1)

                # # Turn off left LED
                # sim.simxSetObjectPosition(
                #     clientID,
                #     leftLED,
                #     robot,
                #     position_OFF_LED,
                #     sim.simx_opmode_oneshot,
                # )

                isTurning = False

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

                # Turn on right LED
                # print("STARTED TURNING RIGHT")
                sim.simxSetObjectPosition(
                    clientID,
                    rightLED,
                    robot,
                    position_ON_RIGHT_LED,
                    sim.simx_opmode_oneshot,
                )

                turning.turn_90_degrees(clientID, -1, robot, rightMotor, leftMotor)

                # Turn off right LED
                sim.simxSetObjectPosition(
                    clientID,
                    rightLED,
                    robot,
                    position_OFF_LED,
                    sim.simx_opmode_oneshot,
                )

                # Turn on left LED
                sim.simxSetObjectPosition(
                    clientID,
                    leftLED,
                    robot,
                    position_ON_LEFT_LED,
                    sim.simx_opmode_oneshot,
                )

                turning.turn_90_degrees(clientID, 1, robot, rightMotor, leftMotor)

                # Turn off left LED
                sim.simxSetObjectPosition(
                    clientID,
                    leftLED,
                    robot,
                    position_OFF_LED,
                    sim.simx_opmode_oneshot,
                )
                # print("STOPED TURNING RIGHT")

                # Going back to normal velocity
                sim.simxPauseCommunication(clientID, True)
                sim.simxSetJointTargetVelocity(
                    clientID, rightMotor, wanderVelocity, sim.simx_opmode_oneshot
                )
                sim.simxSetJointTargetVelocity(
                    clientID, leftMotor, wanderVelocity, sim.simx_opmode_oneshot
                )
                sim.simxPauseCommunication(clientID, False)
                time.sleep(0.1)

                # # Turn off right LED
                # sim.simxSetObjectPosition(
                #     clientID,
                #     rightLED,
                #     robot,
                #     position_OFF_LED,
                #     sim.simx_opmode_oneshot,
                # )

                isTurning = False

                self.send(
                    pyRTOS.Message(
                        HAS_TURNED,
                        self,
                        "wander",
                        colorSensorValue,
                    )
                )

            elif colorSensorValue == "WHITE":
                colorSensorValue = ""
                isTurning = True

                # Turn on right LED
                sim.simxSetObjectPosition(
                    clientID,
                    rightLED,
                    robot,
                    position_ON_RIGHT_LED,
                    sim.simx_opmode_oneshot,
                )

                turning.turn_90_degrees(clientID, -1, robot, rightMotor, leftMotor)
                turning.turn_90_degrees(clientID, -1, robot, rightMotor, leftMotor)

                # Going back to normal velocity
                sim.simxPauseCommunication(clientID, True)
                sim.simxSetJointTargetVelocity(
                    clientID, rightMotor, wanderVelocity, sim.simx_opmode_oneshot
                )
                sim.simxSetJointTargetVelocity(
                    clientID, leftMotor, wanderVelocity, sim.simx_opmode_oneshot
                )
                sim.simxPauseCommunication(clientID, False)
                time.sleep(0.1)

                # Turn off right LED
                sim.simxSetObjectPosition(
                    clientID,
                    rightLED,
                    robot,
                    position_OFF_LED,
                    sim.simx_opmode_oneshot,
                )

                isTurning = False

                self.send(
                    pyRTOS.Message(
                        HAS_TURNED,
                        self,
                        "wander",
                        colorSensorValue,
                    )
                )

            # End Work code

            # yield [pyRTOS.wait_for_message(self)]
            yield [pyRTOS.timeout(0.05)]

    def task_blink_LEDs(self):
        # Setup Code
        global isTurning
        # End Setup code

        # Pass control back to RTOS
        yield

        # Thread loop
        while True:
            # Work code
            if not isTurning:
                sim.simxSetObjectPosition(
                    clientID,
                    leftLED,
                    robot,
                    position_ON_LEFT_LED,
                    sim.simx_opmode_oneshot,
                )
                sim.simxSetObjectPosition(
                    clientID,
                    rightLED,
                    robot,
                    position_ON_RIGHT_LED,
                    sim.simx_opmode_oneshot,
                )
                time.sleep(0.2)
                sim.simxSetObjectPosition(
                    clientID,
                    leftLED,
                    robot,
                    position_OFF_LED,
                    sim.simx_opmode_oneshot,
                )
                sim.simxSetObjectPosition(
                    clientID,
                    rightLED,
                    robot,
                    position_OFF_LED,
                    sim.simx_opmode_oneshot,
                )
                time.sleep(0.2)
            # End Work code
            yield [pyRTOS.timeout(0.2)]

    # ----------------------------------------------------------------------------------------

    # Adding tasks ---------------------------------------------------------------------------
    pyRTOS.add_task(
        pyRTOS.Task(
            task_wander,
            priority=30,
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
            priority=3,
            name="color_sensor",
            notifications=None,
            mailbox=True,
        )
    )
    pyRTOS.add_task(
        pyRTOS.Task(
            task_route_manager,
            priority=10,
            name="route_manager",
            notifications=None,
            mailbox=True,
        )
    )
    pyRTOS.add_task(
        pyRTOS.Task(
            task_blink_LEDs,
            priority=25,
            name="blink_LED",
            notifications=None,
            mailbox=True,
        )
    )

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
