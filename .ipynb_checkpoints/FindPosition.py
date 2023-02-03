import matplotlib.pyplot as plt

# Still needs stuff for adjusting for rotation. :-)
SENSOR_RATE = 1


def trapizoidal_int(prevX, prevY, prevZ, currentX, currentY, currentZ):
    # Use a trapizoidal integration
    xChange = (1/SENSOR_RATE) * ((prevX + currentX)/2)
    yChange = (1/SENSOR_RATE) * ((prevY + currentY)/2)
    zChange = (1/SENSOR_RATE) * ((prevZ + currentZ)/2)
    return xChange, yChange, zChange


def test():
    t = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    AccX = [1, 2, 3, 4, 5, 6, 7, 8, 9]  # Pretend this is a stream of data
    VelX = []
    PosX = []
    curVelX, curVelY, curVelZ = 0, 0, 0     # Current Velocity
    curPosX, curPosY, curPosZ = 0, 0, 0     # Current Position
    prevVelX, prevVelY, prevVelZ = 0, 0, 0  # Previous Velocity
    prevAccX, prevAccY, prevAccZ = 0, 0, 0  # Previous Acceleration

    for xAccel in AccX:
        change_x, change_y, change_z = trapizoidal_int(prevAccX, prevAccY, prevAccZ, xAccel, 0, 0)
        curVelX, curVelY, curVelZ = curVelX + change_x, curVelY + change_y, curVelZ + change_z

        change_x, change_y, change_z = trapizoidal_int(prevVelX, prevVelY, prevVelZ, curVelX, curVelY, curVelZ)
        curPosX, curPosY, curPosZ = curPosX + change_x, curPosY + change_y, curPosZ + change_z

        prevVelX, prevVelY, prevVelZ = curVelX, curVelY, curVelZ
        prevAccX, prevAccY, prevAccZ = xAccel, 0, 0
        print(f"Velocity: {curVelX, curVelY, curVelZ}")
        print(f"Position: {curPosX, curPosY, curPosZ}")
        VelX.append(curVelX)
        PosX.append(curPosX)

    # Plot Results
    plt.subplot(1, 3, 1)
    plt.plot(t, AccX)
    plt.subplot(1,3,2)
    plt.plot(t, VelX)
    plt.subplot(1,3,3)
    plt.plot(t, PosX)
    plt.show()

if __name__ == "__main__":
    test()
