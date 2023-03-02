import math
from functools import partial

import numpy as np
from pyquaternion import Quaternion
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

def main():
    arr = np.loadtxt("rotdata.csv",
                     delimiter=",", dtype=str)
    fig = plt.figure()

    # Defining the axes as a
    # 3D axes so that we can plot 3D
    # data into it.
    ax = plt.axes(projection="3d")
    #anim = FuncAnimation(fig, animate, fargs=(ax, arr), frames=10, interval=20, repeat=False)
    #animate(0, ax, arr)
    animate(3, ax, arr)
    plt.show()


def animate(i, ax, arr):
    ax.clear()
    ax.set_xlim([-10, 10])
    ax.set_ylim([-10, 10])
    ax.set_zlim([-10, 10])
    xi = int(arr[i][0])/100
    x = [0, xi] # Add a zero to the point so that matplotlib will draw it properly.
    yi=int(arr[i][1])/100
    y = [0, yi]
    zi = int(arr[i][2])/100
    z = [0, zi]
    print(math.sqrt(xi*xi + yi*yi + zi*zi))

    quat_array = arr[i,3:7].astype(float)
    quat_array = quat_array / (16384-1) # Quaternion data ranges from 0-2^14-1.
    q = Quaternion(quat_array)

    vect = [0, xi, yi, zi] # Convert vector to quaternion

    accq = Quaternion(vect)

    result = q.conjugate*accq*q # This should apply the opposite rotation to acceleration

    final_vector = result.elements

    # Try with rotation matricies:
    accMtx = np.matrix([[xi], [yi], [zi]])
    rotMtx = np.matrix(q.rotation_matrix)
    result_rotmtx = rotMtx.T * accMtx
    x2 = [0, result_rotmtx[0,0]]
    y2 = [0, result_rotmtx[1,0]]
    z2 = [0, result_rotmtx[2,0]]

    #x2 = [0, final_vector[0]]
    #y2 = [0, final_vector[1]]
    #z2 = [0, final_vector[2]]

    ax.plot3D(x, y, z, 'red')
    ax.plot3D(x2, y2, z2, 'green')



if (__name__ == "__main__"):
    main()