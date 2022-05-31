import serial
import sys
import matplotlib.pyplot as plot
import numpy as np
import math
import time

ACC_DATA = []
ACC_X = []
ACC_Y = []
ACC_Z = []
ACC_POINTS = ""
TIME_AXIS = []

CYCLES = 0
MAX_CYCLES = 50

# Serial Config
# Serial Port is name of the Serial port, check VSCode for connected port
# Baud Rate is the Serial Baud Rate used in Arduino
SERIAL_PORT = "/dev/cu.usbserial-0001"
BAUD_RATE = 9600


try:
    ard_serial = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE)

except serial.SerialException as e:
    print(f"[ERROR]: PORT IS IN USE OR DEVICE IS NOT CONNECTED! STOP OTHER SERIAL PROCESSES AND TRY AGAIN.\n\n{e}")

    sys.exit()

# cycles = eval(input("Enter the number of cycles for data to be collected to then be plotted: "))

while True:
    ser_ln = ard_serial.readline()[:-2].decode("utf-8")

    if ser_ln.startswith("-> PY"):
        ser_data = ser_ln.split("|")
        print(ser_data)

        # Acceleration of 3 Axis Combined
        acc_comb = math.sqrt(pow(float(ser_data[1]), 2) + pow(float(ser_data[2]), 2) + pow(float(ser_data[3]), 2))

        ACC_DATA.append(acc_comb)
        ACC_X.append(ser_data[1])
        ACC_Y.append(ser_data[2])
        ACC_Z.append(ser_data[3])

        # Plot Data
        acc_points = np.array(ACC_DATA)
        acc_x = np.array(ACC_X)
        acc_y = np.array(ACC_Y)
        acc_z = np.array(ACC_Z)

        if CYCLES < MAX_CYCLES:
            CYCLES += 1

        else:
            for i in range(len(ACC_DATA)):
                TIME_AXIS.append(i)

            # Show Plot
            #plot.plot(TIME_AXIS, acc_x, TIME_AXIS, acc_y, TIME_AXIS, acc_z)
            plot.plot(acc_x, color="r")
            plot.plot(acc_y, color="g")
            plot.plot(acc_z, color="b")

            plot.xlabel("Cycles")
            plot.ylabel("Acceleration (ms-2)")
            plot.grid()
            plot.legend(["X", "Y", "Z"])

            plot.show()
            break

