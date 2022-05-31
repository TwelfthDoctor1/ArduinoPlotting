import serial
import sys
import matplotlib.pyplot as plot
import numpy as np
import math
import time

# Data Housing
ACC_DATA = []
ACC_X = []
ACC_Y = []
ACC_Z = []
TIME_AXIS = []

# Cycles
# Set Max Cycles to a specified number, e.g. if you want 10 cycles, put 10
CYCLES = 0  # <- Cycle Count, DO NOT TOUCH VALUE
MAX_CYCLES = 50  # <- Set the number of cycles for data to be collected

# Serial Config
# Serial Port is name of the Serial port, check VSCode or Arduino IDE for connected port
# Baud Rate is the Serial Baud Rate used in Arduino
SERIAL_PORT = "/dev/cu.usbserial-0001"
BAUD_RATE = 9600

# NOTE!
# Before launching the code, check that you have specified the correct Serial Port and Baud Rate
# When launching the code, there is a chance that the code will fail due to incomplete serial data which breaks decoding
# As such, re-run the code until a non-Exception output comes up.

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

        # Data Append to form List for Arrays
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

            # Plot and Show Graph
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

