#  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#  TwelfthDoctor1's Arduino Plotting
#
#  > 2D Plotter
#
#  (C) Copyright TD1 & TWoCC 2022
#  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#  Licensed under MIT License
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
#
#  Codes from other parties are not part of the License and Copyright.
#  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#
import serial
import sys
import matplotlib.pyplot as plot
import numpy as np
import math
import time
from MasterApprenticeLib.TD1_Lib_MasterLogger import MasterLogger

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

# Logging
master_logger = MasterLogger(
    "2D Plotter",
    main_owner="TwelfthDoctor1",
    additional_context="2D Plotter Graph"
)

# NOTE!
# Before launching the code, check that you have specified the correct Serial Port and Baud Rate
# When launching the code, there is a chance that the code will fail due to incomplete serial data which breaks decoding
# As such, re-run the code until a non-Exception output comes up.

# Try to establish Serial Connection
try:
    print(f"Initiate Serial Connection at {SERIAL_PORT} at {BAUD_RATE} baud.")

    ard_serial = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE)

# Except if Serial is in use or no device is connected
except serial.SerialException as e:
    print(f"[ERROR]: PORT IS IN USE OR DEVICE IS NOT CONNECTED! STOP OTHER SERIAL PROCESSES AND TRY AGAIN.\n\n{e}")

    sys.exit()  # Stop code

# cycles = eval(input("Enter the number of cycles for data to be collected to then be plotted: "))

while True:
    # Decoding from bytes to string
    # Unstable Concurrently
    ser_ln = ard_serial.readline()[:-2].decode("utf-8")

    if ser_ln.startswith("-> PY"):
        # In ser_data, there are 8 items in the list.
        # Acceleration takes 1 to 3
        # Gyroscope takes 4 to 6
        # Temperature takes 7
        ser_data = ser_ln.split("|")
        # print(ser_data)

        # Acceleration of 3 Axis Combined
        acc_comb = math.sqrt(pow(float(ser_data[1]), 2) + pow(float(ser_data[2]), 2) + pow(float(ser_data[3]), 2))  # - 8

        # Print Out On Console
        print("================================================================================================")
        print(f"Cycle: {CYCLES}/{MAX_CYCLES}")
        print(f"[Acceleration] -> X: {ser_data[1]} | Y: {ser_data[2]} | Z: {ser_data[3]} | Combined: {acc_comb}")
        print(f"[Rotation] -> X: {ser_data[4]} | Y: {ser_data[5]} | Z: {ser_data[6]}")
        print(f"[Temperature] -> {ser_data[7]} degC")

        master_logger.log(
            f"[Cycles] -> {CYCLES}/{MAX_CYCLES}\n"
            f"[Acceleration] -> X: {ser_data[1]} | Y: {ser_data[2]} | Z: {ser_data[3]} | Combined: {acc_comb}\n"
            f"[Rotation] -> X: {ser_data[4]} | Y: {ser_data[5]} | Z: {ser_data[6]}\n"
            f"[Temperature] -> {ser_data[7]} degC"
        )

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
            print("================================================================================================")
            print(f"Experiment Cycle of {MAX_CYCLES} finished. Plotting Graph...")

            for i in range(len(ACC_DATA)):
                TIME_AXIS.append(i)

            # Plot and Show Graph
            #plot.plot(TIME_AXIS, acc_x, TIME_AXIS, acc_y, TIME_AXIS, acc_z)
            plot.plot(acc_points, color="orange")  # 3 axis Acceleration
            #plot.plot(acc_x, color="r")  # X Axis
            #plot.plot(acc_y, color="g")  # Y Axis
            #plot.plot(acc_z, color="b")  # Z Axis

            plot.xlabel("Cycles")
            plot.ylabel("Acceleration (ms-2)")
            plot.grid()
            plot.legend(["Combined", "X", "Y", "Z"])

            print(f"Please check graph for details. To save, access via the Python Graph.")

            plot.show()
            break

