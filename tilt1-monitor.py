# read tiltmeter (Arduino I/F to AD7747 cap meter)
# Arduino code derived from github.com/jankop2/Arduino-AD7747
# 30-Nov-2024 J.Beale

import serial
import time
from datetime import datetime
import os   # os.path.join()

logDir = r'C:\Users\beale\Documents\Tiltmeter'  # directory to store data output
# ---------------------------------------------------------

def readDat(ser):            
            inString = ser.readline().decode('utf-8').strip()
            inData = inString.split(",")
            cap = float(inData[0])
            degC = float(inData[1])
            return(cap, degC)

# ---------------------------------------------------------
# main program starts here

VERSION = "TiltMeter 0.1 30-Nov-2024"
now = datetime.now()
tsLog = now.strftime("%Y%m%d_%H%M%S_tilt.csv")

serport = 'COM5'  # serial port to talk to Arduino tiltmeter
logfile = os.path.join(logDir, tsLog)

ff = 0.05  # low-pass filter factor
dR = 4    # decimation ratio: print only Nth value from raw data stream
dLF = 500  # flush buffer to disk after this many lines written
i = 0      # reading counter
lc = 0     # line counter

cmd0 = b'XX'          # reset code and AD7747
cmd1 = b'PW0130\r\n'  # set polling repeat time at 130 msec
cmd2 = b'SS\r\n'      # start readout

with open(logfile, 'w') as of:
    print("%s Writing to %s" % (VERSION, logfile))
    of.write('epoch,pF,delta\n')

    try:
        ser = serial.Serial(serport, 115200, timeout=1)  # Replace 'COM5' with your port if different
        time.sleep(2)  # Give the port a moment to initialize

        ser.write(cmd0)  # send string to controller
        time.sleep(1)  
        inData = ser.readline().decode('utf-8').strip()
        inData = ser.readline().decode('utf-8').strip()
        ser.write(cmd1)  # send string to controller
        time.sleep(1)  
        inData = ser.readline().decode('utf-8').strip()
        ser.write(cmd2)  # send string to controller
        inData = ser.readline().decode('utf-8').strip()
        inData = ser.readline().decode('utf-8').strip()
        inData = ser.readline().decode('utf-8').strip() # "periodic sampling started"

        (cap, degC) = readDat(ser)
        lpC = cap  # initial value with no filter
        dMax = 0


        while True:
            (cap, degC) = readDat(ser)

            lpC = (1-ff)*lpC + ff*cap  # lowpass filter
            delta = cap - lpC  # difference between current and LP-filtered reading
            if (abs(delta) > abs(dMax)):
                 dMax = delta  # remember the largest deviation from current average
            i = i+1
            if (i >= dR):
                i = 0
                import time

                epochTms = time.time() * 1000 # time in milliseconds
                epochT = (epochTms / 1000.0)
                outLine = ("%.1f,%.5f,%.2f" % (epochT,lpC,dMax*1000.0))  # one line of output data
                print(outLine)  # capacitance in pF, after lowpass filter
                of.write("%s\n" % (outLine))  # capacitance in pF, after lowpass filter
                lc += 1
                if (lc >= dLF):
                     lc = 0
                     of.flush()  # write accumulated buffer to disk
                dMax = 0
            
        # ser.close()
        

    except serial.SerialException as e:
        print(f"An error occurred: {e}")
