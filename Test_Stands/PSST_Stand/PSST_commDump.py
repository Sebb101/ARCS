# For Arduino-Python Communcation over Serial
import serial
import serial.tools.list_ports
import time
from queue import Queue
import numpy as np

command_q = Queue()

try:
    myports = [tuple(p) for p in list(serial.tools.list_ports.comports())]
    USBport =myports[0][0]
    USB_port_info = [port for port in myports if USBport in port ][0]
    print("Using " + USBport)

    SPARCS_comm = serial.Serial(port=USBport, baudrate=115200, timeout=.1)
    time.sleep(1.75)

    #command_q.put("<r>")

    while(1):

        # if(SPARCS_comm.inWaiting() < 1):
        #     outgoing = input("Send OBC Command: ")
        #     #command_q.put(outgoing)
        #     SPARCS_comm.write(bytes(outgoing, 'utf-8'))
        #     #constellation_comm.write(bytes("<r>", 'utf-8'))
        #     #constellation_comm.write(b"<r>")
        #     time.sleep(0.05)

        while(SPARCS_comm.inWaiting() > 0):
            bytes2read = SPARCS_comm.inWaiting()
            #msg = constellation_comm.readline()
            msg = SPARCS_comm.readline(bytes2read).decode("utf-8")
            msg = msg.strip("\n")
            #data = msg.split(",")
            print(msg)

            # for i in range(len(data)):
            #     data[i] = (float)*data[i]
            
            # print(f"{data[0]:.2f},{data[1]:.2f},{data[2]:.2f},{data[3]:.0f},{data[4]:.0f}")

except Exception as e:
    print(e)
    quit()