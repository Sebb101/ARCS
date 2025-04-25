# For GUI 
import sys, math
from PyQt5 import QtWidgets, uic, QtGui, QtCore
from PyQt5.QtCore import QThread, pyqtSignal, QObject
import pyqtgraph as pg
from PyQt5.QtCore import QTimer, QTime
from random import randint
import os
import os.path
import sys
import serial
import serial.tools.list_ports
import time

# NI Hardware Library
import nidaqmx
from nidaqmx.constants import TerminalConfiguration, VoltageUnits, ThermocoupleType, AcquisitionType
from nidaqmx.stream_readers import AnalogMultiChannelReader
from nidaqmx import constants
from nidaqmx import stream_readers

# Data
import numpy as np
from queue import Queue
import datetime
import time

# Queue Variables
# Queue Variables
onboard_data_log = Queue()
onboard_data_disp = Queue()
command_Queue = Queue()

PT_q = Queue()
PT_q_disp = Queue()
PT_time_q =  Queue()

class Volt_Cur_readerWorker(QObject):
    volt_cur_reader_thread_finished = pyqtSignal() 
    volt_cur_reader_error = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.volt_cur_readerthreadActive = False
        self.loggingworkerActive = False

    def start_volt_cur_reader_task(self):
        print("Start of Voltage/Current Reader Worker")
        try:
            # connect to NI hardware
            pass

        except Exception as e:
            print(e)
            self.volt_cur_reader_error.emit() #Connection Error: Restart

        while(self.volt_cur_readerthreadActive):
            try:
                # read from NI hardware
                self.voltage_and_current_modules()

            except Exception as e:
                print(e)
                self.volt_cur_reader_error.emit() #Timeout Connection: Restart
        self.volt_cur_reader_thread_finished.emit()
        print("Ended Voltage/Current reading task")


    def voltage_and_current_modules(self): #need to group voltage and current modules togther due to limitations on tasks/chasis
        num_channels = 40
        fs_acq = 1000 #sample frequency
        with nidaqmx.Task() as task:
            task.ai_channels.add_ai_voltage_chan("cDAQ1Mod1/ai0:31", terminal_config=TerminalConfiguration.RSE, min_val=-10, max_val=10)
            task.ai_channels.add_ai_current_chan("cDAQ1Mod2/ai0:7")

            task.timing.cfg_samp_clk_timing(rate=fs_acq, sample_mode=constants.AcquisitionType.CONTINUOUS)


            reader = stream_readers.AnalogMultiChannelReader(task.in_stream)

            def reading_task_callback(task_idx, event_type, num_samples, callback_data=None):
                buffer = np.zeros((num_channels, 1000))
                reader.read_many_sample(buffer, num_samples, timeout=constants.WAIT_INFINITELY)

                # Convert the data from channel as a row order to channel as a column
                data = buffer.T
                final_time = time.time()

                # pass data to queues
                if not(self.loggingworkerActive):
                    # comment if fucks shit up
                    for i in range(0,10):
                        PT_q_disp.put(data[i])
                        time.sleep(0.001)
                else:
                    PT_q.put(data)
                    PT_time_q.put(final_time)
                    for i in range(0,10):
                        PT_q_disp.put(data[i])
                        time.sleep(0.001)
                    

                # Do something with the data
                return 0      # callback must return an integer, otherwise callback throws a wall of errors


            task.register_every_n_samples_acquired_into_buffer_event(1000, reading_task_callback)
            task.start()
            while(1):
                time.sleep(.001)
                if not(self.volt_cur_readerthreadActive):
                    break


class volt_cur_loggerWorker(QObject):
    volt_cur_logger_thread_finished = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.volt_cur_loggerthreadActive = False

    def start_volt_cur_logging_task(self):
        try:
            now = datetime.datetime.now()
            time_string = now.strftime("%m-%d-%Y_%H-%M-%S")+".csv"

            ##create fieldnames
            fieldnames = []
            fieldnames.append("Time")
            for i in range(32):
                fieldnames.append("cDAQ1Mod1/ai"+str(i))
            for i in range(8):
                fieldnames.append("cDAQ1Mod2/ai"+str(i))
            fieldnames_str = ','.join(fieldnames)

            index = 0

            while(self.volt_cur_loggerthreadActive):
                time.sleep(.01)
                with open("D:/jasev/Data_Dump/NIdaq_data/PT_"+time_string, 'ab') as csvfile:
                    if (PT_q.qsize() >0):
                        final_time = PT_time_q.get()                                 # get time data
                        time_array = np.linspace(final_time-1, final_time, num=1000) # make time array
                        PT_data = PT_q.get()                                         # get temp data
                        data = np.column_stack((time_array, PT_data))
                        if index==0:
                            np.savetxt(csvfile, data, delimiter=',', header = fieldnames_str )
                        else:
                            np.savetxt(csvfile, data, delimiter=',')
                        index = 1

            self.volt_cur_logger_thread_finished.emit()
            print("Ended Voltage/Current logging task")
        except Exception as e:
            print(e)
            self.volt_cur_logger_thread_finished.emit()
            print("Ended Voltage/Current logging task")

class readerWorker(QObject):
    reader_thread_finished = pyqtSignal() 
    #OBC_Status = pyqtSignal(bool)
    OBC_response = pyqtSignal(str) 
    reader_error = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.readerthreadActive = False
        self.loggingworkerActive = False
        self.DataRequest_Comm = "r"
        self.Outgoing_Comm = None
        self.MSGwaiting = False
        self.counter = 0

    def start_reader_task(self):
        print("Start of Reader Worker")

        # Maybe move to a conncet to OBC button???
        try:
            myports = [tuple(p) for p in list(serial.tools.list_ports.comports())]
            rs485port =myports[0][0]
            self.rs485_port_info = [port for port in myports if rs485port in port ][0]
            print("Using " + rs485port)

            constellation_comm = serial.Serial(port=rs485port, baudrate=19200, timeout=.1)
            time.sleep(1.75)

        except Exception as e:
            print(e)
            self.reader_error.emit() #Connection Error: Restart

        while(self.readerthreadActive):
            try:

                if(command_Queue.qsize() > 0 and self.counter < 130):
                    try:
                        self.Outgoing_Comm = command_Queue.get() # Get command and send to OBC
                        constellation_comm.write(bytes(self.Outgoing_Comm, 'utf-8'))
                        time.sleep(0.01)
                        msg = constellation_comm.readline().decode("utf-8")
                        msg = msg.strip('\n')
                        self.OBC_response.emit(msg) #Getting MSGs from OBC!
                        #self.OBC_Status.emit(True) #No MSGs
                    except Exception as e:
                        print(e)
                        #self.OBC_Status.emit(False) #No MSGs
                else:
                    if(self.counter > 200):
                        constellation_comm.write(bytes("r", 'utf-8'))
                        time.sleep(0.01)
                        self.counter = 0
                    self.counter+=1


                if(constellation_comm.inWaiting() > 0):
                    try:
                        # RS485 Data Parsing
                        msg = constellation_comm.readline()
                        msg = msg.decode("utf-8")
                        msg = msg.strip('\n')
                        #print(msg)

                        if(len(msg) > 0):
                            #self.OBC_Status.emit(True) #Getting MSGs from OBC!
                            data = msg
                            time_stamp = (datetime.datetime.now() - datetime.datetime(1970, 1, 1)).total_seconds()*1000
                            time_stamp = np.format_float_positional(time_stamp)
                            data = time_stamp+','+data
                            data = np.array(data.split(","))

                            #self.OBC_Status.emit(True) #No MSGs

                            if(not self.loggingworkerActive):
                                onboard_data_disp.put(data)
                            else:
                                vdata = np.hstack(data)
                                onboard_data_disp.put(data)
                                onboard_data_log.put(vdata)
                    except Exception as e:
                        print(e)
                        #self.OBC_Status.emit(False) #No MSGs

                    else:
                        #self.OBC_Status.emit(False) #No MSGs
                        pass

            except Exception as e:
                print(e)
                #self.OBC_Status.emit(False) #No MSGs
                self.reader_error.emit() #Timeout Connection: Restart

        self.reader_thread_finished.emit()
        #self.OBC_Status.emit(False) #No MSGs
        print("Ended reading task")

    def check_presence(self,correct_port):
        myports = [tuple(p) for p in list(serial.tools.list_ports.comports())]
        if correct_port not in myports:
            return False
        else:
            return True

class NIdisplayWorker(QObject):
    display_thread_finished = pyqtSignal()
    outgoing_PTdata = pyqtSignal(np.ndarray)
    display_error = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.displaythreadActive = False

    def start_display_task(self):
        try:
            while(self.displaythreadActive):

                if(PT_q_disp.qsize()>0):
                    self.ptdata =  PT_q_disp.get()

                    if(len(self.ptdata)<3):
                        pass
                    else:
                        #print(self.displayData )
                        self.outgoing_PTdata.emit(self.ptdata)
                else:
                    time.sleep(0.01)
                    pass


        except Exception as e:
            print(e)
            print("Display Error")
            self.display_error.emit() #Display Error: Restart
        self.display_thread_finished.emit()
        print("Ended display task")

class loggerWorker(QObject):
    logger_thread_finished = pyqtSignal()
    logger_error = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.loggerthreadActive = False

    def start_logging_task(self):
        try:
            csv_path = os.path.dirname(os.path.abspath(__file__)) # obtain file path
            self.save_path = os.path.join(csv_path, 'ARCS_Data/')
            #os.makedirs('OBC_Data/', exist_ok=True)
            os.makedirs(self.save_path, exist_ok=True)
            self.time_string = datetime.datetime.now().strftime("%m-%d-%Y_%H-%M-%S")+".csv"
            self.file_name = "ARCS_Data_"+self.time_string
            #self.save_path = "D:/jasev/Data_Dump/Onboards_Data/"
            #self.save_path = os.path.join(csv_path, 'OBC_Data/') # If you have a custom folder, rename
            self.completeFileName = os.path.join(self.save_path, self.file_name)
            header = 'UnixTime(ms),OnboardTime(s),Ax,Ay,Az,Gx,Gy,Gz'
            index = 0
            while(self.loggerthreadActive):
                with open(self.completeFileName, 'ab') as csvfile:

                    #get data array from the queue
                    if(onboard_data_log.qsize() > 2):
                        log_data = onboard_data_log.get()
                        log_data2 = onboard_data_log.get()
                        vvlog_data = np.vstack((log_data,log_data2))
                        if(index == 0):
                            np.savetxt(csvfile, vvlog_data, fmt='%s',delimiter=",",header = header)
                            index = 1
                        else:
                            np.savetxt(csvfile, vvlog_data, fmt='%s',delimiter=",",comments='')

                    if(not self.loggerthreadActive):
                        break
        except Exception as e:
            print(e)
            print("Logging Worker Error")
            self.logger_error.emit() #Timeout Connection: Restart

        self.logger_thread_finished.emit()
        print("Ended logging task")


class displayWorker(QObject):
    display_thread_finished = pyqtSignal()
    outgoing_data = pyqtSignal(np.ndarray)
    display_error = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.displaythreadActive = False

    def start_display_task(self):
        try:
            while(self.displaythreadActive):
                if(onboard_data_disp.qsize() > 0):
                    self.displayData = onboard_data_disp.get()
                    if(len(self.displayData)<6):
                        pass
                    else:
                        self.outgoing_data.emit(self.displayData)
                else:
                    time.sleep(0.01)
                    pass
        except Exception as e:
            print(e)
            print("Display Error")
            self.display_error.emit() #Display Error: Restart
        self.display_thread_finished.emit()
        print("Ended display task")
        
#Thruster Class
class RollThrustersWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)

        # For thruster actuation:
        self.roll_plus = False   # R+
        self.roll_minus = False  # R-
        self.yaw_plus = False    # Y+
        self.yaw_minus = False   # Y-
        self.pitch_plus = False  # P+
        self.pitch_minus = False # P-

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        margin = 20
        drawing_rect = self.rect().adjusted(margin, margin, -margin, -margin)
        size = min(drawing_rect.width(), drawing_rect.height())
        center = drawing_rect.center()

        # Draw the main circle (roll thrusters)
        circle_radius = size * 0.3  # Play around with this value
        pen = QtGui.QPen(QtCore.Qt.white, 3)
        painter.setPen(pen)
        painter.drawEllipse(center, circle_radius, circle_radius)

        # Calculate triangle dimensions and positions for roll thrusters
        triangle_height = circle_radius * 0.3  # Can be adjusted
        triangle_width  = circle_radius * 0.3

        left_x = center.x() - circle_radius
        right_x = center.x() + circle_radius
        top_offset = circle_radius * 0.65 
        bottom_offset = circle_radius * 0.65

        left_top = QtCore.QPointF(left_x, center.y() - top_offset)
        left_bottom = QtCore.QPointF(left_x, center.y() + bottom_offset)
        right_top = QtCore.QPointF(right_x, center.y() - top_offset)
        right_bottom = QtCore.QPointF(right_x, center.y() + bottom_offset)

        # Roll Thruster function
        def drawTriangle(centerPoint, orientation, fire=False):
            if orientation == "up":
                tip = QtCore.QPointF(centerPoint.x(), centerPoint.y() + triangle_height / 2)
                left_point = QtCore.QPointF(centerPoint.x() - triangle_width / 2, centerPoint.y() - triangle_height / 2)
                right_point = QtCore.QPointF(centerPoint.x() + triangle_width / 2, centerPoint.y() - triangle_height / 2)
            else:  # down
                tip = QtCore.QPointF(centerPoint.x(), centerPoint.y() - triangle_height / 2)
                left_point = QtCore.QPointF(centerPoint.x() - triangle_width / 2, centerPoint.y() + triangle_height / 2)
                right_point = QtCore.QPointF(centerPoint.x() + triangle_width / 2, centerPoint.y() + triangle_height / 2)
            points = [tip, right_point, left_point]
            polygon = QtGui.QPolygonF(points) #Function to connect points together
            painter.setBrush(QtGui.QBrush(QtCore.Qt.red) if fire else QtGui.QBrush(QtCore.Qt.NoBrush))
            painter.drawPolygon(polygon)
            painter.setBrush(QtGui.QBrush(QtCore.Qt.NoBrush))

        # Top Left R-
        drawTriangle(left_top, "up", fire=self.roll_minus)
        painter.drawText(left_top + QtCore.QPointF(-triangle_width, -triangle_height/2), "R-") #Text indicating the thrusters
        # Top right R+
        drawTriangle(right_top, "up", fire=self.roll_plus)
        painter.drawText(right_top + QtCore.QPointF(5, -triangle_height/2), "R+")
        # Bottom left R+
        drawTriangle(left_bottom, "down", fire=self.roll_plus)
        painter.drawText(left_bottom + QtCore.QPointF(-triangle_width, triangle_height), "R+")
        # Bottom right R-
        drawTriangle(right_bottom, "down", fire=self.roll_minus)
        painter.drawText(right_bottom + QtCore.QPointF(5, triangle_height), "R-")

        
        PY_triangle_height = circle_radius * 0.3
        PY_triangle_width = circle_radius * 0.3

        
        yaw_top = QtCore.QPointF(center.x(), center.y() - circle_radius - PY_triangle_height/2) #Center position of top thruster
        yaw_bottom = QtCore.QPointF(center.x(), center.y() + circle_radius + PY_triangle_height/2) #Center position of bottom thruster

        pitch_left = QtCore.QPointF(center.x() - circle_radius - PY_triangle_height/2, center.y()) #Center position of left thruster
        pitch_right = QtCore.QPointF(center.x() + circle_radius + PY_triangle_height/2, center.y()) #Center position of right thruster

        # Pitch and Yaw thruster function
        def drawRotatedTriangle(centerPoint, width, height, rotation_angle, fire=False):
            points = [QtCore.QPointF(0, -height/2), QtCore.QPointF(-width/2, height/2), QtCore.QPointF(width/2, height/2)] #0 degree triangle with tip at top
            polygon = QtGui.QPolygonF(points)
            transform = QtGui.QTransform()
            transform.translate(centerPoint.x(), centerPoint.y())
            transform.rotate(rotation_angle)
            polygon = transform.map(polygon)
            painter.setBrush(QtGui.QBrush(QtCore.Qt.red) if fire else QtGui.QBrush(QtCore.Qt.NoBrush))
            painter.drawPolygon(polygon)
            painter.setBrush(QtGui.QBrush(QtCore.Qt.NoBrush))

        #Angle must coorespond with the direction thruster will be facing based on the function original triangle
        #Top Y+
        drawRotatedTriangle(yaw_top, PY_triangle_width, PY_triangle_height, 180, fire=self.yaw_plus)
        painter.drawText(yaw_top + QtCore.QPointF(-10, -PY_triangle_height), "Y+")
        #Bottom Y-
        drawRotatedTriangle(yaw_bottom, PY_triangle_width, PY_triangle_height, 0, fire=self.yaw_minus)
        painter.drawText(yaw_bottom + QtCore.QPointF(-10, PY_triangle_height+10), "Y-")

        #Right P+
        drawRotatedTriangle(pitch_right, PY_triangle_width, PY_triangle_height, -90, fire=self.pitch_plus)
        painter.drawText(pitch_right + QtCore.QPointF(PY_triangle_width+5, 5), "P+")
        #Left P-
        drawRotatedTriangle(pitch_left, PY_triangle_width, PY_triangle_height, 90, fire=self.pitch_minus)
        painter.drawText(pitch_left + QtCore.QPointF(-PY_triangle_width-10, 5), "P-")

        painter.end()

# DIAL CLASS
class Dial(QtWidgets.QWidget):
    def __init__(self, label="Dial", min_val=0, max_val=100, initial_val=None, parent=None):
        super().__init__(parent)
        # Optional layout (not strictly needed unless you want to add sub-widgets)
        self.layout = QtWidgets.QGridLayout()
        self.setLayout(self.layout)
        self.setMinimumSize(150, 150)  # Have a minimum size

        self.label = label       # Label for the dial
        self.val = initial_val          # Current value of the dial
        self.min_val = min_val         # Minimum dial value
        self.max_val = max_val      # Maximum dial value

        self.val = initial_val if initial_val is not None else min_val

    def setValue(self, value):
        #"""Update the dial's value and trigger a redraw."""
        self.val = value
        self.update()

    def paintEvent(self, event):
        paint = QtGui.QPainter(self)
        paint.setRenderHint(QtGui.QPainter.Antialiasing)  # Smooth drawing

        fm = paint.fontMetrics()  # Font metrics for text size
        margin = fm.height() + 10  # Margin to prevent text clipping
        rectangle = self.rect().adjusted(margin, margin, -margin, -margin)
        # Use the full widget bounds so that the dial fills its container
        #rectangle = self.rect()
        size = min(rectangle.width(), rectangle.height())
        middle = rectangle.center()
        radius = size * 0.5  # Dial radius is 40% of smallest side

        # Draw dial arc
        dial_rect = QtCore.QRectF(middle.x() - radius, middle.y() - radius, 2 * radius, 2 * radius)
        pen = QtGui.QPen(QtCore.Qt.gray, 5)
        paint.setPen(pen)
        paint.drawArc(dial_rect, 225 * 16, -270 * 16) 

        # Draw tick marks
        tick_interval = (self.max_val - self.min_val) / 10
        tick_marks = int(self.max_val // tick_interval) + 1
        tick_pen = QtGui.QPen(QtCore.Qt.gray, 3)
        paint.setPen(tick_pen)

        for i in range(tick_marks):
            angle = 225 - (270 / (tick_marks - 1)) * i
            angle_rad = math.radians(angle)
            tick_low = 0.8 * radius
            tick_high = 0.9 * radius

            start = QtCore.QPointF(middle.x() + tick_low * math.cos(angle_rad), middle.y() - tick_low * math.sin(angle_rad))
            end = QtCore.QPointF(middle.x() + tick_high * math.cos(angle_rad), middle.y() - tick_high * math.sin(angle_rad))
            paint.drawLine(start, end)

            # Draw number on every 5th tick
            if i % 5 == 0 or i == tick_marks - 1:
                numerical_tick = self.min_val + (i / (tick_marks - 1)) * (self.max_val - self.min_val)
                text = f"{int(numerical_tick)}"
                font = paint.font()
                font.setBold(False)
                paint.setFont(font)

                text_radius = 1.2 * radius
                text_placement = QtCore.QPointF(middle.x() + text_radius * math.cos(angle_rad), middle.y() - text_radius * math.sin(angle_rad))
                text_rect = fm.boundingRect(text)
                text_rect.moveCenter(text_placement.toPoint())
                paint.drawText(text_rect, QtCore.Qt.AlignCenter, text)

        # Draw the needle
        if self.max_val != self.min_val:
            proportion = (self.val - self.min_val) / (self.max_val - self.min_val)
        else:
            proportion = 0
        needle_angle = 225 - (270 * proportion)
        needle_rad = math.radians(needle_angle)
        needle_length = radius * 0.75
        end_needle = QtCore.QPointF(middle.x() + needle_length * math.cos(needle_rad), middle.y() - needle_length * math.sin(needle_rad))
        needle_pen = QtGui.QPen(QtCore.Qt.red, 4)
        paint.setPen(needle_pen)
        paint.drawLine(middle, end_needle)

        # Draw center circle
        paint.setBrush(QtCore.Qt.black)
        paint.drawEllipse(middle, 5, 5)

        # Draw the label at the bottom center
        paint.setPen(QtGui.QPen(QtCore.Qt.white))
        font = paint.font()
        font.setPointSize(12)
        paint.setFont(font)
        paint.drawText(rectangle, QtCore.Qt.AlignBottom | QtCore.Qt.AlignHCenter, self.label)

        paint.end()




class Ui(QtWidgets.QMainWindow):
    def __init__(self):
        super(Ui, self).__init__() # Call the inherited classes __init__ method
        ui_path = os.path.dirname(os.path.abspath(__file__)) # obtain file path
        uic.loadUi(os.path.join(ui_path, 'PSST_MainGUI_V2.ui'), self)
        self.Logo.setPixmap(QtGui.QPixmap(os.path.join(ui_path, 'ARCS_MissionPatch.png')))
        self.show() # Show the GUI

        # Data values

        self.sizeOfline = 20
        self.time = list(range(self.sizeOfline))

        self.gx = [randint(0, 1) for _ in range(self.sizeOfline)]
        self.gy = [randint(0, 1) for _ in range(self.sizeOfline)]
        self.gz = [randint(0, 1) for _ in range(self.sizeOfline)]

        self.copv = [randint(0, 1) for _ in range(self.sizeOfline)]
        self.highpress = [randint(0, 1) for _ in range(self.sizeOfline)]
        self.lowpress = [randint(0, 1) for _ in range(self.sizeOfline)]

        self.omegaXDial = Dial(label="Omega_x", min_val=-100, max_val=100, parent=self.groupBox_omega_x_dial)
        self.verticalLayout_omega_x_dial.addWidget(self.omegaXDial)
        self.horizontalLayout_angularDials.addWidget(self.groupBox_omega_x_dial)

        self.omegaYDial = Dial(label="Omega_y", min_val=-100, max_val=100, parent=self.groupBox_omega_y_dial)
        self.verticalLayout_omega_y_dial.addWidget(self.omegaYDial)
        self.horizontalLayout_angularDials.addWidget(self.groupBox_omega_y_dial)

        self.omegaZDial = Dial(label="Omega_z", min_val=-100, max_val=100, parent=self.groupBox_omega_z_dial)
        self.verticalLayout_omega_z_dial.addWidget(self.omegaZDial)
        self.horizontalLayout_angularDials.addWidget(self.groupBox_omega_z_dial)

        self.thetaDial = Dial(label="Theta", min_val=-180, max_val=180, parent=self.groupBox_theta_dial)
        self.verticalLayout_theta_dial.addWidget(self.thetaDial)
        self.horizontalLayout_anglesDials.addWidget(self.groupBox_theta_dial)

        self.phiDial = Dial(label="Phi", min_val=-180, max_val=180, parent=self.groupBox_phi_dial)
        self.verticalLayout_phi_dial.addWidget(self.phiDial)
        self.horizontalLayout_anglesDials.addWidget(self.groupBox_phi_dial)

        self.psiDial = Dial(label="Psi", min_val=-180, max_val=180, parent=self.groupBox_psi_dial)
        self.verticalLayout_psi_dial.addWidget(self.psiDial)
        self.horizontalLayout_anglesDials.addWidget(self.groupBox_psi_dial)

        self.verticalLayout_roll = QtWidgets.QVBoxLayout(self.group_box_display_panel)
        self.rollWidget = RollThrustersWidget()
        self.verticalLayout_roll.addWidget(self.rollWidget)
        self.verticalLayout_center.addWidget(self.group_box_display_panel)
        self.horizontalLayout_main.addLayout(self.verticalLayout_center)

        self.thrusterButtonLayout = QtWidgets.QGridLayout()
        self.thrusterButtonLayout.setObjectName("thrusterButtonLayout")
        
        self.rollPlusButton = QtWidgets.QPushButton("Roll+")
        self.rollPlusButton.setStyleSheet("background-color: white; color: black;")
        self.rollMinusButton = QtWidgets.QPushButton("Roll-")
        self.rollMinusButton.setStyleSheet("background-color: white; color: black;")
        self.yawPlusButton = QtWidgets.QPushButton("Yaw+")
        self.yawPlusButton.setStyleSheet("background-color: white; color: black;")
        self.yawMinusButton = QtWidgets.QPushButton("Yaw-")
        self.yawMinusButton.setStyleSheet("background-color: white; color: black;")
        self.pitchPlusButton = QtWidgets.QPushButton("Pitch+")
        self.pitchPlusButton.setStyleSheet("background-color: white; color: black;")
        self.pitchMinusButton = QtWidgets.QPushButton("Pitch-")
        self.pitchMinusButton.setStyleSheet("background-color: white; color: black;")

        self.thrusterButtonLayout.addWidget(self.rollPlusButton, 0, 0)
        self.thrusterButtonLayout.addWidget(self.rollMinusButton, 0, 1)
        self.thrusterButtonLayout.addWidget(self.yawPlusButton, 1, 0)
        self.thrusterButtonLayout.addWidget(self.yawMinusButton, 1, 1)
        self.thrusterButtonLayout.addWidget(self.pitchPlusButton, 2, 0)
        self.thrusterButtonLayout.addWidget(self.pitchMinusButton, 2, 1)
        
        self.verticalLayout_center.addLayout(self.thrusterButtonLayout)

        self.rollPlusButton.pressed.connect(lambda: self.set_thruster_state("roll_plus", True))
        self.rollPlusButton.pressed.connect(lambda: self.on_press("a"))
        self.rollPlusButton.released.connect(lambda: self.set_thruster_state("roll_plus", False))
        self.rollPlusButton.released.connect(lambda: self.on_release())

        self.rollMinusButton.pressed.connect(lambda: self.set_thruster_state("roll_minus", True))
        self.rollMinusButton.pressed.connect(lambda: self.on_press("d"))
        self.rollMinusButton.released.connect(lambda: self.set_thruster_state("roll_minus", False))
        self.rollMinusButton.released.connect(lambda: self.on_release())


        self.yawPlusButton.pressed.connect(lambda: self.set_thruster_state("yaw_plus", True))
        self.yawPlusButton.pressed.connect(lambda: self.on_press("q"))
        self.yawPlusButton.released.connect(lambda: self.set_thruster_state("yaw_plus", False))
        self.yawPlusButton.released.connect(lambda: self.on_release())


        self.yawMinusButton.pressed.connect(lambda: self.set_thruster_state("yaw_minus", True))
        self.yawMinusButton.pressed.connect(lambda: self.on_press("e"))
        self.yawMinusButton.released.connect(lambda: self.set_thruster_state("yaw_minus", False))
        self.yawPlusButton.released.connect(lambda: self.on_release())

        self.pitchPlusButton.pressed.connect(lambda: self.set_thruster_state("pitch_plus", True))
        self.pitchPlusButton.pressed.connect(lambda: self.on_press("w"))
        self.pitchPlusButton.released.connect(lambda: self.set_thruster_state("pitch_plus", False))
        self.pitchPlusButton.released.connect(lambda: self.on_release())


        self.pitchMinusButton.pressed.connect(lambda: self.set_thruster_state("pitch_minus", True))
        self.pitchMinusButton.pressed.connect(lambda: self.on_press("s"))
        self.pitchMinusButton.released.connect(lambda: self.set_thruster_state("pitch_minus", False))
        self.pitchMinusButton.released.connect(lambda: self.on_release())
        

        self.copvDial = Dial(label="COPV", min_val=0, max_val=2500, parent=self.groupBox_COPV_dial)
        self.verticalLayout_COPV_dial.addWidget(self.copvDial)
        self.horizontalLayout_pressureDials.addWidget(self.groupBox_COPV_dial)

        self.hpDial = Dial(label="HP", min_val=0, max_val=2000,parent=self.groupBox_HP_dial)
        self.verticalLayout_HP_dial.addWidget(self.hpDial)
        self.horizontalLayout_pressureDials.addWidget(self.groupBox_HP_dial)

        self.lpDial = Dial(label="LP", min_val=0, max_val=100, parent=self.groupBox_LP_dial)
        self.verticalLayout_LP_dial.addWidget(self.lpDial)
        self.horizontalLayout_pressureDials.addWidget(self.groupBox_LP_dial)

        #Buttons:
        self.start_button.setStyleSheet("QPushButton" "{" "background-color:  red; color: black;" "}" "QPushButton::hover""{""background-color : lightgreen;""}")
        self.start_NIbutton.setStyleSheet("QPushButton" "{" "background-color:  red; color: black;" "}" "QPushButton::hover""{""background-color : lightgreen;""}")
        self.start_data_button.setStyleSheet("QPushButton" "{" "background-color:  red; color: black;" "}" "QPushButton::hover""{""background-color : lightgreen;""}")
        self.log_button.setStyleSheet("QPushButton" "{" "background-color:  red; color: black;" "}" "QPushButton::hover""{""background-color : lightgreen;""}")
        self.log_NIbutton.setStyleSheet("QPushButton" "{" "background-color:  red; color: black;" "}" "QPushButton::hover""{""background-color : lightgreen;""}")
        self.stop_button.setStyleSheet("QPushButton" "{" "background-color:  red; color: black;" "}" "QPushButton::hover""{""background-color : lightgreen;""}")
        self.stop_NIbutton.setStyleSheet("QPushButton" "{" "background-color:  red; color: black;" "}" "QPushButton::hover""{""background-color : lightgreen;""}")

        self.start_button.pressed.connect(self.start_buttion_action)
        self.start_NIbutton.pressed.connect(self.start_NIbutton_action)
        self.stop_button.pressed.connect(self.stop_buttion_action)
        self.stop_NIbutton.pressed.connect(self.stop_NIbutton_action)

        self.start_data_button.pressed.connect(self.control_buttonAction)

        self.log_button.pressed.connect(self.log_buttion_action)
        self.log_NIbutton.pressed.connect(self.log_NIbutton_action)

        # Logging Variable
        self.loggingOBCState = False
        self.readingState = False
        self.controlLoopState = False

        #NILogging
        self.NIloggingState = False
        self.NIreadingState = False

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_GUI)

        self.button_timer = QtCore.QTimer()
        self.button_timer.timeout.connect(self.on_timeout)

    def on_timeout(self):
        command_Queue.put(self.OBCcommand)

    def on_press(self,msg):
        self.button_timer.start(150)
        self.OBCcommand = msg

    def on_release(self):
        self.button_timer.stop()
        self.OBCcommand = " "

    def set_thruster_state(self, thruster, state):
        setattr(self.rollWidget, thruster, state)
        self.rollWidget.update()



    def incoming_data(self,incoming_data):
        self.time = self.time[1:]
        self.currentTime = int((datetime.datetime.now() - datetime.datetime(1970, 1, 1)).total_seconds()*1000) - self.timeZero
        self.time.append(self.currentTime) # converts from ms to s & appends new time to array

        print(incoming_data[7])
        self.gx = self.gx[1:]
        self.gy = self.gy[1:]
        self.gz = self.gz[1:]

        self.gx.append(float(incoming_data[5])) # Angular Rate X
        self.gy.append(float(incoming_data[6])) # Angular Rate Y
        self.gz.append(float(incoming_data[7])) # Angular Rate Z

    def incoming_NIdata(self,incoming_data):
        self.time = self.time[1:]
        self.currentTime = int((datetime.datetime.now() - datetime.datetime(1970, 1, 1)).total_seconds()*1000) - self.timeZero
        self.time.append(self.currentTime) # converts from ms to s & appends new time to array

        #print(incoming_data[7])

        self.copv = self.copv[1:]
        self.lowpress = self.lowpress[1:]
        self.highpress = self.highpress[1:]

        self.copv.append(float(incoming_data[0])*250) # COPV PT (0-2500) Ain0
        self.lowpress.append(float(incoming_data[1])*200) # HP PT (0-2000) Ain1
        self.highpress.append(float(incoming_data[2])*50) # LP (0-500) Ain2

        self.COPVLabel2.setText("{0:0.1f} psi".format(self.copv[-1]))
        self.HPLabel2.setText("{0:0.1f} psi".format(self.highpress[-1]))
        self.LPLabel2.setText("{0:0.1f} psi".format(self.lowpress[-1]))


    def update_GUI(self):
        if self.readingState:
            self.omegaXDial.setValue(self.gx[-1])
            self.omegaYDial.setValue(self.gy[-1])
            self.omegaZDial.setValue(self.gz[-1])

        if self.NIreadingState:
            self.copvDial.setValue(self.copv[-1])
            self.lpDial.setValue(self.lowpress[-1])
            self.hpDial.setValue(self.highpress[-1])

    def control_buttonAction(self):
            if(self.controlLoopState):
                self.controlLoopState = False
            else:
                self.controlLoopState = True

            if(self.readingState and self.controlLoopState):
                command_Queue.put("c")
            if(self.readingState and not self.controlLoopState):
                command_Queue.put("x")

    def start_NIbutton_action(self):
        
        if(not self.NIreadingState):
            self.timeZero = int((datetime.datetime.now() - datetime.datetime(1970, 1, 1)).total_seconds()*1000)
            self.start_NIbutton.setStyleSheet("QPushButton" "{" "background-color: rgb(0, 255, 0);" "}" 
                                            "QPushButton::hover""{""background-color : lightgreen;""}")
            
            # Create Reader and Display threads and Workers 
            # Create Thread and worker objects
            self.volt_cur_readingthread = QThread()
            self.NIdisplaythread = QThread() 

            self.volt_cur_reader_worker = Volt_Cur_readerWorker()
            self.NIdisplay_worker = NIdisplayWorker()

            # Step 4: Move worker to the thread
            self.volt_cur_reader_worker.moveToThread(self.volt_cur_readingthread)
            self.NIdisplay_worker.moveToThread(self.NIdisplaythread)

            # Set control variables to true
            self.volt_cur_reader_worker.volt_cur_readerthreadActive = True
            self.volt_cur_reader_worker.loggingworkerActive = False

            self.NIdisplay_worker.displaythreadActive = True
            self.NIreadingState = True

            # Start the threads
            self.volt_cur_readingthread.start()
            self.NIdisplaythread.start()
            

            #Connect signals and slots
            # Voltage and Current Module Reader Thread
            self.volt_cur_readingthread.started.connect(self.volt_cur_reader_worker.start_volt_cur_reader_task)
            self.volt_cur_reader_worker.volt_cur_reader_thread_finished.connect(self.volt_cur_readingthread.quit)
            self.volt_cur_reader_worker.volt_cur_reader_thread_finished.connect(self.volt_cur_reader_worker.deleteLater)
            self.volt_cur_readingthread.finished.connect(self.volt_cur_readingthread.deleteLater)
            #self.volt_cur_readingthread.finished.connect(lambda: self.error_terminal.append("> Voltage & Current Reading Thread Closed"))

            # Display Thread
            self.NIdisplaythread.started.connect(self.NIdisplay_worker.start_display_task)
            self.NIdisplay_worker.display_thread_finished.connect(self.NIdisplaythread.quit)
            self.NIdisplay_worker.display_thread_finished.connect(self.NIdisplay_worker.deleteLater)
            self.NIdisplaythread.finished.connect(self.NIdisplaythread.deleteLater)

            self.NIdisplay_worker.outgoing_PTdata.connect(self.incoming_NIdata)

            #self.NIdisplaythread.finished.connect(lambda: self.error_terminal.append("> Display Thread Closed"))
            
            self.timer.start(100)

            # If error is present show
            #self.volt_cur_reader_worker.volt_cur_reader_error.connect(lambda: self.error_terminal.append("> PT Reader Worker Error"))
            self.volt_cur_reader_worker.volt_cur_reader_error.connect(lambda: self.stop_NIbutton_action())

            # Individual responses from OBC are displayed on the OBC response terminal

            #self.display_worker.display_error.connect(lambda: self.error_terminal.append("> Display Worker Error"))
            #self.display_worker.display_error.connect(lambda: self.stop_buttion_action())
            print("> PT Reading Thread Started")
            print("> Display Thread Started")
            

            
        else:
            print("> Already reading...")


    def start_buttion_action(self):
        #self.status_terminal.append("> Start Button Pressed 😊 ")

        if(not self.readingState):
            self.timeZero = int((datetime.datetime.now() - datetime.datetime(1970, 1, 1)).total_seconds()*1000)
            self.start_button.setStyleSheet("QPushButton" "{" "background-color: rgb(0, 255, 0);" "}" 
                                            "QPushButton::hover""{""background-color : lightgreen;""}")
            
            # Create Reader and Display threads and Workers 
            # Create Thread and worker objects
            self.readingthread = QThread()
            self.displaythread = QThread() 

            self.reader_worker = readerWorker()
            self.display_worker = displayWorker()

            # Step 4: Move worker to the thread
            self.reader_worker.moveToThread(self.readingthread)
            self.display_worker.moveToThread(self.displaythread)
        

            # Set control variables to true
            self.reader_worker.readerthreadActive = True
            self.reader_worker.loggingworkerActive = False
            self.display_worker.displaythreadActive = True
            self.readingState = True

            # Start the threads
            self.readingthread.start()
            self.displaythread.start()
            

            #Connect signals and slots

            # Reader Thread
            self.readingthread.started.connect(self.reader_worker.start_reader_task)
            self.reader_worker.reader_thread_finished.connect(self.readingthread.quit)
            self.reader_worker.reader_thread_finished.connect(self.reader_worker.deleteLater)
            self.readingthread.finished.connect(self.readingthread.deleteLater)
            #self.readingthread.finished.connect(lambda: self.error_terminal.append("> Reading Thread Closed"))

            # Display Thread
            self.displaythread.started.connect(self.display_worker.start_display_task)
            self.display_worker.display_thread_finished.connect(self.displaythread.quit)
            self.display_worker.display_thread_finished.connect(self.display_worker.deleteLater)
            self.displaythread.finished.connect(self.displaythread.deleteLater)
            self.display_worker.outgoing_data.connect(self.incoming_data)
            #self.displaythread.finished.connect(lambda: self.error_terminal.append("> Display Thread Closed"))
            
            self.timer.start(150)

            # If error is present show
            #self.reader_worker.reader_error.connect(lambda: self.error_terminal.append("> Reader Worker Error"))
            self.reader_worker.reader_error.connect(lambda: self.stop_buttion_action())
            #self.reader_worker.OBC_Status.connect(self.OBCStatus_Action)

            # Individual responses from OBC are displayed on the OBC response terminal

            #self.reader_worker.OBC_response.connect(self.OBCresponse_Action)

            #self.display_worker.display_error.connect(lambda: self.error_terminal.append("> Display Worker Error"))
            self.display_worker.display_error.connect(lambda: self.stop_buttion_action())

            #self.error_terminal.append("> Reading Thread Started")
            #self.error_terminal.append("> Display Thread Started")
            

            
        else:
            #self.status_terminal.append("> Already reading...")
            pass

        
    def log_buttion_action(self):

        if(self.readingState):
            if(not self.loggingOBCState):
                #print("logging!")
                #self.status_terminal.append("> Started Logging")

                self.loggingthread = QThread()
                self.logger_worker = loggerWorker()

                self.logger_worker.moveToThread(self.loggingthread) 

                self.logger_worker.loggerthreadActive = True
                self.reader_worker.loggingworkerActive = True
                self.loggingOBCState = True

                self.loggingthread.start()

                self.loggingthread.started.connect(self.logger_worker.start_logging_task)
                #self.loggingthread.started.connect(lambda: self.error_terminal.append("> Logging Thread Started"))
                self.logger_worker.logger_thread_finished.connect(self.loggingthread.quit)
                self.logger_worker.logger_thread_finished.connect(self.logger_worker.deleteLater)
                self.loggingthread.finished.connect(self.loggingthread.deleteLater)

                #self.loggingthread.finished.connect(lambda: self.error_terminal.append("> Logging Thread Closed"))

                #self.logger_worker.logger_error.connect(lambda: self.error_terminal.append("> Logging Worker Error"))

                self.log_button.setStyleSheet("QPushButton" "{" "background-color: rgb(0, 255, 0);" "}" 
                                            "QPushButton::hover""{""background-color : lightgreen;""}")
                
                #self.error_terminal.append("> Logging Thread Started")
                
            elif(self.loggingOBCState):
                self.logger_worker.loggerthreadActive = False
                self.reader_worker.loggingworkerActive = False
                self.loggingOBCState = False
                #print("Stopped logging!")
                #self.status_terminal.append("> Stopped Logging")
                self.log_button.setStyleSheet("QPushButton" "{" "background-color: rgb(255, 0, 0);" "}" 
                                            "QPushButton::hover""{""background-color : lightgreen;""}")

        else:
            self.status_terminal.append("> Must be reading first before logging")
            
    def log_NIbutton_action(self):

        if(self.NIreadingState):
            if(not self.NIloggingState):
                #print("logging!")
                print("> Started Logging")
                self.volt_cur_reader_worker.loggingworkerActive = True
                self.NIloggingState = True


                # PT logging thread variables
                self.volt_cur_loggingthread = QThread()
                self.volt_cur_logger_worker = volt_cur_loggerWorker()

                self.volt_cur_logger_worker.moveToThread(self.volt_cur_loggingthread) 

                self.volt_cur_logger_worker.volt_cur_loggerthreadActive = True

                self.volt_cur_loggingthread.start()

                self.volt_cur_loggingthread.started.connect(self.volt_cur_logger_worker.start_volt_cur_logging_task)
                self.volt_cur_logger_worker.volt_cur_logger_thread_finished.connect(self.volt_cur_loggingthread.quit)
                self.volt_cur_logger_worker.volt_cur_logger_thread_finished.connect(self.volt_cur_logger_worker.deleteLater)
                self.volt_cur_loggingthread.finished.connect(self.volt_cur_loggingthread.deleteLater)

                self.volt_cur_loggingthread.finished.connect(lambda: self.error_terminal.append("> TC Logging Thread Closed"))


                self.log_NIbutton.setStyleSheet("QPushButton" "{" "background-color: rgb(0, 255, 0);" "}" 
                                            "QPushButton::hover""{""background-color : lightgreen;""}")
                
                print("> Logging Thread Started")
                
            elif(self.NIloggingState):
                self.volt_cur_logger_worker.volt_cur_loggerthreadActive = False
                self.volt_cur_reader_worker.loggingworkerActive = False
                self.NIloggingState = False
                #print("Stopped logging!")
                print("> Stopped Logging")
                self.log_NIbutton.setStyleSheet("QPushButton" "{" "background-color: rgb(255, 0, 0);" "}" 
                                            "QPushButton::hover""{""background-color : lightgreen;""}")

        else:
            print("> Must be reading first before logging")
        
    def stop_buttion_action(self):
        #self.status_terminal.append("> Stop button pressed!")
        if(self.readingState):
            self.timer.stop()
            self.reader_worker.readerthreadActive = False
            self.reader_worker.loggingworkerActive = False
            self.readingState = False
            self.display_worker.displaythreadActive = False

            #self.OBCstatusLabel.setText("OFF   ")
            #self.OBCstatusLabel.setStyleSheet("background-color: rgb(255, 0, 0); color: rgb(0, 0, 0); font: 8pt Verdana;")

            if(self.loggingOBCState):
                self.logger_worker.loggerthreadActive = False
                self.loggingOBCState = False

            self.start_button.setStyleSheet("QPushButton" "{" "background-color: rgb(255, 0, 0);" "}" 
                                         "QPushButton::hover""{""background-color : lightgreen;""}")
            self.log_button.setStyleSheet("QPushButton" "{" "background-color: rgb(255, 0, 0);" "}" 
                                         "QPushButton::hover""{""background-color : lightgreen;""}")
        else:
            #self.error_terminal.append("> No Threads to stop")
            print("No threads to stop")

    def stop_NIbutton_action(self):
        print("> Stop button pressed!")
        if(self.NIreadingState):
            self.timer.stop()
            self.volt_cur_reader_worker.volt_cur_readerthreadActive = False
            self.volt_cur_reader_worker.loggingworkerActive = False
            self.NIreadingState = False
            self.NIdisplay_worker.displaythreadActive = False


            if(self.NIloggingState):
                self.volt_cur_logger_worker.volt_cur_loggerthreadActive = False
                self.NIloggingState = False

            self.start_NIbutton.setStyleSheet("QPushButton" "{" "background-color: rgb(255, 0, 0);" "}" 
                                         "QPushButton::hover""{""background-color : lightgreen;""}")
            self.log_NIbutton.setStyleSheet("QPushButton" "{" "background-color: rgb(255, 0, 0);" "}" 
                                         "QPushButton::hover""{""background-color : lightgreen;""}")
        else:
            print("> No Threads to stop")

if __name__ == "__main__":
    QtWidgets.QApplication.setAttribute(QtCore.Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    app = QtWidgets.QApplication(sys.argv)
    window = Ui()   
    sys.exit(app.exec_())