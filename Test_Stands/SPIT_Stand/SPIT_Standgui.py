# For GUI 
from PyQt5 import QtWidgets, uic, QtGui, QtCore
from PyQt5.QtCore import QThread, pyqtSignal, QObject
import pyqtgraph as pg
from random import randint
import os
import os.path
import sys

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
LC_q = Queue()
LC_q_disp = Queue()

LC_time_q =  Queue()



class LCreaderWorker(QObject):
    lcreader_thread_finished = pyqtSignal() 
    lcreader_error = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.lcreaderthreadActive = False
        self.loggingworkerActive = False

    def start_lcreader_task(self):
        print("Start of LC Reader Worker")
        try:
            # connect to NI hardware
            pass

        except Exception as e:
            print(e)
            self.lcreader_error.emit() #Connection Error: Restart

        while(self.lcreaderthreadActive):
            try:
                # read from NI hardware
                self.loadcell_module()

            except Exception as e:
                print(e)
                self.lcreader_error.emit() #Timeout Connection: Restart
        self.lcreader_thread_finished.emit()
        print("Ended LC reading task")


    def loadcell_module(self):
        num_channels = 4
        fs_acq = 1000 #sample frequency
        with nidaqmx.Task() as task:

            task.ai_channels.add_ai_force_bridge_table_chan(
                physical_channel="cDAQ3Mod1/ai0:3",
                min_val=0,
                max_val=11.2769,
                voltage_excit_val=10,
                nominal_bridge_resistance=700,
                electrical_vals=[0.286, 0.387, 0.495, 0.574, 0.703, 0.804,0.887,1.058,1.327],
                physical_vals= [0,1.1, 2.2795, 3.1931, 4.5398, 5.642,6.5425,8.4106,11.2769])

            task.timing.cfg_samp_clk_timing(rate=fs_acq, sample_mode=constants.AcquisitionType.CONTINUOUS)
            reader = stream_readers.AnalogMultiChannelReader(task.in_stream)

            def reading_task_callback_LC(task_idx, event_type, num_samples, callback_data=None):
                initial_time = time.time()
                buffer = np.zeros((num_channels, 1000))
                reader.read_many_sample(buffer, num_samples, timeout=constants.WAIT_INFINITELY)
                # Convert the data from channel as a row order to channel as a column
                data = buffer.T
                disp_data = data

                if not(self.loggingworkerActive):
                    for i in range(0,10):
                        LC_q_disp.put(disp_data[i])
                        time.sleep(0.01)
                else:
                    LC_q.put(data)
                    LC_time_q.put(initial_time)
                    
                    LC_q_disp.put(disp_data[0])
                    #print(disp_data[i])
                    time.sleep(0.01)
                    
                    

                # Do something with the data
                return 0      # callback must return an integer, otherwise callback throws a wall of errors


            task.register_every_n_samples_acquired_into_buffer_event(1000, reading_task_callback_LC)
            task.start()
            while(1):
                time.sleep(.01)
                if not(self.lcreaderthreadActive):
                    break
            


class lcloggerWorker(QObject):
    lclogger_thread_finished = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.lcloggerthreadActive = False

    def start_lclogging_task(self):
        try:
            now = datetime.datetime.now()
            time_string = now.strftime("%m-%d-%Y_%H-%M-%S")+".csv"

            ##create fieldnames
            fieldnames = []
            fieldnames.append("Time")
            for i in range(4):
                fieldnames.append("cDAQ3Mod1/ai"+str(i))
            fieldnames_str = ','.join(fieldnames)

            index = 0
            while(self.lcloggerthreadActive):
                time.sleep(.01)
                with open("D:/jasev/Data_Dump/ARCS_data/SPIT_"+time_string, 'ab') as csvfile:
                    if (LC_q.qsize()>0):
                        # print("LC logging")
                        ## get data array from the queues
                        initial_time = LC_time_q.get() #get time data
                        time_array = np.linspace(initial_time, initial_time+1, num=1000) # make time array
                        # time_col = np.reshape(time_array,(1000,1))
                        LC_data = LC_q.get() #get pressure data
                        data = np.column_stack((time_array, LC_data))
                        # print(data.shape)
                        if index==0:
                            # print("header")
                            np.savetxt(csvfile, data, delimiter=',', header = fieldnames_str )
                        else:
                            np.savetxt(csvfile, data, delimiter=',')
                        index = 1
            self.lclogger_thread_finished.emit()
            print("Ended TC logging task")
        except Exception as e:
            print(e)
            self.lclogger_thread_finished.emit()
            print("Ended LC logging task")


class displayWorker(QObject):
    display_thread_finished = pyqtSignal()
    outgoing_TCdata = pyqtSignal(np.ndarray)
    outgoing_LCdata = pyqtSignal(np.ndarray)
    display_error = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.displaythreadActive = False

    def start_display_task(self):
        try:
            while(self.displaythreadActive):


                if(LC_q_disp.qsize()>0):
                    self.lcdata =  LC_q_disp.get()

                    if(len(self.lcdata)<3):
                        pass
                    else:
                        #print(self.displayData )
                        self.outgoing_LCdata.emit(self.lcdata)
                else:
                    time.sleep(0.01)
                    pass


        except Exception as e:
            print(e)
            print("Display Error")
            self.display_error.emit() #Display Error: Restart
        self.display_thread_finished.emit()
        print("Ended display task")


class Ui(QtWidgets.QMainWindow):
    def __init__(self):
        super(Ui, self).__init__() # Call the inherited classes __init__ method
        ui_path = os.path.dirname(os.path.abspath(__file__)) # obtain file path
        uic.loadUi(os.path.join(ui_path, 'new_proto_NIgui.ui'), self)
        self.logo.setPixmap(QtGui.QPixmap(os.path.join(ui_path, 'ARCS_MissionPatch.png')))
        self.show() # Show the GUI


        # Graphs Set up

        self.HECOPV_Optemp = 110
        # Creates random values to draw the graph once
        self.sizeOfline = 7
        self.time = list(range(self.sizeOfline))

        self.TC_COPV = [randint(0, 1) for _ in range(self.sizeOfline)]
        self.TC_Dome = [randint(0, 1) for _ in range(self.sizeOfline)]
        self.TC_Hand = [randint(0, 1) for _ in range(self.sizeOfline)]
        self.TC_LOX = [randint(0, 1) for _ in range(self.sizeOfline)]

        self.lc1 = [randint(0, 1) for _ in range(self.sizeOfline)]
        self.lc2 = [randint(0, 1) for _ in range(self.sizeOfline)]

        # Load Cell Graph Parameters
        self.lc_graph = pg.PlotWidget(self.centralwidget)
        self.lc_graph.setObjectName("lc_graph")
        self.horizontalLayout.addWidget(self.lc_graph)
        self.lc_graph.setBackground("k")
        styles = {"color": "red", "font-size": "18px"}
        self.lc_graph.setTitle("Load Cell Sensors", color="r", size="15pt")
        self.lc_graph.setLabel("left", "Force (lbf)", **styles)
        self.lc_graph.setLabel("bottom", "Time (s)", **styles)
        self.lc_graph.addLegend()
        self.lc_graph.showGrid(x=True, y=True)
        self.lc_graph.setYRange(0, 15)

        # Creates line reference for LOX Tank PT
        self.thrust_line = self.lc_graph.plot(self.time,self.lc1,name="Solenoid Thrust",pen=pg.mkPen(color=(200, 200, 200), width=2),
            symbol="t",
            symbolSize=5,
            symbolBrush=(0, 0, 200),
        )

        # Creates line reference for Fuel Tank PT
        # self.fuelLC_line = self.lc_graph.plot(self.time,self.lc2,name="Fuel Tank LC",pen=pg.mkPen(color=(225,200,125), width=2),
        #     symbol="t",
        #     symbolSize=5,
        #     symbolBrush=(225,200,125),
        # )

        # Rocket Picture Data Labels
        self.titlecard.setText(" S.P.I.T Stand Controller ")
        self.titlecard.setStyleSheet("color: rgb(255, 255, 255); font-size:36pt; font-weight:600; ")

        self.TC_COPV_Label.setText(" ")
        self.TC_Dome_Label.setText(" ")
        self.TC_Hand_Label.setText(" ")
        self.TC_LOX_Label.setText(" ")

        self.thrustLC_label.setText("{0:0.3f} lbf".format(self.lc1[-1]))
        self.fuelLC_label.setText(" ")


        # Label text defaults to black text must change to white manually
        # self.TC_Dome_Label.setStyleSheet("color: rgb(255, 255, 255); font: 9pt Verdana; ")
        # self.TC_COPV_Label.setStyleSheet("color: rgb(255, 255, 255); font: 9pt Verdana;")
        # self.TC_Hand_Label.setStyleSheet("color: rgb(255, 255, 255); font: 9pt Verdana; ")
        # self.TC_LOX_Label.setStyleSheet("color: rgb(255, 255, 255); font: 9pt Verdana;")

        self.thrustLC_label.setStyleSheet("color: rgb(255, 255, 255); font: 8pt Verdana;")
        #self.fuelLC_label.setStyleSheet("color: rgb(255, 255, 255); font: 8pt Verdana;")

        # Button hover color change + background color change
        self.start_button.setStyleSheet("QPushButton" "{" "background-color: rgb(255, 0, 0);" "}" 
                                         "QPushButton::hover""{""background-color : lightgreen;""}")
        self.log_button.setStyleSheet("QPushButton" "{" "background-color: rgb(255, 0, 0);" "}" 
                                         "QPushButton::hover""{""background-color : lightgreen;""}")
        self.stop_button.setStyleSheet("QPushButton" "{" "background-color: rgb(255, 0, 0);" "}" 
                                         "QPushButton::hover""{""background-color : lightgreen;""}")
        self.clearText_button.setStyleSheet("QPushButton" "{" "background-color: rgb(255, 255, 0);" "}" 
                                         "QPushButton::hover""{""background-color : lightgreen;""}")
        
        # Connect button press to actions
        self.start_button.clicked.connect(self.start_buttion_action)
        self.log_button.clicked.connect(self.log_buttion_action)
        self.stop_button.clicked.connect(self.stop_buttion_action)
        self.clearText_button.clicked.connect(self.clearText_button_action)

        # Logging Variable
        self.loggingState = False
        self.readingState = False

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_graph)

    def LCappend_incoming_data(self,incoming_data):
        self.time = self.time[1:]
        self.currentTime = int((datetime.datetime.now() - datetime.datetime(1970, 1, 1)).total_seconds()*1000) - self.timeZero
        self.time.append(self.currentTime/1000) # converts from ms to s & appends new time to array

        self.lc1 = self.lc1[1:]
        self.lc2 = self.lc2[1:]

        self.lc1.append(float(incoming_data[0])) # Mod4/ain 0
        self.lc2.append(float(incoming_data[1])) # Mod4/ain 1

        # self.TC_COPV.append( (float( ((incoming_data[0])*(9/5))+32))) # Mod3/ain 0
        # self.TC_Dome.append( (float( ((incoming_data[1])*(9/5))+32))) # Mod3/ain 1
        # self.TC_Hand.append( (float( ((incoming_data[2])*(9/5))+32))) # Mod3/ain 2
        # self.TC_LOX.append( (float( ((incoming_data[3])*(9/5))+32))) # Mod3/ain 3


    def update_graph(self):
        if self.readingState:
            #self.TC1_line.setData(self.time, self.TC_Dome) # ADC ch3
            #self.TC2_line.setData(self.time, self.TC_COPV) # ADC ch3

            self.thrust_line.setData(self.time, self.lc1) # ADC ch2

            # self.TC_Dome_Label.setText("{0:0.1f} F".format(self.TC_Dome[-1]))
            # self.TC_COPV_Label.setText("{0:0.1f} F".format(self.TC_COPV[-1]))
            # self.TC_Hand_Label.setText("{0:0.1f} F".format(self.TC_Hand[-1]))
            # self.TC_LOX_Label.setText("{0:0.1f} F".format(self.TC_LOX[-1]))

            self.thrustLC_label.setText("{0:0.3f} lbf".format(self.lc1[-1]))
            #self.fuelLC_label.setText("{0:0.1f} lbf".format(self.lc2[-1]))


    def start_buttion_action(self):
        self.status_terminal.append("> Start Button Pressed 😊 ")

        if(not self.readingState):
            self.timeZero = int((datetime.datetime.now() - datetime.datetime(1970, 1, 1)).total_seconds()*1000)
            self.start_button.setStyleSheet("QPushButton" "{" "background-color: rgb(0, 255, 0);" "}" 
                                            "QPushButton::hover""{""background-color : lightgreen;""}")
            
            # Create Reader and Display threads and Workers 
            # Create Thread and worker objects
            self.lcreadingthread = QThread()
            self.displaythread = QThread() 

            self.lcreader_worker = LCreaderWorker()
            self.display_worker = displayWorker()

            # Step 4: Move worker to the thread
            self.lcreader_worker.moveToThread(self.lcreadingthread)
            self.display_worker.moveToThread(self.displaythread)

            # Set control variables to true
            self.lcreader_worker.lcreaderthreadActive = True
            self.lcreader_worker.loggingworkerActive = False

            self.display_worker.displaythreadActive = True
            self.readingState = True

            # Start the threads
            self.lcreadingthread.start()
            self.displaythread.start()
            

            #Connect signals and slots

            # LC Reader Thread
            self.lcreadingthread.started.connect(self.lcreader_worker.start_lcreader_task)
            self.lcreader_worker.lcreader_thread_finished.connect(self.lcreadingthread.quit)
            self.lcreader_worker.lcreader_thread_finished.connect(self.lcreader_worker.deleteLater)
            self.lcreadingthread.finished.connect(self.lcreadingthread.deleteLater)
            self.lcreadingthread.finished.connect(lambda: self.error_terminal.append("> LC Reading Thread Closed"))

            # Display Thread
            self.displaythread.started.connect(self.display_worker.start_display_task)
            self.display_worker.display_thread_finished.connect(self.displaythread.quit)
            self.display_worker.display_thread_finished.connect(self.display_worker.deleteLater)
            self.displaythread.finished.connect(self.displaythread.deleteLater)

            self.display_worker.outgoing_LCdata.connect(self.LCappend_incoming_data)

            self.displaythread.finished.connect(lambda: self.error_terminal.append("> Display Thread Closed"))
            
            self.timer.start(100)

            # If error is present show
            self.lcreader_worker.lcreader_error.connect(lambda: self.error_terminal.append("> LC Reader Worker Error"))
            self.lcreader_worker.lcreader_error.connect(lambda: self.stop_buttion_action())

            # Individual responses from OBC are displayed on the OBC response terminal

            self.display_worker.display_error.connect(lambda: self.error_terminal.append("> Display Worker Error"))
            self.display_worker.display_error.connect(lambda: self.stop_buttion_action())

            self.error_terminal.append("> LC Reading Thread Started")
            self.error_terminal.append("> Display Thread Started")
            

            
        else:
            self.status_terminal.append("> Already reading...")

        
    def log_buttion_action(self):

        if(self.readingState):
            if(not self.loggingState):
                #print("logging!")
                self.status_terminal.append("> Started Logging")
                self.lcreader_worker.loggingworkerActive = True
                self.loggingState = True

                # LC logging thread variables
                self.lcloggingthread = QThread()
                self.lclogger_worker = lcloggerWorker()

                self.lclogger_worker.moveToThread(self.lcloggingthread) 

                self.lclogger_worker.lcloggerthreadActive = True

                self.lcloggingthread.start()

                self.lcloggingthread.started.connect(self.lclogger_worker.start_lclogging_task)
                self.lclogger_worker.lclogger_thread_finished.connect(self.lcloggingthread.quit)
                self.lclogger_worker.lclogger_thread_finished.connect(self.lclogger_worker.deleteLater)
                self.lcloggingthread.finished.connect(self.lcloggingthread.deleteLater)

                self.lcloggingthread.finished.connect(lambda: self.error_terminal.append("> LC Logging Thread Closed"))


                self.log_button.setStyleSheet("QPushButton" "{" "background-color: rgb(0, 255, 0);" "}" 
                                            "QPushButton::hover""{""background-color : lightgreen;""}")
                
                self.error_terminal.append("> Logging Thread Started")
                
            elif(self.loggingState):
                self.lclogger_worker.lcloggerthreadActive = False
                self.lcreader_worker.loggingworkerActive = False
                self.loggingState = False
                #print("Stopped logging!")
                self.status_terminal.append("> Stopped Logging")
                self.log_button.setStyleSheet("QPushButton" "{" "background-color: rgb(255, 0, 0);" "}" 
                                            "QPushButton::hover""{""background-color : lightgreen;""}")

        else:
            self.status_terminal.append("> Must be reading first before logging")
            

        
    def stop_buttion_action(self):
        self.status_terminal.append("> Stop button pressed!")
        if(self.readingState):
            self.timer.stop()
            self.lcreader_worker.lcreaderthreadActive = False
            self.lcreader_worker.loggingworkerActive = False
            self.readingState = False
            self.display_worker.displaythreadActive = False


            if(self.loggingState):
                self.lclogger_worker.lcloggerthreadActive = False
                self.loggingState = False

            self.start_button.setStyleSheet("QPushButton" "{" "background-color: rgb(255, 0, 0);" "}" 
                                         "QPushButton::hover""{""background-color : lightgreen;""}")
            self.log_button.setStyleSheet("QPushButton" "{" "background-color: rgb(255, 0, 0);" "}" 
                                         "QPushButton::hover""{""background-color : lightgreen;""}")
        else:
            self.error_terminal.append("> No Threads to stop")

    def clearText_button_action(self):
        self.status_terminal.setText(" Action Status Terminal ")
        self.error_terminal.setText(" Error / Thread Status Terminal ")



if __name__ == "__main__":
    QtWidgets.QApplication.setAttribute(QtCore.Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    app = QtWidgets.QApplication(sys.argv)
    window = Ui()   
    sys.exit(app.exec_())