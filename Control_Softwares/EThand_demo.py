import serial
import serial.tools.list_ports
import time
import queue
from SDK import servo_controller
import numpy as np
import os
import sys
import threading
from PyQt5.QtWidgets import QApplication, QMainWindow, QSlider, QLabel, QLineEdit, QVBoxLayout, QHBoxLayout, QGridLayout, QWidget, QGraphicsOpacityEffect
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from PyQt5.QtCore import Qt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

os.chdir(os.path.abspath(os.path.dirname(__file__)))

FRAME_HEADER = b"\xaa\x55\xaa"
FRAME_FOOTER = b"\x0d\x0a"

ports = serial.tools.list_ports.comports()
available_ports = [port.device for port in ports]
for i, port in enumerate(available_ports):
    print(f"{i+1}. {port}")
choice = int(input("Select Sensor COM(num):"))

selected_port = available_ports[choice - 1]
serial_sensor = serial.Serial(
    port=selected_port, baudrate=1000000
)

receive_queue = queue.Queue()

sensor_type = np.zeros((12, 1), dtype=np.int_)
sensor_rawdata_array = np.zeros((12, 16))
history_size = 100
history_type1 = [[] for _ in range(12)]
history_type2 = [[] for _ in range(12)]

# Servo and simulation initialization
BusServo = servo_controller.ServoController()
client = RemoteAPIClient()
sim = client.require("sim")
sim.startSimulation()

jointHandles = [
    [sim.getObjectHandle(f"/EThand/Rjoint{i}") for i in range(3)],
    [sim.getObjectHandle(f"/EThand/Rjoint{i}") for i in range(3, 6)],
    [sim.getObjectHandle(f"/EThand/Rjoint{i}") for i in range(6, 9)]
]
angle_to_rad = np.pi / 180.0

Servo_deg_range = 240
Servo_deg_step = Servo_deg_range / 1000
servo_v = 500
Closs_Direction = [1, 1, -1, 1, -1, 1, -1, -1, 1]
origin_table = [442, 400, 435, 420, 450, 378, 400, 460, 460]
MaxOpenJ1Angle = 60
MinCloseJ1Angle = -50
MaxOpenJ2Angle = 20
MinCloseJ2Angle = -70
ServoIndex = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
SetJointPos = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
GetJointPos = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

def exp_function(x, a, b):
    return a * np.exp(x * b)

def polynomial(x, *coeffs):
    return np.polyval(coeffs, x)

coeffs = np.zeros(4)
coeffs[0] = 6.4586e-09
coeffs[1] = -2.5266e-05
coeffs[2] = 2.8187e-02
coeffs[3] = -5.2599e00

def transform_forc(x):
    y = polynomial(x, *coeffs)
    if y > 0:
        return y
    else:
        return 0

a = 52.66
b = -0.0143
def transform_prox(x):
    return exp_function(x, a, b)

def Joint_set(ID, deg):
    BusServo.set_servo_position(
        ID, origin_table[ID - 1] + Closs_Direction[ID - 1] * (deg / Servo_deg_step), servo_v)

def read_thread():
    while True:
        try:
            buffer = bytearray()
            header_found = False
            while not header_found:
                byte = serial_sensor.read(1)
                if byte == FRAME_HEADER[0:1]:
                    next_byte = serial_sensor.read(1)
                    if next_byte == FRAME_HEADER[1:2]:
                        next_next_byte = serial_sensor.read(1)
                        if next_next_byte == FRAME_HEADER[2:3]:
                            buffer.extend(FRAME_HEADER)
                            header_found = True
                            continue
            while True:
                byte = serial_sensor.read(1)
                buffer.extend(byte)
                if buffer[-2:] == FRAME_FOOTER:
                    break

            if buffer:
                receive_queue.put(buffer[0: -len(FRAME_FOOTER)])
                print(f"Received 1 line:{buffer.hex()}\r\n")

        except serial.SerialException as e:
            print(f"Error reading to serial port: {e}")
            break

def write_thread():
    while True:
        try:
            data = input('Enter data to send (start or end):')

            if data == 'start':
                buf=b'\x53'
                serial_sensor.write(buf)  # send
            elif data=='end':
                buf=b'\x45'
                serial_sensor.write(buf)  # send
            else:
                print(f'Please write: start or end\r\n')

        except serial.SerialException as e:
            print(f'Error writing to serial port: {e} \r\n')
            break

def data_processing_thread():  
    while True:   
        data = receive_queue.get()  
        process_data(data)  
        receive_queue.task_done()

def process_data(data):
    global sensor_type, sensor_rawdata_array 
    if (len(data)==399):
        if (data[0] == 170) & (data [1] == 85) & (data[2] == 170) :
            sensor_serial_data = data[3:]
            sensor_serial_array = np.frombuffer(sensor_serial_data, dtype=np.uint8)  
            if len(sensor_serial_array) == 396:
                sensor_serial_array = sensor_serial_array.reshape((12, 33))
                sensor_type = sensor_serial_array[:, 0].reshape(12, 1)  
                remaining_bytes = sensor_serial_array[:, 1:]  
                reshaped_remaining = remaining_bytes.reshape(12, 16, 2)  
                reshaped_remaining_contiguous = np.ascontiguousarray(reshaped_remaining)  
                reshaped_remaining_uint16 = reshaped_remaining_contiguous.view(np.uint16)   
                reshaped_remaining_uint16 = reshaped_remaining_uint16.byteswap()  
                sensor_rawdata_array = reshaped_remaining_uint16.reshape(12, -1) 
            else:  
                print("Can not be remolded into a 12x33 array.")
        else:
            print("Frame header error\r\n")
    else: 
        print("Length error\r\n") 

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle(r"ET-Hand")
        self.setGeometry(100, 100, 1000, 800)

        self.centralWidget = QWidget()
        self.setCentralWidget(self.centralWidget)
        self.layout = QGridLayout(self.centralWidget)

        self.background_label = QLabel(self)
        pixmap = QtGui.QPixmap("NCU.png")
        self.background_label.setPixmap(pixmap)
        self.background_label.setScaledContents(True)
        self.background_label.setGeometry(self.rect())
        self.background_label.lower()

        opacity_effect = QGraphicsOpacityEffect(self)
        opacity_effect.setOpacity(0.2)
        self.background_label.setGraphicsEffect(opacity_effect)

        #12 PlotWidget
        self.plots = [pg.PlotWidget() for _ in range(12)]
        for plot in self.plots:
            plot.setBackground("w")

        #PlotWidget
        layout_order = [[1, 3, 5], [2, 4, 6], [7, 9, 11], [8, 10, 12]]
        row, col = 0, 0
        for index in [item for sublist in layout_order for item in sublist]:
            self.layout.addWidget(self.plots[index - 1], row, col)
            col += 1
            if col == 3:
                row += 1
                col = 0

        self.label0 = QLabel("Proximal Joint", self)
        
        self.slider00 = QSlider(Qt.Horizontal, self)
        self.slider00.setMinimum(-50)
        self.slider00.setMaximum(60)
        self.slider00.setValue(0)
        self.slider00.valueChanged.connect(lambda: self.updateSlider00(self.slider00.value()))

        self.slider01 = QSlider(Qt.Horizontal, self)
        self.slider01.setMinimum(-50)
        self.slider01.setMaximum(60)
        self.slider01.setValue(0)
        self.slider01.valueChanged.connect(lambda: self.updateSlider01(self.slider01.value()))

        self.slider02 = QSlider(Qt.Horizontal, self)
        self.slider02.setMinimum(-50)
        self.slider02.setMaximum(60)
        self.slider02.setValue(0)
        self.slider02.valueChanged.connect(lambda: self.updateSlider02(self.slider02.value()))

        self.label1 = QLabel("Intermediate Joint", self)
        self.slider1 = QSlider(Qt.Horizontal, self)
        self.slider1.setMinimum(-50)
        self.slider1.setMaximum(60)
        self.slider1.setValue(60)
        self.slider1.valueChanged.connect(lambda: self.updateSlider1(self.slider1.value()))

        self.label2 = QLabel("Distal Joint", self)
        self.slider2 = QSlider(Qt.Horizontal, self)
        self.slider2.setMinimum(-70)
        self.slider2.setMaximum(20)
        self.slider2.setValue(0)
        self.slider2.valueChanged.connect(lambda: self.updateSlider2(self.slider2.value()))

        self.joint_labels = []
        joint_names = ["F1_2", "F2_2", "F3_2", "F1_1", "F2_1", "F3_1", "F1_0", "F2_0", "F3_0"]
        for name in joint_names:
            label = QLabel(f"{name}: 0.0", self)
            self.joint_labels.append(label)

        control_layout = QVBoxLayout()
        control_layout.addWidget(self.label0)
        h_layout = QHBoxLayout()
        h_layout.addWidget(self.slider00)
        h_layout.addWidget(self.slider01)
        h_layout.addWidget(self.slider02)
        control_layout.addLayout(h_layout)

        control_layout.addWidget(self.label1)
        control_layout.addWidget(self.slider1)
        control_layout.addWidget(self.label2)
        control_layout.addWidget(self.slider2)

        self.layout.addLayout(control_layout, row + 1, 0, 1, 3)

        grid_layout = QGridLayout()
        for i in range(3):
            for j in range(3):
                grid_layout.addWidget(self.joint_labels[i * 3 + j], i, j)
        self.layout.addLayout(grid_layout, row + 2, 0, 1, 3)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(100)

    def update_plots(self):
        global sensor_type, sensor_rawdata_array, history_size, history_type1, history_type2

        self.x_axis_counts = [0] * len(self.plots)   
          
        for i, plot in enumerate(self.plots):   
            plot.clear()
              
            if sensor_type[i] == 1:  
                history_type1[i].append(transform_prox(sensor_rawdata_array[i, 0]))  
                if len(history_type1[i]) > history_size:  
                    history_type1[i].pop(0)
                x = np.arange(len(history_type1[i]))  
                plot.plot(x, history_type1[i], pen='r')    
            elif sensor_type[i] == 2:  
                history_type2[i].append(transform_forc(np.sum(sensor_rawdata_array[i,:8])/8)) 
                if len(history_type2[i]) > history_size:  
                    history_type2[i].pop(0)
                x = np.arange(len(history_type2[i]))  
                plot.plot(x, history_type2[i], pen='g')

            elif sensor_type[i] == 3:  
                image = pg.ImageItem(sensor_rawdata_array[i].reshape(4, 4))  
                plot.addItem(image)

            else:
                image = pg.ImageItem(sensor_rawdata_array[i].reshape(4, 4))  
                plot.addItem(image)

    def updateSlider00(self, value):
        Joint_set(ServoIndex[0][0], np.float_(value))
        sim.setJointTargetPosition(jointHandles[0][0], np.float_(value) * angle_to_rad)
        time.sleep(0.1)
        GetJointPos[0][0] = (sim.getJointPosition(jointHandles[0][0]) / angle_to_rad)
        self.joint_labels[0].setText(f"F1_0: {GetJointPos[0][0]:.2f}")

    def updateSlider01(self, value):
        Joint_set(ServoIndex[1][0], np.float_(value))
        sim.setJointTargetPosition(jointHandles[1][0], np.float_(value) * angle_to_rad)
        time.sleep(0.1)
        GetJointPos[1][0] = (sim.getJointPosition(jointHandles[1][0]) / angle_to_rad)
        self.joint_labels[1].setText(f"F2_0: {GetJointPos[1][0]:.2f}")

    def updateSlider02(self, value):
        Joint_set(ServoIndex[2][0], np.float_(value))
        sim.setJointTargetPosition(jointHandles[2][0], np.float_(value) * angle_to_rad)
        time.sleep(0.1)
        GetJointPos[2][0] = (sim.getJointPosition(jointHandles[2][0]) / angle_to_rad)
        self.joint_labels[2].setText(f"F3_0: {GetJointPos[2][0]:.2f}")

    def updateSlider1(self, value):
        for finger in range(3):
            Joint_set(ServoIndex[finger][1], np.float_(value))
            sim.setJointTargetPosition(jointHandles[finger][1], value * angle_to_rad)
            
        time.sleep(0.1)
        for finger in range(3):
            GetJointPos[finger][1] = (sim.getJointPosition(jointHandles[finger][1]) / angle_to_rad)
        
        self.joint_labels[3].setText(f"F1_1: {GetJointPos[0][1]:.2f}")
        self.joint_labels[4].setText(f"F2_1: {GetJointPos[1][1]:.2f}")
        self.joint_labels[5].setText(f"F3_1: {GetJointPos[2][1]:.2f}")

    def updateSlider2(self, value):
        for finger in range(3):
            Joint_set(ServoIndex[finger][2], np.float_(value))
            sim.setJointTargetPosition(jointHandles[finger][2], value * angle_to_rad)
            GetJointPos[finger][2] = (sim.getJointPosition(jointHandles[finger][2]) / angle_to_rad)

        time.sleep(0.1)
        for finger in range(3):
            GetJointPos[finger][2] = (sim.getJointPosition(jointHandles[finger][2]) / angle_to_rad)
        
        self.joint_labels[6].setText(f"F1_2: {GetJointPos[0][2]:.2f}")
        self.joint_labels[7].setText(f"F2_2: {GetJointPos[1][2]:.2f}")
        self.joint_labels[8].setText(f"F3_2: {GetJointPos[2][2]:.2f}")


if __name__ == "__main__":
    threading.Thread(target=read_thread, daemon=True).start()
    threading.Thread(target=write_thread, daemon=True).start()
    threading.Thread(target=data_processing_thread, daemon=True).start()
    

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    sys.exit(app.exec_())
