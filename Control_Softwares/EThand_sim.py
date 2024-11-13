import numpy as np
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QSlider, QLabel, QLineEdit, QVBoxLayout, QHBoxLayout, QGridLayout, QWidget
from PyQt5.QtCore import Qt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require("sim")

sim.startSimulation()

jointHandles = [
    [
        sim.getObjectHandle("/EThand/Rjoint0"),
        sim.getObjectHandle("/EThand/Rjoint1"),
        sim.getObjectHandle("/EThand/Rjoint2"),
    ],
    [
        sim.getObjectHandle("/EThand/Rjoint3"),
        sim.getObjectHandle("/EThand/Rjoint4"),
        sim.getObjectHandle("/EThand/Rjoint5"),
    ],
    [
        sim.getObjectHandle("/EThand/Rjoint6"),
        sim.getObjectHandle("/EThand/Rjoint7"),
        sim.getObjectHandle("/EThand/Rjoint8"),
    ],
]
angle_to_rad = np.pi / 180.0

MaxOpenJ1Angle = 60
MinCloseJ1Angle = -50
MaxOpenJ2Angle = 20
MinCloseJ2Angle = -70

GetJointPos = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.label00 = QLabel("Proximal Joint 1", self)
        self.lineEdit00 = QLineEdit(self)
        self.label01 = QLabel("Proximal Joint 2", self)
        self.lineEdit01 = QLineEdit(self)
        self.label02 = QLabel("Proximal Joint 3", self)
        self.lineEdit02 = QLineEdit(self)

        self.label1 = QLabel("Intermediate Joint", self)
        self.slider1 = QSlider(Qt.Horizontal, self)
        self.slider1.setMinimum(-50)
        self.slider1.setMaximum(60)
        self.slider1.setValue(60)

        self.lineEdit1 = QLineEdit(self)
        self.lineEdit1.setFixedWidth(110)
        self.lineEdit1.setText(str(self.slider1.value()))

        self.label2 = QLabel("Distal Joint", self)
        self.slider2 = QSlider(Qt.Horizontal, self)
        self.slider2.setMinimum(-70)
        self.slider2.setMaximum(20)
        self.slider2.setValue(0)

        self.lineEdit2 = QLineEdit(self)
        self.lineEdit2.setFixedWidth(90)
        self.lineEdit2.setText(str(self.slider2.value()))

        self.joint_labels = []
        joint_names = ["F1_2", "F2_2", "F3_2", "F1_1", "F2_1", "F3_1", "F1_0", "F2_0", "F3_0"]
        for name in joint_names:
            label = QLabel(f"{name}: 0.0", self)
            self.joint_labels.append(label)

        self.lineEdit00.textChanged.connect(self.updateLineEdit00)
        self.lineEdit01.textChanged.connect(self.updateLineEdit01)
        self.lineEdit02.textChanged.connect(self.updateLineEdit02)

        self.slider1.valueChanged.connect(self.updateLineEdit1)
        self.lineEdit1.textChanged.connect(self.updateSlider1)
        self.slider2.valueChanged.connect(self.updateLineEdit2)
        self.lineEdit2.textChanged.connect(self.updateSlider2)

        layout = QVBoxLayout()
        layout.addWidget(self.label00)
        layout.addWidget(self.lineEdit00)
        layout.addWidget(self.label01)
        layout.addWidget(self.lineEdit01)
        layout.addWidget(self.label02)
        layout.addWidget(self.lineEdit02)
        layout.addWidget(self.label1)
        layout.addWidget(self.slider1)
        layout.addWidget(self.lineEdit1)
        layout.addWidget(self.label2)
        layout.addWidget(self.slider2)
        layout.addWidget(self.lineEdit2)

        grid_layout = QGridLayout()

        for i in range(3):
            for j in range(3):
                grid_layout.addWidget(self.joint_labels[i * 3 + j], i, j)

        layout.addLayout(grid_layout)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.show()

    def updateLineEdit00(self, value):

        sim.setJointTargetPosition(jointHandles[0][0], np.float_(value) * angle_to_rad)
        GetJointPos[0][0] = (sim.getJointPosition(jointHandles[0][0]) / angle_to_rad)
        self.joint_labels[0].setText(f"F1_0: {GetJointPos[0][0]:.2f}")

    def updateLineEdit01(self, value):

        sim.setJointTargetPosition(jointHandles[1][0], np.float_(value) * angle_to_rad)
        GetJointPos[1][0] = (sim.getJointPosition(jointHandles[1][0]) / angle_to_rad)
        self.joint_labels[1].setText(f"F2_0: {GetJointPos[1][0]:.2f}")

    def updateLineEdit02(self, value):

        sim.setJointTargetPosition(jointHandles[2][0], np.float_(value) * angle_to_rad)
        GetJointPos[2][0] = (sim.getJointPosition(jointHandles[2][0]) / angle_to_rad)
        self.joint_labels[2].setText(f"F3_0: {GetJointPos[2][0]:.2f}")

    def updateLineEdit1(self, value):
        self.lineEdit1.setText(str(value))
        
        for finger in range(3):
                sim.setJointTargetPosition(jointHandles[finger][1], np.float_(value) * angle_to_rad)
                GetJointPos[finger][1] = (sim.getJointPosition(jointHandles[finger][1]) / angle_to_rad)
        
        self.joint_labels[3].setText(f"F1_1: {GetJointPos[0][1]:.2f}")
        self.joint_labels[4].setText(f"F2_1: {GetJointPos[1][1]:.2f}")
        self.joint_labels[5].setText(f"F3_1: {GetJointPos[2][1]:.2f}")

    def updateSlider1(self, text):
        try:
            value = int(text)
            self.slider1.setValue(value)
            for finger in range(3):
                sim.setJointTargetPosition(jointHandles[finger][1], np.float_(value) * angle_to_rad)
                GetJointPos[finger][1] = (sim.getJointPosition(jointHandles[finger][1]) / angle_to_rad)
        
            self.joint_labels[3].setText(f"F1_1: {GetJointPos[0][1]:.2f}")
            self.joint_labels[4].setText(f"F2_1: {GetJointPos[1][1]:.2f}")
            self.joint_labels[5].setText(f"F3_1: {GetJointPos[2][1]:.2f}")
        except ValueError:
            pass

    def updateLineEdit2(self, value):
        self.lineEdit2.setText(str(value))
        for finger in range(3):
            sim.setJointTargetPosition(jointHandles[finger][2], np.float_(value) * angle_to_rad)
            GetJointPos[finger][2] = (sim.getJointPosition(jointHandles[finger][2]) / angle_to_rad)
        
        self.joint_labels[6].setText(f"F1_2: {GetJointPos[0][2]:.2f}")
        self.joint_labels[7].setText(f"F2_2: {GetJointPos[1][2]:.2f}")
        self.joint_labels[8].setText(f"F3_2: {GetJointPos[2][2]:.2f}")

    def updateSlider2(self, text):
        try:
            value = int(text)
            self.slider2.setValue(value)
            for finger in range(3):
                sim.setJointTargetPosition(jointHandles[finger][2], np.float_(value) * angle_to_rad)
                GetJointPos[finger][2] = (sim.getJointPosition(jointHandles[finger][2]) / angle_to_rad)
        
            self.joint_labels[6].setText(f"F1_2: {GetJointPos[0][2]:.2f}")
            self.joint_labels[7].setText(f"F2_2: {GetJointPos[1][2]:.2f}")
            self.joint_labels[8].setText(f"F3_2: {GetJointPos[2][2]:.2f}")

        except ValueError:
            pass 

if __name__ == "__main__":

    try:

        app = QApplication(sys.argv)
        window = MainWindow()
        app.quit()
        sys.exit(app.exec_())

    except KeyboardInterrupt:
        print("Exiting...")
        app.quit()
        sys.exit(0)
