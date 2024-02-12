import ntcore
from wpilib import SmartDashboard
from PyQt6.QtWidgets import QApplication, QMainWindow, QGridLayout, QLabel, QPushButton, QVBoxLayout, QWidget

class Dashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dashboard")
        self.setGeometry(100, 100, 800, 600)
        self.layout = QGridLayout()
        self.central_widget = QWidget()
        self.central_widget.setLayout(self.layout)
        self.setCentralWidget(self.central_widget)
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.nt.startClient4('dashboard')
        self.nt.startClient3('dashboard')
        self.ds = self.nt.startDSClient()
        self.init_ui()

    def init_ui(self):
        self.layout.addWidget(QLabel("Dashboard"), 0, 0)
        self.layout.addWidget(QPushButton("Button"), 1, 0)

        self.show()
        
    def update(self):
        ...
    
    def closeEvent(self, event):
        self.nt.stopClient()
        event.accept()
        
        
if __name__ == "__main__":
    app = QApplication([])
    window = Dashboard()
    app.exec()
    
    
