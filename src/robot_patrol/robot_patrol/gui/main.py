import sys
from PyQt5.QtWidgets import QApplication
from dashboard import SecurityRobotPyQt

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    win = SecurityRobotPyQt()
    win.show()

    sys.exit(app.exec_())