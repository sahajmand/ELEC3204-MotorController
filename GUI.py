import os
os.environ['QT_QPA_PLATFORM'] = 'xcb'

import sys, time
import serial
from collections import deque
from serial.tools import list_ports
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

class Speedometer(QtWidgets.QWidget):
    def __init__(self, max_rpm=200, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._value = 0
        self._max = max_rpm

    def setValue(self, val):
        self._value = max(0, min(self._max, abs(val)))
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        rect = self.rect().adjusted(10, 10, -10, -10)
        pen = QtGui.QPen(QtGui.QColor('#4C566A'), 20)
        painter.setPen(pen)
        painter.drawArc(rect, 45 * 16, 270 * 16)
        angle = 45 + (self._value / self._max) * 270
        painter.save()
        painter.translate(rect.center())
        painter.rotate(angle)
        pen.setWidth(4)
        pen.setColor(QtGui.QColor('#88C0D0'))
        painter.setPen(pen)
        x2 = int(rect.width() / 2 - 20)
        painter.drawLine(0, 0, x2, 0)
        painter.restore()
        painter.setPen(QtGui.QColor('#D8DEE9'))
        font = painter.font()
        font.setPointSize(16)
        painter.setFont(font)
        painter.drawText(self.rect(), QtCore.Qt.AlignCenter, f"{int(self._value)} RPM")

class SerialReader(QtCore.QThread):
    data_received = QtCore.pyqtSignal(int, int, int, int)

    def __init__(self, serial_inst, parent=None):
        super().__init__(parent)
        self.ser = serial_inst
        self._running = True

    def run(self):
        while self._running:
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
            except Exception:
                continue
            if not line:
                continue
            parts = line.split(',')
            if len(parts) != 4:
                continue
            try:
                loop, direction, rpm, estop = map(int, parts)
            except ValueError:
                continue
            self.data_received.emit(loop, direction, rpm, estop)

class StateDashboard(QtWidgets.QWidget):
    def __init__(self, port=None):
        super().__init__()
        self.last_target = 0  # for graphing setpoint
        self.start_time = time.time()

        # Serial port selection
        ports = list_ports.comports()
        candidates = [p.device for p in ports if 'ACM' in p.device or 'USB' in p.device]
        if port and port in candidates:
            use_port = port
        elif candidates:
            use_port = candidates[0]
        else:
            QtWidgets.QMessageBox.critical(
                None, "Serial Error",
                "No USB/ACM serial ports detected.\n" + "\n".join(p.device for p in ports)
            )
            sys.exit(1)
        # Open serial port
        try:
            self.ser = serial.Serial(use_port, 9600, timeout=0.1)
        except Exception as e:
            QtWidgets.QMessageBox.critical(
                None, "Serial Error",
                f"Failed to open {use_port}: {e}"
            )
            sys.exit(1)

        # Build UI
        self._build_ui()
        self.setWindowTitle("Motor State Dashboard")
        self.setStyleSheet("background-color: #2E3440; color: #D8DEE9;")
        QtCore.QTimer.singleShot(0, self.showFullScreen)

        # Serial reader thread
        self.reader = SerialReader(self.ser)
        self.reader.data_received.connect(self.update_state)
        self.reader.start()

        # Timer to update plot
        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self._update_plot)
        self.plot_timer.start(100)

    def _make_box(self, title):
        box = QtWidgets.QGroupBox(title)
        box.setStyleSheet(
            "QGroupBox{border:2px solid #4C566A; border-radius:12px; background:#3B4252;}"
            "QGroupBox::title{color:#8FBCBB;}"
        )
        lbl = QtWidgets.QLabel("—")
        lbl.setAlignment(QtCore.Qt.AlignCenter)
        lbl.setFont(QtGui.QFont('', 18, QtGui.QFont.Bold))
        layout = QtWidgets.QVBoxLayout(box)
        layout.addStretch(); layout.addWidget(lbl); layout.addStretch()
        box.state_label = lbl
        box.setFixedSize(200, 150)
        return box

    def _build_ui(self):
        # State boxes
        self.box_loop  = self._make_box("Control Mode")
        self.box_dir   = self._make_box("Direction")
        self.box_estop = self._make_box("E-Stop")
        state_layout = QtWidgets.QHBoxLayout()
        state_layout.setSpacing(40)
        state_layout.addWidget(self.box_loop)
        state_layout.addWidget(self.box_dir)
        state_layout.addWidget(self.box_estop)

        # Speedometer
        self.speedo = Speedometer(200)
        self.speedo.setMinimumSize(300, 300)

        # PlotWidget
        self.plot_widget = pg.PlotWidget(title="RPM over Time")
        self.plot_widget.setBackground('#2E3440')
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_curve   = self.plot_widget.plot(pen=pg.mkPen('#88C0D0', width=2))
        self.target_curve = self.plot_widget.plot(pen=pg.mkPen('#FF5555', width=2, style=QtCore.Qt.DashLine))
        self.time_data    = deque(maxlen=200)
        self.rpm_data     = deque(maxlen=200)
        self.target_data  = deque(maxlen=200)

        # Controls layout
        btn_layout = QtWidgets.QHBoxLayout()
        for text in ["START", "STOP", "REVERSE"]:
            btn = QtWidgets.QPushButton(text)
            btn.clicked.connect(lambda _, t=text: self.send_cmd(t))
            btn_layout.addWidget(btn)
        self.edit_speed = QtWidgets.QLineEdit()
        self.edit_speed.setPlaceholderText("RPM")
        btn_set = QtWidgets.QPushButton("SET")
        btn_set.clicked.connect(self.on_set_speed)
        btn_layout.addWidget(self.edit_speed)
        btn_layout.addWidget(btn_set)

        # Main layout
        left = QtWidgets.QVBoxLayout()
        left.addLayout(state_layout)
        left.addWidget(self.speedo, alignment=QtCore.Qt.AlignCenter)
        left.addLayout(btn_layout)
        main = QtWidgets.QHBoxLayout(self)
        main.setContentsMargins(20,20,20,20)
        main.addLayout(left)
        main.addWidget(self.plot_widget)

    def update_state(self, loop, direction, rpm, estop):
        self.box_loop.state_label.setText("Closed-Loop" if loop else "Open-Loop")
        self.box_dir.state_label.setText("Reverse" if direction else "Forward")
        # E-stop color
        self.box_estop.state_label.setText("ON" if estop else "OFF")
        if estop:
            self.box_estop.setStyleSheet(
                "QGroupBox{border:2px solid #BF616A; background:#BF616A; border-radius:12px;}"
                "QGroupBox::title{color:#2E3440;}"
            )
        else:
            self.box_estop.setStyleSheet(
                "QGroupBox{border:2px solid #4C566A; background:#3B4252; border-radius:12px;}"
                "QGroupBox::title{color:#8FBCBB;}"
            )
        self.speedo.setValue(rpm)
        t = time.time() - self.start_time
        self.time_data.append(t)
        self.rpm_data.append(abs(rpm))
        self.target_data.append(self.last_target if loop else float('nan'))

    def _update_plot(self):
        self.plot_curve.setData(self.time_data, self.rpm_data)
        self.target_curve.setData(self.time_data, self.target_data)

    def send_cmd(self, cmd):
        try:
            self.ser.write((cmd + "\n").encode('ascii'))
            if cmd.startswith("SPEED "):
                self.last_target = int(cmd.split()[1])
        except:
            pass

    def on_set_speed(self):
        txt = self.edit_speed.text().strip()
        if txt.lstrip('-').isdigit():
            self.send_cmd(f"SPEED {txt}")

    def closeEvent(self, event):
        self.reader._running = False
        self.reader.wait(200)
        event.accept()

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    dash = StateDashboard()
    sys.exit(app.exec_())




# import os
# os.environ['QT_QPA_PLATFORM'] = 'xcb'

# import sys, time
# import serial
# from collections import deque
# from serial.tools import list_ports
# from PyQt5 import QtWidgets, QtCore, QtGui
# import pyqtgraph as pg

# class Speedometer(QtWidgets.QWidget):
#     def __init__(self, max_rpm=100, *args, **kwargs):
#         super().__init__(*args, **kwargs)
#         self._value = 0
#         self._max = max_rpm

#     def setValue(self, val):
#         self._value = max(0, min(self._max, abs(val)))
#         self.update()

#     def paintEvent(self, event):
#         painter = QtGui.QPainter(self)
#         painter.setRenderHint(QtGui.QPainter.Antialiasing)
#         rect = self.rect().adjusted(10, 10, -10, -10)
#         pen = QtGui.QPen(QtGui.QColor('#4C566A'), 20)
#         painter.setPen(pen)
#         painter.drawArc(rect, 45 * 16, 270 * 16)
#         angle = 45 + (self._value / self._max) * 270
#         painter.save()
#         painter.translate(rect.center())
#         painter.rotate(angle)
#         pen.setWidth(4)
#         pen.setColor(QtGui.QColor('#88C0D0'))
#         painter.setPen(pen)
#         x2 = int(rect.width() / 2 - 20)
#         painter.drawLine(0, 0, x2, 0)
#         painter.restore()
#         painter.setPen(QtGui.QColor('#D8DEE9'))
#         font = painter.font()
#         font.setPointSize(16)
#         painter.setFont(font)
#         painter.drawText(self.rect(), QtCore.Qt.AlignCenter, f"{int(self._value)} RPM")

# class SerialReader(QtCore.QThread):
#     data_received = QtCore.pyqtSignal(int, int, int, int)
 
#     def __init__(self, serial_inst, parent=None):
#         super().__init__(parent)
#         self.ser = serial_inst
#         self._running = True

#     def run(self):
#         while self._running:
#             try:
#                 line = self.ser.readline().decode('ascii', errors='ignore').strip()
#             except Exception:
#                 continue
#             if not line:
#                 continue
#             parts = line.split(',')
#             if len(parts) != 4:
#                 continue
#             try:
#                 loop, direction, rpm, estop = map(int, parts)
#             except ValueError:
#                 continue
#             self.data_received.emit(loop, direction, rpm, estop)

# class StateDashboard(QtWidgets.QWidget):
#     def __init__(self, port=None):
#         super().__init__()
#         self.last_target = 0  # store last setpoint for plotting

#     def __init__(self, port=None):
#         super().__init__()
#         # Serial port selection
#         ports = list_ports.comports()
#         candidates = [p.device for p in ports if 'ACM' in p.device or 'USB' in p.device]
#         if port and port in candidates:
#             use_port = port
#         elif candidates:
#             use_port = candidates[0]
#         else:
#             QtWidgets.QMessageBox.critical(
#                 None, "Serial Error",
#                 "No USB/ACM serial ports detected.\n" + "\n".join(p.device for p in ports)
#             )
#             sys.exit(1)
#         # Open serial port
#         try:
#             self.ser = serial.Serial(use_port, 9600, timeout=0.1)
#         except Exception as e:
#             QtWidgets.QMessageBox.critical(
#                 None, "Serial Error",
#                 f"Failed to open {use_port}: {e}"
#             )
#             sys.exit(1)

#         self.setWindowTitle("Motor State Dashboard")
#         self.setStyleSheet("background-color: #2E3440; color: #D8DEE9;")
#         self._build_ui()
#         QtCore.QTimer.singleShot(0, self.showFullScreen)

#         # Serial reader
#         self.reader = SerialReader(self.ser)
#         self.reader.data_received.connect(self.update_state)
#         self.reader.start()

#         # Timer to update plot
#         self.plot_timer = QtCore.QTimer(self)
#         self.plot_timer.timeout.connect(self._update_plot)
#         self.plot_timer.start(100)

#     def _make_box(self, title):
#         box = QtWidgets.QGroupBox(title)
#         box.setStyleSheet(
#             "QGroupBox{border:2px solid #4C566A; border-radius:12px; background:#3B4252;}"
#             "QGroupBox::title{color:#8FBCBB;}"
#         )
#         lbl = QtWidgets.QLabel("—")
#         lbl.setAlignment(QtCore.Qt.AlignCenter)
#         lbl.setFont(QtGui.QFont('', 18, QtGui.QFont.Bold))
#         layout = QtWidgets.QVBoxLayout(box)
#         layout.addStretch(); layout.addWidget(lbl); layout.addStretch()
#         box.state_label = lbl
#         box.setFixedSize(200, 150)
#         return box

#     def _build_ui(self):
#         # State boxes
#         self.box_loop  = self._make_box("Control Mode")
#         self.box_dir   = self._make_box("Direction")
#         self.box_estop = self._make_box("E-Stop")
#         states_layout = QtWidgets.QHBoxLayout()
#         states_layout.setSpacing(40)
#         states_layout.addWidget(self.box_loop)
#         states_layout.addWidget(self.box_dir)
#         states_layout.addWidget(self.box_estop)

#         # Speedometer
#         self.speedo = Speedometer(100)
#         self.speedo.setMinimumSize(300, 300)

#         # Plot with PyQtGraph
#         self.plot_widget = pg.PlotWidget(title="RPM over Time")
#         self.plot_widget.setBackground('#2E3440')
#         self.plot_widget.showGrid(x=True, y=True)
#         # actual RPM curve (blue)
#         self.plot_curve = self.plot_widget.plot(pen=pg.mkPen('#88C0D0', width=2))
#         # target RPM curve (orange dashed)
#         self.target_curve = self.plot_widget.plot(pen=pg.mkPen('#D08770', width=2, style=QtCore.Qt.DashLine))
#         self.time_data = deque(maxlen=200)
#         self.rpm_data  = deque(maxlen=200)
#         self.target_data = deque(maxlen=200)
#         self.start_time = time.time()
#         self.time_data = deque(maxlen=200)
#         self.rpm_data  = deque(maxlen=200)
#         self.start_time = time.time()

#         # Controls
#         self.btn_start   = QtWidgets.QPushButton("START")
#         self.btn_stop    = QtWidgets.QPushButton("STOP")
#         self.btn_reverse = QtWidgets.QPushButton("REVERSE")
#         self.edit_speed  = QtWidgets.QLineEdit(); self.edit_speed.setPlaceholderText("RPM")
#         self.btn_set     = QtWidgets.QPushButton("SET")
#         ctrl_layout = QtWidgets.QHBoxLayout()
#         for btn, cmd in [(self.btn_start, "START"), (self.btn_stop, "STOP"), (self.btn_reverse, "REVERSE")]:
#             btn.clicked.connect(lambda _, c=cmd: self.send_cmd(c))
#             ctrl_layout.addWidget(btn)
#         ctrl_layout.addWidget(self.edit_speed)
#         self.btn_set.clicked.connect(self.on_set_speed)
#         ctrl_layout.addWidget(self.btn_set)

#         # Left layout
#         left_layout = QtWidgets.QVBoxLayout()
#         left_layout.addLayout(states_layout)
#         left_layout.addWidget(self.speedo, alignment=QtCore.Qt.AlignCenter)
#         left_layout.addLayout(ctrl_layout)

#         # Main layout
#         main_layout = QtWidgets.QHBoxLayout(self)
#         main_layout.setContentsMargins(20, 20, 20, 20)
#         main_layout.addLayout(left_layout)
#         main_layout.addWidget(self.plot_widget)

#     def update_state(self, loop, direction, rpm, estop):
#         self.box_loop.state_label.setText("Closed-Loop" if loop else "Open-Loop")
#         self.box_dir.state_label.setText("Reverse" if direction else "Forward")
#         self.box_estop.state_label.setText("ON" if estop else "OFF")
#         self.speedo.setValue(rpm)
#         # buffer data for plot
#         t = time.time() - self.start_time
#         self.time_data.append(t)
#         self.rpm_data.append(abs(rpm))

#     def _update_plot(self):
#         self.plot_curve.setData(list(self.time_data), list(self.rpm_data))

#     def send_cmd(self, cmd):
#         try:
#             self.ser.write((cmd+"\n").encode('ascii'))
#         except:
#             pass

#     def on_set_speed(self):
#         txt = self.edit_speed.text().strip()
#         if not txt.isdigit():
#             return
#         self.send_cmd(f"SPEED {txt}")

#     def closeEvent(self, event):
#         self.reader._running = False
#         self.reader.wait(200)
#         event.accept()

# if __name__ == '__main__':
#     app = QtWidgets.QApplication(sys.argv)
#     dash = StateDashboard()
#     sys.exit(app.exec_())
