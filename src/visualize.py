import serial
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore

# Remplace 'COM3' par ton port série
ser = serial.Serial('COM4', 115200)

app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(title="MPU6050 - Roll et Pitch")
plot = win.addPlot(title="Roll et Pitch")
curve_roll = plot.plot(pen='r', name="Roll")
curve_pitch = plot.plot(pen='b', name="Pitch")
data_roll = []
data_pitch = []

def update():
    global data_roll, data_pitch
    line = ser.readline().decode().strip()
    try:
        roll, pitch = map(float, line.split(","))
        data_roll.append(roll)
        data_pitch.append(pitch)

        # Limite la taille des listes à 200 points pour éviter de surcharger la mémoire
        if len(data_roll) > 200:
            data_roll = data_roll[-200:]
            data_pitch = data_pitch[-200:]

        curve_roll.setData(data_roll)
        curve_pitch.setData(data_pitch)
    except Exception as e:
        print("Erreur:", e)

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(2)

win.show()
app.exec_()
