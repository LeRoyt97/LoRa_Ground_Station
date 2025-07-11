import os
from _pyrepl import reader

from LoRaReader import LoRaReader, LoraDataObject
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.uic import loadUi
import sys
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QObject, QThread
import serial.tools.list_ports
import time
from Ground_Station_Arduino import Ground_Station_Arduino
from satelliteTrackingMath import trackMath
from sunposition import sunpos
import datetime
import csv
import statistics
import numpy as np
from pylab import *

# todo: LeRoy: Reorganize GUI: Make statusBox always visible, so returns/feedback can be seen during setup.
#todo: Clean up unused Debug prints. Some don't have "Debug" in them
#todo: save serial monitor data to a .txt file
#todo: calculate distance from balloon
class Window(QMainWindow):
    log_signal = pyqtSignal(str)

    def __init__(self):
        super(Window, self).__init__()
        loadUi('LoRa_Designer.ui', self)

        # Reader placeholder
        self.reader = None

        self.arduinoConnected = False
        self.GSLocationSet = False
        self.calibrated = False
        self.loraPortNames = []
        self.arduinoPortNames = []
        self.LoRaListening = False
        self.tracking = False
        self.predictingTrack = False

        self.GSArduino = None  # classes will be instantiated later

        self.ports = None

        self.GSLat = None
        self.GSLong = None
        self.GSAlt = None

        # self.log_signal.connect(self.display_data)
# todo: LeRoy: fix issue that makes it so you need to refresh the ports before any can be selected -- Done: Now Test
        # Fill ComboBox initially
        self.refresh_ports(self.LoRaComboBox, self.loraPortNames)
        self.refresh_ports(self.ArduinoComboBox, self.arduinoPortNames)

        # Connect Serial Port Buttons
        self.LoRaRefreshButton.clicked.connect(lambda: self.refresh_ports(self.LoRaComboBox, self.loraPortNames))
        self.LoRaSelectButton.clicked.connect(self.start_lora_reader)
        self.ArduinoRefreshButton.clicked.connect(lambda: self.refresh_ports(self.ArduinoComboBox, self.arduinoPortNames))
        self.ArduinoSelectButton.clicked.connect(self.start_arduino)
        self.ClearSerialButton.clicked.connect(self.clear_serial)

        # Connect Adjustment Buttons
        self.UpButton.clicked.connect(self.tiltUp)
        self.DownButton.clicked.connect(self.tiltDown)
        self.log_signal.connect(self.display_data)
        self.ClockWiseButton.clicked.connect(self.panCounterClockwise)
        self.CounterClockeWiseButton.clicked.connect(self.panClockwise)

        # Connect SetUp Buttons
        self.setGSLocationButton.clicked.connect(self.setGSLocation)
        self.calculateStartingPosButton.clicked.connect(self.getStartingPos)
        self.returnToSunButton.clicked.connect(self.returnToSun)
        self.setStartingPosButton.clicked.connect(self.calibrate)

        # Connect Start and Stop Buttons
        self.startButton.clicked.connect(self.checkIfReady)
        self.stopButton.clicked.connect(self.stopTracking)
        self.EStopButton.clicked.connect(self.EStop)
        self.predictionStartButton.clicked.connect(self.setPredictTrack)


    def refresh_ports(self, combo_box, port_names_list):
        try:
            combo_box.clear()
            port_names_list.clear()
            ports = serial.tools.list_ports.comports()
            for port_info in sorted(ports, key=lambda p: p.device):
                combo_box.addItem(port_info.description)
                port_names_list.append(port_info.device)
        except Exception as e:
            print(f"Refresh ports error: {e}")
            self.statusBox.append(f"Refresh ports error: {e}")


    def start_lora_reader(self):
        # print("Select button clicked")
        selected_index = self.LoRaComboBox.currentIndex()

        if not self.LoRaListening and 0 <= selected_index < len(self.loraPortNames):
            selected_port = self.loraPortNames[selected_index]
            self.statusBox.setPlainText(f"LoRa Connecting to {selected_port}")
            try:
                self.reader = LoRaReader(selected_port, callback=self.log_signal)
                self.reader.start()
                self.LoRaListening = True
            except Exception as e:
                self.log_signal.emit(f"Error in LoRaReader: {e}")
            # print("LoRaReader thread Started")
        else:
            self.log_signal.emit("No valid LoRa port selected")


    def start_arduino(self):
        # checks if arduino is selected, and if the connection is not already made, instantiates an instance of
        # the Ground_Station_Arduino class
        # if an arduino is connected, or one is not selected, the function returns

        selected_index = self.ArduinoComboBox.currentIndex()

        if not self.arduinoConnected and 0 <= selected_index < self.ArduinoComboBox.count():
            try:
                selected_port = self.arduinoPortNames[selected_index]
                self.GSArduino = Ground_Station_Arduino(selected_port, 9600)
                print("After GSArduino init", self.GSArduino)
                self.AdjustmentLogBox.append("Connected to {selected_port}!")
                self.statusBox.append(f"Connected to {selected_port}!")
                self.arduinoConnected = True
            except Exception as e:
                self.statusBox.append(f"Failed to connect Arduino: {e}")
        elif self.arduinoConnected:
            self.AdjustmentLogBox.setPlainText("Arduino already connected")
            self.statusBox.append("Arduino already connected")
        else:
            self.statusBox.append("Unable to connect to Arduino")
            self.AdjustmentLogBox.setPlainText("Unable to connect to Arduino")
        return


    def tiltUp(self):
        # if an arduino is connected, uses GSArduino to adjust the tilt up
        if self.arduinoConnected:
            self.GSArduino.adjustTiltUp(self.degreesPerClickBox.currentText())
            self.AdjustmentLogBox.setPlainText("adjusting tilt up " + self.degreesPerClickBox.currentText() + " degrees")
        else:
            print("Unable to connect to ground station motors")
            self.AdjustmentLogBox.setPlainText("Not connected to ground station motors")

        return


    def tiltDown(self):
        # if an arduino is connected, uses GSArduino to adjust the tilt down
        if self.arduinoConnected:
            self.GSArduino.adjustTiltDown(self.degreesPerClickBox.currentText())
            self.AdjustmentLogBox.setPlainText("adjusting tilt down " + self.degreesPerClickBox.currentText() + " degrees")
        else:
            print("Unable to connect to ground station motors")
            self.AdjustmentLogBox.setPlainText("Not connected to ground station motors")

        return


    def panCounterClockwise(self):
        # if an arduino is connected, uses GSArduino to adjust the pan counter-clockwise
        if self.arduinoConnected:
            self.GSArduino.adjustPanNegative(self.degreesPerClickBox.currentText())
            self.AdjustmentLogBox.setPlainText("adjusting pan " + self.degreesPerClickBox.currentText() + " Counter Clockwise")
        else:
            print("Unable to connect to ground station motors")
            self.AdjustmentLogBox.setPlainText("Not connected to ground station motors")

        return


    def panClockwise(self):
        # if an arduino is connected, uses GSArduino to adjust the pan clockwise
        if self.arduinoConnected:
            self.GSArduino.adjustPanPositive(self.degreesPerClickBox.currentText())
            self.AdjustmentLogBox.setPlainText("adjusting pan " + self.degreesPerClickBox.currentText() + " Clockwise")
        else:
            print("Unable to connect to ground station motors")
            self.AdjustmentLogBox.setPlainText("Not connected to ground station motors")

        return


    def display_data(self, data):
        # Called from LoRaReader thread - must use signal-safe method
        self.statusBox.append(data)


    def setGSLocation(self):
        # this ensures that the arduino is connected, and valid text is present in the gs location text boxes
        # if the values present can be converted to floats, the starting location of the gs is set
        try:
            if self.arduinoConnected:
                latStr = self.GSLatBox.text().strip()
                self.GSLat = float(latStr)
                print(self.GSLat)

                longStr = self.GSLongBox.text().strip()
                self.GSLong = float(longStr)
                print(self.GSLong)

                altStr = self.GSAltBox.text().strip()
                self.GSAlt = float(altStr)
                print(self.GSAlt)

                self.statusBox.append("Ground station location entered successfully!")
                self.GSLocationSet = True
            else:
                self.statusBox.append("Please connect arduino")
                self.GSLocationSet = False
        except ValueError:
            print("numbers only for GPS location (decimal degrees)")
            self.statusBox.append("Invalid GPS location entered. Please only enter numbers")
        except Exception as e:
            self.statusBox.append(f"setGSLocation error: {e}")


    def getStartingPos(self):
        # this makes a call to sunposition to calculate the azimuth and elevation of the sun at the current location
        # of the ground station
        # it populates the starting aziumth and elevation boxes
        try:
            if self.GSLocationSet:
                now = datetime.datetime.now(tz=datetime.timezone.utc)
                az, elev = sunpos(now, self.GSLat, self.GSLong, self.GSAlt)[:2]  # discard RA, dec, H

                self.startingAzimuth = az
                self.startingElevation = elev

                self.startingAzimuthBox.setText(str(az))
                self.startingElevationBox.setText(str(elev))

            else:
                self.statusBox.append("Please set ground station location "
                                            "and point at the sun using solar sight")
        except Exception as e:
            self.statusBox.append(f"getStartingPos error: {e}")
            print("getStartingPos error: ", e)

        return


    def calibrate(self):
        # sends the GSArduino class the starting azimuth and elevation
        if self.arduinoConnected:
            try:
                startingAzimuthStr = self.startingAzimuthBox.toPlainText().strip()
                startingAzimuth = float(startingAzimuthStr)
                print(startingAzimuth)

                startingElevationStr = self.startingElevationBox.toPlainText().strip()
                startingElevation = float(startingElevationStr)
                print(startingElevation)

                self.GSArduino.calibrate(startingAzimuth, startingElevation)
                self.calibrated = True
                self.statusBox.append("Successfully calibrated!")
            except ValueError:
                print("numbers only for initial azimuth and elevation")
                self.statusBox.append("Invalid input for initial azimuth and elevation")
            except Exception as e:
                print(f"calibrate error: {e}")
                self.statusBox.append(f"calibrate error: {e}")
        else:
            print("not connected to arduino")
            self.statusBox.append("Not connected to arduino")

        return


    def returnToSun(self):
        if self.arduinoConnected and self.GSLocationSet and self.calibrated:
            now = datetime.datetime.now(tz=datetime.timezone.utc)
            az, elev = sunpos(now, self.GSLat, self.GSLong, self.GSAlt)[:2]  # discard RA, dec, H

            self.GSArduino.move_position(az, elev)

            self.startingAzimuth = az
            self.startingElevation = elev

            self.startingAzimuthBox.setPlainText(str(self.startingAzimuth))
            self.startingElevationBox.setPlainText(str(self.startingElevation))
            self.statusBox.append("at new sun position")

        else:
            self.statusBox.append("Ensure that arduino is connected, GS location is set and calibration is set")
            print("Cannot point back at the sun")

        return


    def setPredictTrack(self):
        # sets the predict track bool variable
        # then calls the checkIfReady function to ensure all conditions to track have been met
        self.predictingTrack = True
        self.checkIfReady()
        return


    def checkIfReady(self):
        # this function ensures that all conditions to track have been met
        # if they have been, it calls the appropriate function to either start tracking with/without predictions
        try:
            if self.calibrated:
                print("Calibrated!")
                # self.statusBox.append("Calibrated!")
            else:
                print("starting position not set")
                self.statusBox.append("Please set staring azimuth and elevation")

            if self.GSLocationSet:
                print("Ground station location set!")
            else:
                print("Ground Station Location not assigned")
                self.statusBox.append("Ground Station location not assigned")

            if self.LoRaListening:
                print("LoRa is listening!")
            else:
                print("LoRa is not listening!")
                self.statusBox.append("LoRa is not listening!")

            if self.arduinoConnected:
                print("Arduino connected!")
            else:
                print("Please connect to the Arduino")
                self.statusBox.append("Please connect to the Arduino")

            print("\n")

            if self.arduinoConnected and self.LoRaListening and self.calibrated and self.GSLocationSet:

                if self.predictingTrack:
                    self.statusBox.append("Starting tracking with predictions!")
                    self.callPredictTrack()
                else:
                    self.statusBox.append("Starting tracking!")
                    print("starting tracking!")
                    self.callTrack()
                    return True
            else:
                print("not ready to track yet")
                return False
        except Exception as e:
            print(f"checkIfReady error: {e}")


    def callTrack(self):
        # sets up the qt thread to start tracking, and starts the thread
        try:
            self.tracking = True
            self.statusBox.append("Tracking!")
            self.trackThread = QThread()
            self.worker = Worker(self.reader)

            self.worker.moveToThread(self.trackThread)

            self.trackThread.started.connect(self.worker.track)

            self.worker.finished.connect(self.trackThread.quit)  # pycharm has bug, this is correct
            self.worker.finished.connect(self.worker.deleteLater)  # https://youtrack.jetbrains.com/issue/PY-24183?_ga=2.240219907.1479555738.1625151876-2014881275.1622661488
            self.trackThread.finished.connect(self.trackThread.deleteLater)

            self.startButton.setEnabled(False)
            self.predictionStartButton.setEnabled(False)
            self.setStartingPosButton.setEnabled(False)

            self.trackThread.start()
        except Exception as e:
            print(f"callTrack error: {e}")


    def callPredictTrack(self):
        # sets up the qt thread to start tracking with predictions and starts the thread
        self.statusBox.append("Tracking with predictions!")
        print("In predictTrack call")
        self.tracking = True
        self.trackThread = QThread()
        self.worker = Worker(self.reader)

        self.worker.moveToThread(self.trackThread)

        self.trackThread.started.connect(self.worker.predictTrack)

        self.worker.finished.connect(self.trackThread.quit)  # pycharm has bug, this is correct
        self.worker.finished.connect(self.worker.deleteLater)  # https://youtrack.jetbrains.com/issue/PY-24183?_ga=2.240219907.1479555738.1625151876-2014881275.1622661488
        self.trackThread.finished.connect(self.trackThread.deleteLater)

        self.startButton.setEnabled(False)
        self.predictionStartButton.setEnabled(False)
        self.setStartingPosButton.setEnabled(False)

        self.trackThread.start()


    def stopTracking(self):
        # this stops the tracking thread, thus stopping the tracking
        if self.tracking:
            self.tracking = False
            self.predictingTrack = False
            self.startButton.setEnabled(True)
            self.predictionStartButton.setEnabled(True)
            self.setStartingPosButton.setEnabled(True)
            self.statusBox.append("tracking stopped")
        return


    def EStop(self):
        if self.arduinoConnected:
            self.GSArduino.sendEStop()
            self.stopTracking()

            self.statusBox.append("E-Stop triggered \n Please recalibrate before starting again")
            print("E-Stopped must recalibrate before starting tracking")

        return


    def displayCalculations(self, distance, azimuth, elevation):
        # this displays the outputs from the tracking threads on the GUI
        self.distanceDisplay.setPlainText(str(distance))
        self.azimuthDisplay.setPlainText(str(azimuth))
        self.elevationDisplay.setPlainText(str(elevation))
        return

    def closeEvent(self, event):
        if self.reader:
            self.reader.stop()
            self.reader.join()
        event.accept()

    def clear_serial(self):
        self.statusBox.clear()


class Worker(QObject):
    # worker class to track without making the GUI hang
    finished = pyqtSignal()
    calcSignal = pyqtSignal(float, float, float)


    def __init__(self, reader):
        super().__init__()
        self.reader = reader
        self.i = 0

    def track(self):
        # basic tracking algorithm
        # checks for updated position every 5 seconds
        # if a new position has been found, calculate the azimuth and elevation to point at the new location
        # send the motors a command to move to the new position

        print("LoRa data:", self.reader.data)
        print("GSArduino:", MainWindow.GSArduino)

        timer = time.time() - 4.0
        try:
            while MainWindow.tracking:

                if (time.time() - timer) > 5.0:
                    timer = time.time()
                    Balloon_Coor = [self.reader.data.latitude, self.reader.data.longitude, self.reader.data.altitude]
                    if not Balloon_Coor:
                        pass
                    else:
                        # note that trackMath takes arguments as lat, long, altitude
                        Tracking_Calc = trackMath(MainWindow.GSLat, MainWindow.GSLong, MainWindow.GSAlt, *Balloon_Coor)

                        distance = Tracking_Calc.distance
                        newElevation = Tracking_Calc.elevation()
                        newAzimuth = Tracking_Calc.azimuth()

                        print(str(self.i) + " Distance " + str(distance) + " Azimuth: " + str(
                            newAzimuth) + ", Elevation: " + str(newElevation))

                        self.calcSignal.connect(
                            MainWindow.displayCalculations)  # this seems to happen a lot for some reason
                        self.calcSignal.emit(distance, newAzimuth, newElevation)

                        MainWindow.GSArduino.move_position(newAzimuth, newElevation)

                        self.i += 1


            print("All done!")
            self.finished.emit()  # same pycharm bug as above
            return
        except Exception as e:
            print(f"error in tracking while {e}")

    def predictTrack(self):
        # check for new location from server
        # if the new location is still the same, go to prediction
        # if there is new location, go to that latest location

        # find the difference between the latest lat/long location and the one before the last one and time
        # using last vertical velocity/altitude, find difference between altitudes/new altitude after ~1 second
        # find the amount that the position is changing each ~second
        # input the new predicted lat/long/alt into math equations to get new azimuth/elevation
        # go to the predicted elevation/azimuth

        print("In predictTrack")

        timer = time.time()
        newestLocation = [self.reader.data.latitude, self.reader.data.longitude, self.reader.data.altitude]
        oldLocation = [self.reader.data.latitude, self.reader.data.longitude, self.reader.data.altitude]
        i = 1

        calculations = open("predictedOutput.csv", "w")
        csvWriter = csv.writer(calculations)
        calcFields = ["Distance", "Azimuth", "Elevation", "r/p"]
        csvWriter.writerow(calcFields)

        azimuthList = []
        elevationList = []

        while MainWindow.predictingTrack:
            if (time.time() - timer) > 1:
                timer = time.time()
                currData = [self.reader.data.latitude, self.reader.data.longitude, self.reader.data.altitude]

                if newestLocation == currData:
                    # need to predict!
                    print("predicted output")
                    timeDelta = MainWindow.Balloon.getTimeDiff()
                    print("The time delta is: " + str(timeDelta))
                    latStep = (newestLocation[0] - oldLocation[0]) / timeDelta
                    longStep = (newestLocation[1] - oldLocation[1]) / timeDelta
                    altStep = (newestLocation[2] - oldLocation[2]) / timeDelta

                    Tracking_Calc = trackMath(MainWindow.GSLat, MainWindow.GSLong, MainWindow.GSAlt,
                                              newestLocation[1] + (i * longStep), newestLocation[0] + (i * latStep),
                                              newestLocation[2] + (i * altStep))

                    distance = Tracking_Calc.distance
                    newElevation = Tracking_Calc.elevation()
                    newAzimuth = Tracking_Calc.azimuth()

                    elevationList.append(newElevation)
                    azimuthList.append(newAzimuth)

                    # keep average of azimuth/elevations
                    # if new calculation is outlier, throw it out, don't go to new spot
                    # reset average between pings
                    # alternatively, implement some type of filter (savitzky golay, kalman, etc)

                    """
                    if newElevation > np.mean(elevationList) + (2 * np.std(elevationList)) or newElevation < np.mean(elevationList) - (2 * np.std(elevationList)) \
                            or newAzimuth > np.mean(azimuthList) + (2 * np.std(azimuthList)) or newAzimuth < np.mean(azimuthList) - (2 * np.std(azimuthList)):
                        print("outlier detected! ")
                        pass
                    else:
                        print("distance: " + str(distance))
                        print("elevation: " + str(newElevation))
                        print("azimuth: " + str(newAzimuth) + "\n")
                    """
                    self.calcSignal.connect(MainWindow.displayCalculations)
                    self.calcSignal.emit(distance, newAzimuth, newElevation)

                    row = [distance, newAzimuth, newElevation, "p"]
                    csvWriter.writerow(row)

                    MainWindow.GSArduino.move_position(newAzimuth, newElevation)

                    i += 1

                else:
                    # go to the new actual spot
                    oldLocation = newestLocation
                    newestLocation = currData

                    # note that trackMath takes arguments as lat, long, alt
                    Tracking_Calc = trackMath(MainWindow.GSLat, MainWindow.GSLong, MainWindow.GSAlt, *currData)

                    distance = Tracking_Calc.distance
                    newElevation = Tracking_Calc.elevation()
                    newAzimuth = Tracking_Calc.azimuth()

                    self.calcSignal.connect(
                        MainWindow.displayCalculations)  # this seems to happen a lot for some reason
                    self.calcSignal.emit(distance, newAzimuth, newElevation)

                    print("Got new real ping!")
                    print("distance: " + str(distance))
                    print("elevation: " + str(newElevation))
                    print("azimuth: " + str(newAzimuth) + "\n")

                    MainWindow.GSArduino.move_position(newAzimuth, newElevation)

                    row = [distance, newAzimuth, newElevation, "r"]
                    csvWriter.writerow(row)

                    i = 1
                    azimuthList = []
                    elevationList = []

        print("All done tracking with predictions! :)")
        calculations.close()
        self.finished.emit()
        return
#


if __name__ == '__main__':
    os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
    app = QApplication(sys.argv)
    MainWindow = Window()
    MainWindow.showMaximized()
    app.exec_()

