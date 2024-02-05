import time
import os
import glob
import board
import adafruit_bno055
from bmp280 import BMP280
try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus
import busio
import adafruit_gps
import serial
import csv
import sys
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
from SX127x.LoRa import *
from SX127x.board_config import BOARD
from pyquaternion import Quaternion
import math
import numpy as np
from gpiozero import AngularServo


class LoRaBeacon(LoRa):



    def __init__(self, verbose=False):
        super(LoRaBeacon, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        # sleep to save power
        self.set_dio_mapping([1,0,0,0,0,0])
        #go to this web to read the doc: https://cdn-shop.adafruit.com/product-files/3179/sx1276_77_78_79.pdf

    def start(self):
        global args
        self.write_payload([])
        self.set_mode(MODE.TX)
        while True:
            time.sleep(1)

    def on_tx_done(self, dataToSent):
        self.set_mode(MODE.STDBY)
        self.clear_irq_flags(TxDone=1)
        sys.stdout.flush()
        time.sleep(2)
        data=dataToSent
        a=[int(hex(ord(m)), 0) for m in data]
        #set format array data in 1 byte
        print(a)
        self.write_payload(a)
        self.set_mode(MODE.TX)


class Cansat:
    def __init__(self):
        self.temp_c = 0
        self.pressure = 0
        self.time = 0
        self.longitude = 0
        self.latitude = 0
        self.altitude_from_pressure = 0
        self.altitude_from_GPS = 0
        self.data9DOF = {
            'acceleration': [0,0,0],
            'magnetic': [0,0,0],
            'angular velocity': [0,0,0],
            'euler angle': [0,0,0],
            'quaterion': [0,0,0,0],
            'linear acceleration': 0,
            'gravity': 0,
        }
        self.theta = 0
        self.phi = 0
        self.data = ""
        #self.lora = LoRaBeacon()

        self.speed = 1
        self.spin_time = 0.4
        self.direction = 1
        self.direction_change_frequency = 4
        self.spin_count = 0

        #self.radioFrequency = radioFrequency
        self.flightPhase = 'beforeStart'
        self.initializeGPS()
        self.initializeTemperatureSensor()
        self.initialize9DOF()
        self.initializePressureSensor()
        #self.initializeLoRa()
        self.pressureOnGround = self.getPressure()
        self.initializeServo(27,17)

        # starting a separate code that utilizes our nn model and
        # automatically detects new images in photos folder
        #os.system("python3 running.py &")


    def initializeTemperatureSensor(self):
        # turns on the temperature sensor
        os.system('modprobe w1-gpio')
        os.system('modprobe w1-therm')

        base_dir = '/sys/bus/w1/devices/'
        device_folder = glob.glob(base_dir + '28*')[0]
        self.device_file = device_folder + '/w1_slave'

    def getTemperature(self):
        # returns temperature in Celsius degrees
        f = open(self.device_file, 'r')
        lines = f.readlines()
        f.close()
        while lines[0].strip()[-3:] != 'YES':

            f = open(self.device_file, 'r')
            lines = f.readlines()
            f.close()
        equals_pos = lines[1].find('t=')
        if equals_pos != -1:
            temp_string = lines[1][equals_pos+2:]
            self.temp_c = float(temp_string) / 1000.0
            return self.temp_c
        return -100

    def initializePressureSensor(self):
        bus = SMBus(1)
        self.bmp280 = BMP280(i2c_dev=bus)

    def getPressure(self):
        self.pressure =  self.bmp280.get_pressure()
        return self.pressure


    def initializeGPS(self):
        # initializes GPS module
        uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=3000)
        self.gps = adafruit_gps.GPS(uart)
        self.gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        self.gps.send_command(b"PMTK220,1000")

    def getGpsCoordinates(self):
        # get gps coordiantes of CanSat
        timestamp = time.monotonic()
        while True:
            self.data = self.gps.read(32)  # read up to 32 bytes
            #print(data)  # this is a bytearray type

            self.readGPSData(str(self.data))

            if self.data is not None:
                # convert bytearray to string
                data_string = "".join([chr(b) for b in self.data])
                return data_string


            if time.monotonic() - timestamp > 5:
                # every 5 seconds...
                self.gps.send_command(b"PMTK605")  # request firmware version
                timestamp = time.monotonic()

    def processGPSData(self, txt):
        txt = txt.replace('b\'','')
        txt = txt.replace('\'','')
        txt_splited = txt.split(",")
        if txt_splited[0] == '$GNRMC':
            time = float(txt_splited[1])
            long = float(txt_splited[3])
            lat = float(txt_splited[5])
            return time, long, lat
        return None, None, None

    def readGPSData(self, txt):
        time, long, lat = self.processGPSData(txt)
        self.time = time
        self.longitude = long
        self.latitude  = lat
        print("GNMRC")
        print(time)
        print(long)
        print(lat)


    def getHeightFromPressure(self): #fuer Rzonca
        p = self.pressure
        g = 9.814 #the change of g over 3km is negligibly small
        d = 1.293 #assumption of air density
        self.altitude_from_pressure = (self.pressureOnGround-p) / (d *g)
        return self.altitude_from_pressure

    def getHeightFromGPS(self): #height is from the ground
        if self.gps.height_geoid is not None:
            self.altitude_from_GPS=self.gps.height_geoid

    def initialize9DOF(self):
        # turns ot the IMU sensor
        i2c = board.I2C()  # uses board.SCL and board.SDA
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        last_val = 0xFFFF

    def get9DOF(self):
        # return data from 9DOF sensor
        self.data9DOF = {
            'acceleration': self.sensor.acceleration,
            'magnetic': self.sensor.magnetic,
            'angular velocity': self.sensor.gyro,
            'euler angle': self.sensor.euler,
            'quaterion': self.sensor.quaternion,
            'linear acceleration': self.sensor.linear_acceleration,
            'gravity': self.sensor.gravity,
        }
        return self.data9DOF

    def getPhoto(self):
        return

    def setCameraOrientation(self, azimuth, altitude):
        return


    def calculateGroundCoordiantes(self):
        q1 = Quaternion(axis=[1, 0, 0], angle=self.theta) #ToDo: here should be first angle from Klara
        q2 = Quaternion(axis=[0, 0, 1], angle=self.phi) #ToDo: here should be second angle from Klara
        q3 = Quaternion(self.sensor.quaternion)
        qf = q1*q2*q3
        v_prime = qf.rotate([0, 0, -1])
        if v_prime[2] >= 0:
            return np.nan
        t = -self.altitude_from_pressure / v_prime[2]
        dx = v_prime[0] * t
        dy = v_prime[1] * t
        r = 6371000
        latitude = self.latitude + dy / r / math.pi * 180
        longitude = self.longitude + dx / r / math.cos(self.latitude/180*math.pi) / math.pi * 180
        return latitude, longitude


    def initializeLoRa(self):
        self.lora = LoRaBeacon(verbose=False)
        self.lora.set_pa_config(pa_select=1)
        if(self.lora.get_agc_auto_on() == 1):
            assert(self.lora.get_agc_auto_on() == 1)

    def sendData(self):
        try:
            self.lora.start()
        except KeyboardInterrupt:
            sys.stdout.flush()
            sys.stderr.write("KeyboardInterrupt\n")
        #print the transmitted values on the console and terminate the program using a keyboard interrupt
        finally:
            sys.stdout.flush()
            self.lora.set_mode(MODE.SLEEP)
            BOARD.teardown()
            GPIO.cleanup()
            print("Clean-uped")
            return


    def saveToSdCard(self, file_name):
        f = open(file_name, 'a')
        row = [self.temp_c, self.pressure, [i for i in self.data9DOF.values()], self.data]
        #if self.isFlightModeOn:
        #    row = [self.data_string]
        # create the csv writer
        writer = csv.writer(f)

        # write a row to the csv file
        writer.writerow(row)

        # close the file
        f.close()
        return

    def isFlightModeOn(self):
        return True

    #def getFlightPhase(self):
    #    return self.flightPhase

    def setFlightPhase(self, flightPhase):
        self.flightPhase = flightPhase

    def turnToStationaryMode(self):
        return

    def initializeServo(self, servo_180_pin, servo_360_pin):
        self.servo180 =AngularServo(servo_180_pin, min_angle=0, max_angle=270,
                                 min_pulse_width=0.0005, max_pulse_width=0.0025)
        GPIO.setup(servo_360_pin,GPIO.OUT)
        self.servo360 = GPIO.PWM(servo_360_pin, 50)
        self.servo360.start(0)

    def setCameraAngle(self, theta):
        self.servo180.angle = theta


    def setCameraSpeed(self, servo360_speed):
        # servo360_speed must be in range 2 to 12, where 2 is highest speed
        # anticlockwise, 7 is stop, and 12 is the highiest speed clockwise
        self.servo360.ChangeDutyCycle(servo360_speed)


    def hideCamera(self):
        self.setCameraAngle(0)


    def takePhoto(self, width, height, photo_name):
        os.system("libcamera-still --width " + str(width) + " --height " +
                  str(height) + " -o "+ photo_name + " &")


    def rotateCamera(self):
        servo_speed = 7 + self.direction * self.speed
        self.setCameraSpeed(servo_speed)
        time.sleep(self.spin_time)
        self.setCameraSpeed(7)
        self.spin_count = self.spin_count + 1
        if self.spin_count == self.direction_change_frequency:
            self.direction = -self.direction
            self.spin_count = 0









cansat = Cansat()
cansat.hideCamera()
photo_id = 1
last_photo_time = 0
directory = "photo_folder"
cansat.takePhoto(3840, 2160, 'photo')
run_test = True
phase1_start = time.time()
print("Entering phase 1")
while True:                                     # WAITING-FOR-THE-FLIGHT MODE
    temperature = cansat.getTemperature()
    pressure = cansat.getPressure()
    height = cansat.getHeightFromPressure()
    #cansat.sendData()

    print("height: ", height)
    print("Pressure: ", pressure)
    print("Temperature: ", temperature)

    cansat.saveToSdCard("data.csv")

    if run_test and time.time()-phase1_start>60:
        break
    if not run_test and height > 200:
        break


print("Entering phase 2")
if run_test:
    cansat.setCameraAngle(45)
max_height = 0
phase2_start = time.time()
while cansat.isFlightModeOn():
    start_time = time.time()
    temperature = cansat.getTemperature()
    pressure = cansat.getPressure()
    imu = cansat.get9DOF()
    height = cansat.getHeightFromPressure()

    if max_height < height:
        max_height = height
    elif height < max_height - 50:
        cansat.setCameraAngle(45)


    coordinates = cansat.getGpsCoordinates()
    cansat.sendData()

    print("height: ", height)
    print("Pressure: ", pressure)
    print(imu)
    print("Temperature: ", temperature)
    print(coordinates)

    time_since_last_photo = time.time() - last_photo_time
    if time_since_last_photo > 10:
       cansat.rotateCamera()
       #cansat.takePhoto(4624, 3472, directory + "/photo_" + photo_id)
       cansat.takePhoto(3840, 2160, directory + "/photo_" + str (photo_id))
       photo_id = photo_id + 1
       last_photo_time = time.time()


    cansat.saveToSdCard("data.csv")
    change_time = time.time() - start_time
    if change_time < 1:
        time.sleep(1 - change_time)

    if height > 50:
        cansat.hideCamera()

    if run_test and time.time()-phase2_start>60:
        break
    if not run_test and height > 50:
        break

print("Entering phase 3")
phase3_start = time.time()
while True:                                     # WAITING-FOR-THE-FLIGHT MODE
    coordinates = cansat.getGpsCoordinates()
    temperature = cansat.getTemperature()
    pressure = cansat.getPressure()
    height = cansat.getHeightFromPressure()
    print("height: ", height)
    print("Pressure: ", pressure)
    print("Temperature: ", temperature)
    print(coordinates)
    if run_test and time.time()-phase3_start>60:
        break
    if not run_test and height < 50:
        break
    cansat.sendData()


    cansat.saveToSdCard("data.csv")