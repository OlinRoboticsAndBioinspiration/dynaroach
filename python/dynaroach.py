'''
File: dynaroach.py
Author: Aaron M. Hoover
Date: 2012-05-03
Description: A class representing the functionality necessary for testing and
running the dynaRoACH robot.
'''

import sys
import time
import math

from serial import Serial, SerialException
import numpy as np

from struct import pack, unpack
from operator import attrgetter

from lib import cmd
from lib.basestation import BaseStation
from lib.payload import Payload

DEFAULT_BAUD_RATE = 230400

DEFAULT_DEST_ADDR = '\x00\x12'
DEFAULT_DEV_NAME = '/dev/ttyUSB0' #Dev ID for ORANGE antenna base station

SMA_RIGHT = 0
SMA_LEFT =  1

GYRO_LSB2DEG = 0.0695652174  # 14.375 LSB/(deg/s)
GYRO_LSB2RAD = 0.00121414209

DFMEM_PAGE_SIZE = 264
SAMPLE_BYTES = 35

TICKS_PER_MILLI     = 625.0
XL_CNTS_PER_G       = 256.0
G                   = 9.81
BEMF_VOLTS_PER_CNT  = 3.3/512
VBATT_VOLTS_PER_CNT = 3.3/512

ACCEL_MIN = [(-190,-10,120),(0,-195,125)]
ACCEL_MAX = [(-180,0,130),(10,-185,140)]

GYRO_MAX = [(
GYRO_MIN =



class DynaRoach(object):
    '''Class representing the dynaRoACH robot'''

    def __init__(self, dev_name=DEFAULT_DEV_NAME, baud_rate=DEFAULT_BAUD_RATE, dest_addr=DEFAULT_DEST_ADDR):
        '''
        Description:
        Initiate the 802.15.4 connection and configure the target.
        Class must be instantiated with a connection name. On Windows this is
        typically of the form "COMX". On Mac, the serial device connection
        string is typically '/dev/tty.DEVICE_NAME-SPP-(1)' where the number at
        the end is optional depending on the version of the OS.


        Inputs:
            dev_name: The device name to connect to. Examples are COM5 or
                      /dev/tty.usbserial.
                print ord(datum)
        '''

        self.dest_addr = dest_addr
        self.last_packet = None
        self.data_cnt = 0
        self.state_data = []
        self.last_sample_count = 0

        self.radio = BaseStation(dev_name, baud_rate, dest_addr, self.receive)
        self.receive_callback = []

        self.acc_res = [(0,0,0),(0,0,0)]
        self.gyro_res = [(0,0,0),(0,0,0)]

    def add_receive_callback(self, callback):
        self.receive_callback.append(callback)

    def receive(self, packet):
        self.last_packet = packet
        pld = Payload(packet.get('rf_data'))
        typeID = pld.type
        data = pld.data
        for callback in self.receive_callback:
            callback(pld)
        if typeID == cmd.TEST_ACCEL:
            self.acc_res = unpack('<3h', data)
        elif typeID == cmd.TEST_GYRO:
            self.gyro_res = unpack('<3h', data)
        elif typeID == cmd.TEST_DFLASH:
            print ''.join(data)
        elif typeID == cmd.TEST_BATT:
            print unpack('2H', data)
        elif typeID == cmd.TX_SAVED_DATA:
            datum = list(unpack('<L3f3h2HB4H', data))
            self.state_data.append(datum)
            self.data_cnt += 1
            if self.data_cnt % 100 == 0:
                print self.data_cnt, "/", self.last_sample_count
        elif typeID == cmd.GET_SAMPLE_COUNT:
            self.last_sample_count = unpack('H', data)[0]
            print('Last sample count %d' % self.last_sample_count)
        elif typeID == cmd.GET_GYRO_CALIB_PARAM:
            self.gyro_offsets = list(unpack('<fff', data))
            print(self.gyro_offsets)
        elif cmd.DATA_STREAMING:
            if (len(data) == 35):
              datum = list(unpack('<L3f3h2HB4H', data))
              # print datum[6:]

    def echo(self):
        '''
        Description:
            This test sends three packets to the target requesting
            it echo them back. The results should be the
            receipt of three packets. The payloads of those three packets
            should print as consecutive integers 0-9, 10-19, and 20-29
            respectively.
        '''

        for i in range(1, 4):
            self.last_packet = None
            data_out = ''.join([chr(datum) for datum in range((i-1)*10,i*10)])
            print("Transmitting data " + str(i) + "...")
            self.radio.send(0, cmd.ECHO, data_out)
            time.sleep(0.3)

            packet= self.last_packet
            assert (packet is not None), "Radio test failed. No packet received"
            pld = Payload(packet.get('rf_data'))
            typeID = pld.type
            data = pld.data
            assert (data ==data_out), "Radio test failed. Incorrect data"
            packet = self.last_packet

            assert (packet is not None), "Radio test failed. No packet received."

            pld = Payload(packet.get('rf_data'))
            data = pld.data

            assert (data == data_out), "Radio test failed, incorrect data received."
            print('\n')
            print('\n')
            time.sleep(1)

        print ("Radio working.")

    def set_motor_config(self, rising_duty_cycle, falling_duty_cycle):
      '''
      Set the motor config for rising and falling strides.
      Rising_duty_cycle and falling_duty_cycle are floats from 0-1
      '''
      max_int = 2**15;
      cmd_data = str(pack('h', int(max_int * rising_duty_cycle)))
      cmd_data += str(pack('h', int(max_int * falling_duty_cycle)))
      self.radio.send(cmd.STATUS_UNUSED, cmd.MOTOR_CONFIG, cmd_data)
      time.sleep(0.5)

    def set_data_streaming(self, val):
      data = ''.join(chr(val));
      self.radio.send(cmd.STATUS_UNUSED, cmd.DATA_STREAMING, data)
      time.sleep(0.5)

    def reset(self, do_wait=True):
      self.radio.send(cmd.STATUS_UNUSED, cmd.RESET, [])
      if do_wait:
        time.sleep(11)
    
    def configure_trial(self, trial):
        '''
            Description:
                Configure trial parameters.
            Parameters:
                trial: The trial configuration to be saved.
        '''

        data_out = trial.to_cmd_data()
        print("Configuring trial...")
        self.radio.send(cmd.STATUS_UNUSED, cmd.CONFIG_TRIAL, data_out)

    def configure_sma(self, sma):
        '''
            Description:
                Configure SMA actuation parameters
            Parameters:
                sma: the sma configuration to be saved. 
        '''

        data_out = sma.to_cmd_data()
        self.state_data = []
        self.data_cnt = 0
        self.radio.send(cmd.STATUS_UNUSED, cmd.CONFIG_SMA, data_out)

    def run_trial(self):
        '''
            Description:
                Start a trial running.
            Parameters:
                trial: The configured trial to be executed.
        '''

        self.radio.send(cmd.STATUS_UNUSED, cmd.RUN_TRIAL, [])

    def run_gyro_calib(self, num_samples='2000'):
        print("Calibrating gyroscope...")
        self.radio.send(cmd.STATUS_UNUSED, cmd.RUN_GYRO_CALIB, [num_samples])

    def get_gyro_calib_param(self):
        print("Requesting gyro calibration parameters...")
        self.radio.send(cmd.STATUS_UNUSED, cmd.GET_GYRO_CALIB_PARAM, [])
        self.gyro_offsets = None

    def test_gyro(self):
        '''
        Description:
            Read the XYZ values from the gyroscope.
        '''

        print("Testing gyroscope...")
        for i in range(0,3):
			gyro_res = None;
            print("Put the board into the slit position x=1, y=2, z=3"+ str(i+1))
            time.sleep(2)
            self.radio.send(cmd.STATUS_UNUSED, cmd.TEST_GYRO, [])
            time.sleep(.3)
            #assert (self.gyro_res <= GYRO_MAX[i]),"Test failed, x value is invalid"
            #assert (self.gyro_res >= GYRO_MIN[i]), "Test failed, y value is invalid"
            #assert (self.gyro_res <= GYRO_MAX
			assert (gyro_res is not None), "Check the radio. No packet received"

    def test_accel(self):
        '''
        Description:
            Read the XYZ values from the accelerometer.
        '''

        print("Testing accelerometer...")
        for i in range(0,2):
            print("Place the board in position "+ str(i+1))
            time.sleep(2)
            self.radio.send(cmd.STATUS_UNUSED, cmd.TEST_ACCEL, [])
            time.sleep(.3)
            assert (self.acc_res <= ACCEL_MAX[i]),"Test failed, accelerometer reading too high."
            assert (self.acc_res >= ACCEL_MIN[i]), "Test failed, accelerometer reading too low."
			assert (acc_res is not None), "Check the radio. No packet received"
        print("Accelerometer working.")

    def test_dflash(self):
        '''
        Description:
            Read out a set of strings that have been written to and read from
            memory.
        '''

        print("Testing data flash...")
        self.radio.send(cmd.STATUS_UNUSED, cmd.TEST_DFLASH, [])

    def test_motor(self, motor_id, time, duty_cycle, direction, return_emf=0):
        '''
        Description:
           Turn on a motor.
       Parameters:
            motor_id    : The motor number to turn on
            time        : The amount of time to turn the motor on for (in
                             seconds)
            duty_cycle  : The duty cycle of the PWM signal used to control the
                             motor in percent (0 - 100) 
            direction   : The direction to spin the motor. There are *three*
                             options for this parameter. 0 - reverse, 1 - forward, 
                             2 high impedance motor controller output = braking
            return_emf  : Send the back emf readings over the radio channel.
        '''
    
        if direction >= 2:
            direction = 2
        elif direction <= 0:
            direction = 0
        else:
            direction = 1
    
        if return_emf != 1:
            return_emf = 0
    
        data_out = chr(cmd.STATUS_UNUSED) + chr(cmd.TEST_MOTOR) + chr(motor_id) + \
                    chr(time) + chr(duty_cycle) + chr(direction) + \
                    chr(return_emf)
        if(self.check_conn()):
            self.radio.tx(dest_addr=self.dest_addr, data=data_out)

#    def test_sma(self, chan_id, time, duty_cycle):
#        '''
#        Description:
#            Turn on an SMA
#        Parameters:
#            chan_id     : The SMA channel to turn on
#            time        : The amount of time to turn the SMA on for (in
#                          seconds)
#            duty_cycle  : The duty cycle of the PWM signal used to control the
#                          SMA in percent (0 - 100)
#        '''
#
#        if(duty_cycle < 0 or duty_cycle > 100):
#            print("You entered an invalid duty cycle.")
#            return
#
#        data_out = chr(cmd.STATUS_UNUSED) + chr(cmd.TEST_SMA) + chr(chan_id) + \
#                   chr(time) + chr(duty_cycle)
#
#        if(self.check_conn()):
#            self.radio.tx(dest_addr=self.dest_addr, data=data_out)

    def test_hall(self):
        self.radio.send(cmd.STATUS_UNUSED, cmd.TEST_HALL, [])

    def test_batt(self):
        data = ''.join(chr(0) for i in range(4))
        self.radio.send(cmd.STATUS_UNUSED, cmd.TEST_BATT, data)

    def get_sample_count(self):
        self.radio.send(cmd.STATUS_UNUSED, cmd.GET_SAMPLE_COUNT, pack('H', 0))

    def transmit_saved_data(self):
        if(self.last_sample_count == 0):
            self.get_sample_count()
            time.sleep(0.5)

        if(self.last_sample_count == 0):
            print("There is no previously saved data.")
            return
        else:
            self.data_cnt = 0
            start_page = 0x200
            print("Transmitting saved data...")
            self.state_data = []
            self.data_cnt = 0
            self.radio.send(cmd.STATUS_UNUSED, cmd.TX_SAVED_DATA, pack('3H', start_page, self.last_sample_count, SAMPLE_BYTES))

    def erase_mem_sector(self, sector):
        print("Erasing memory sector " + str(sector))
        self.radio.send(cmd.STATUS_UNUSED, cmd.ERASE_MEM_SECTOR, pack('H', sector))

    def print_packet(self, packet):
        '''
        Description:
            Print the contents of a packet to the screen. This function
            will primarily be used for debugging purposes. May need to
            replace print with stdio or stderr to accommodate GUI test
            suite.
        '''
        if(packet is not None):
            print("Received the following: ")
            print("RSSI: " + str(ord(packet.get('rssi'))))
            (src_h, src_l) = unpack('cc', packet.get('source_addr'))
            print("Source Address: 0x%.2X 0x%.2X" % (ord(src_h),
                  ord(src_l)))
            print("ID: " + (packet.get('id')))
            print("Options: " + str(ord(packet.get('options'))))
            rf_data = packet.get('rf_data')
            print("Status Field: " + str(ord(rf_data[0])))
            print("Type Field: " + str(ord(rf_data[1])))
            print("Payload Data: " + ''.join([str(ord(i)) for i in rf_data[2:]]))

    def save_trial_data(self, fname):
        print('%d total state data were received.' % self.data_cnt)
        state_data_arr = np.array(self.state_data)
        print state_data_arr 
        fmt = '%d, %f, %f, %f, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d'
        np.savetxt(fname, state_data_arr, fmt)
        #np.savetxt(self.datestring() + ".csv", state_data_arr, fmt)
        print("State data saved to file: " + fname)

    def __del__(self):
        '''
        Description:
            Clean up the connection when the object is deleted.
        '''
        try:
            self.radio.close()
        except AttributeError:
            print('Caught Attribute Error')
            pass

class Trial():
    def __init__(self):
        self.state_transitions = []
        self.save_data = True

    def to_cmd_data(self):
        self.state_transitions = sorted(self.state_transitions,
                     key=attrgetter('timestamp'))
        cmd_data = pack('B', self.save_data)
        for st in self.state_transitions:
            cmd_data += pack('H', st.timestamp)
            cmd_data += pack('B', st.cmd)
            for param in st.params:
                cmd_data += pack('B', param)
        return cmd_data

    def save_to_file(self, fname, **kwds):
        outfile = open(fname + '.csv', 'w')
        outfile.write('%d\n' % self.save_data)
        for t in self.state_transitions:
            outfile.write('%d, %d, %d, %d \n' % (t.timestamp, t.cmd, t.params[0], \
                     t.params[1]))
        outfile.flush()
        outfile.close()

        outfile = open(fname + '.py', 'w')
        outfile.write('%s' % kwds)
        outfile.close()

    def add_state_transition(self, state_transition):
      self.state_transitions.append(state_transition)

    def load_from_file(self, fname):
        try:
            infile = open(fname, 'r')
            self.state_transitions = []
            self.save_data = False
            self.save_data = (bool)((int)(infile.readline().rstrip('\n')))
            for line in infile.readlines():
                st = line.rstrip('\n').split(',')
                print(st)
                if (int(st[1]) == cmd.MOTOR_CONFIG):
                  m1 = float(st[2])
                  m2 = float(st[3])
                  fixed_max = (2**15 - 1)
                  i1 = fixed_max * m1
                  i2 = fixed_max * m2

                  i1_0 = i1%256
                  i1_1 = i1/256

                  i2_0 = i2%256
                  i2_1 = i2/256

                  self.state_transitions.append(StateTransition(int(st[0]),\
                                                              int(st[1]),\
                                                              [i1_0,\
                                                              i1_1, i2_0, i2_1]))
                else:
                  self.state_transitions.append(StateTransition(int(st[0]),\
                                                              int(st[1]),\
                                                              [int(st[2]),\
                                                              int(st[3])]))
        except IOError:
            'File doesn\'t exist. Try again.'

class MCUData():
    '''
        Description:
            A class for reading MCUData from standard files resulting from
            saving data from a trial.
    '''

    #TODO: Is there a better way to share constants across Python/C?
    TICKS_PER_MS = 625.0
    XL_DEFAULT_SCALE = 0.03832 #9.81/256
    BEMF_SCALE = 0.00403 #4.125/1024
    VBATT_SCALE = 0.006445 #3.3/1024 * 2
    GYRO_SCALE = 0.0695652174 #LSB/(deg/s)


    def __init__(self):
        self.time = []
        self.pose = []
        self.accel = []
        self.bemf = []
        self.phase = []
        self.hall_state = []
        self.v_batt = []
        self.raw_gyro =[]
        pass

    def load_from_file(self, fname):
        '''
            Description:
                Read in MCU state data from a file generated by a trial run of
                the robot.

            Inputs:
                fname: The name of the file to load the data from.
        '''
        try:
            data = np.genfromtxt(fname, delimiter=',')
            self.time = data[:,0] / MCUData.TICKS_PER_MS
            self.pose = np.degrees(data[:,1:4])
            self.accel = np.array(data[:,4:7], dtype=np.short) * MCUData.XL_DEFAULT_SCALE
            self.bemf = data[:,7:8] * MCUData.BEMF_SCALE
            self.phase = data[:,8:9]
            self.hall_state = data[:,9:10]
            self.v_batt = data[:,10:11] * MCUData.VBATT_SCALE
            self.raw_gyro = data[:,11:] * MCUData.GYRO_SCALE

        except IOError:
            'File does not exist. Try again.'

class StateTransition():

    def __init__(self, ts=0, cmd=0, params=[]):
        self.timestamp = ts
        self.cmd = cmd
        self.params = params

def datestring():
  """
  Datestring

  by Sam Burden, Shai Revzen 2012
  """
  t = time.localtime()

  ye = '%04d'%t.tm_year
  mo = '%02d'%t.tm_mon
  da = '%02d'%t.tm_mday
  ho = '%02d'%t.tm_hour
  mi = '%02d'%t.tm_min

  return ye+mo+da+'-'+ho+mi


