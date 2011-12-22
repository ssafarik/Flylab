#!/usr/bin/env python
#
# MotorController.py
#
# Control interface for at90usb based step and direction motor controller board.
# Provides python module and a command line utility.
#
# Note, need to set permissions correctly to get device to respond to nonroot
# users. This required adding and rules file to udev/rules.d and adding a
# group.
#
#    who when        what
#    --- ----        ----
#    pjp 08/19/09    version 1.0
# ---------------------------------------------------------------------------
from __future__ import division

import roslib; roslib.load_manifest('stage')
import rospy

import USBDevice
import ctypes
import time
import math
import numpy
import threading

# Motorcontroller device parameters
_motor_num = 3
_entries_max = 5
_lookup_table_size = 100
_lookup_table_update_freq = 62.5

# Input/Output Structures
class MotorState_t(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ =[('Frequency', ctypes.c_uint16),
               ('Position', ctypes.c_uint16)]

class LookupTableRow_t(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ =[('Motor', MotorState_t * _motor_num)]

class USBPacketOut_t(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ =[('MotorUpdate', ctypes.c_uint8),
               ('EntryCount', ctypes.c_uint8),
               ('EntryLocation', ctypes.c_uint8),
               ('Entry', LookupTableRow_t * _entries_max)]

class USBPacketIn_t(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ =[('AllMotorsInPosition', ctypes.c_uint8),
               ('LookupTableMoveInProgress', ctypes.c_uint8),
               ('HomeInProgress', ctypes.c_uint8),
               ('MotorsHomed', ctypes.c_uint8),
               ('State', LookupTableRow_t)]

class MotorControllerDevice(USBDevice.USB_Device):
    def __init__(self,frequency_max,position_min,position_max,serial_number=None):

        self.frequency_max = frequency_max
        self.position_min = position_min
        self.position_max = position_max

        # USB device parameters
        self.vendor_id = 0x0004
        self.product_id = 0x0002
        self.bulkout_ep_address = 0x01
        self.bulkin_ep_address = 0x82
        self.buffer_out_size = 64
        self.buffer_in_size = 64
        self.serial_number = serial_number

        USBDevice.USB_Device.__init__(self,
                                      self.vendor_id,
                                      self.product_id,
                                      self.bulkout_ep_address,
                                      self.bulkin_ep_address,
                                      self.buffer_out_size,
                                      self.buffer_in_size,
                                      self.serial_number)

        # USB Command IDs
        self.USB_CMD_GET_STATE = ctypes.c_uint8(1)
        self.USB_CMD_SET_STATE = ctypes.c_uint8(2)
        self.USB_CMD_HOME = ctypes.c_uint8(3)
        self.USB_CMD_LOOKUP_TABLE_FILL = ctypes.c_uint8(4)
        self.USB_CMD_LOOKUP_TABLE_POS_MOVE = ctypes.c_uint8(5)
        self.USB_CMD_LOOKUP_TABLE_VEL_MOVE = ctypes.c_uint8(6)

        self.USBPacketOut = USBPacketOut_t()
        self.USBPacketIn = USBPacketIn_t()

        self.reentrant_lock = threading.Lock()

    def get_state(self):
        self._get_motor_state()
        state = self._return_state()
        return state

    def update_velocity(self,motor0_vel,motor1_vel,vel_mag):
        motor0_vel_list = list(motor0_vel)
        motor1_vel_list = list(motor1_vel)
        vel_mag_list = list(vel_mag)
        vel_mag_list,point_count = self._condition_vel_mag_list(motor0_vel_list,motor1_vel_list,vel_mag_list)
        if 0 < len(vel_mag_list):
            for point_n in range(point_count):
                vel_norm = numpy.linalg.norm([motor0_vel_list[point_n],motor1_vel_list[point_n]])
                if 0 < vel_norm:
                    vel_scale = vel_mag_list[point_n]/vel_norm
                    motor0_vel_list[point_n] *= vel_scale
                    motor1_vel_list[point_n] *= vel_scale
                else:
                    motor0_vel_list[point_n] = 0
                    motor1_vel_list[point_n] = 0

        state = self._send_cmds_receive_state(None,motor0_vel_list,None,motor1_vel_list)
        return state

    def update_position(self,motor0_pos,motor1_pos,vel_mag):
        motor0_pos_list = list(motor0_pos)
        motor1_pos_list = list(motor1_pos)
        vel_mag_list = list(vel_mag)
        state = self.get_state()
        motor0_initial_pos = state[0]
        motor1_initial_pos = state[2]
        motor0_vel_list,motor1_vel_list = self._find_velocity_from_position(motor0_pos_list,motor1_pos_list,vel_mag_list,motor0_initial_pos,motor1_initial_pos)
        
        state = self._send_cmds_receive_state(motor0_pos_list,motor0_vel_list,motor1_pos_list,motor1_vel_list)
        return state

    def home(self):
        self.USBPacketOut.MotorUpdate = ctypes.c_uint8(7)
        self._send_usb_cmd(self.USB_CMD_HOME,True)
        state = self._return_state()
        return state

    def _condition_vel_mag_list(self,motor0_list,motor1_list,vel_mag_list):
        point_count = min(len(motor0_list),len(motor1_list))
        if (0 < len(vel_mag_list)) and (len(vel_mag_list) < point_count):
            vel_mag_list = [vel_mag_list[0]]*point_count
        vel_mag_list = [abs(vel_mag) for vel_mag in vel_mag_list]
        return vel_mag_list, point_count

    def _find_velocity_from_position(self,motor0_pos_list,motor1_pos_list,vel_mag_list,motor0_initial_pos,motor1_initial_pos):
        motor0_vel_list = []
        motor1_vel_list = []
        vel_mag_list,point_count = self._condition_vel_mag_list(motor0_pos_list,motor1_pos_list,vel_mag_list)

        for point_n in range(point_count):
            if point_n == 0:
                delta_x = abs(motor0_pos_list[point_n] - motor0_initial_pos)
                delta_y = abs(motor1_pos_list[point_n] - motor1_initial_pos)
            else:
                delta_x = abs(motor0_pos_list[point_n] - motor0_pos_list[point_n - 1])
                delta_y = abs(motor1_pos_list[point_n] - motor1_pos_list[point_n - 1])
            try:
                alpha = math.sqrt((vel_mag_list[point_n]**2)/(delta_x**2 + delta_y**2))
            except ZeroDivisionError:
                alpha = 0
            motor0_vel_list.append(alpha*delta_x)
            motor1_vel_list.append(alpha*delta_y)
        return motor0_vel_list,motor1_vel_list

    def _send_cmds_receive_state(self,motor0_pos_list,motor0_vel_list,motor1_pos_list,motor1_vel_list):
        point_count = min(len(motor0_vel_list),len(motor1_vel_list))
        if _lookup_table_size < point_count:
            point_count = _lookup_table_size

        if (motor0_pos_list is None) or (motor1_pos_list is None):
            motor0_pos_list = [None]*point_count
            motor1_pos_list = [None]*point_count
            vel_move = True
        else:
            vel_move = False

        if point_count == 1:
            motor0_pos = motor0_pos_list[0]
            motor0_vel = motor0_vel_list[0]
            motor1_pos = motor1_pos_list[0]
            motor1_vel = motor1_vel_list[0]
            self._convert_and_set_setpoint(motor0_pos,motor0_vel,motor1_pos,motor1_vel,0)
            self._set_motor_state()
        else:
            packet_count = int(math.ceil(point_count/_entries_max))
            point_n = 0
            for packet_n in range(packet_count):
                packet_point_n = 0
                while (packet_point_n < _entries_max) and (point_n < point_count):
                    motor0_pos = motor0_pos_list[point_n]
                    motor0_vel = motor0_vel_list[point_n]
                    motor1_pos = motor1_pos_list[point_n]
                    motor1_vel = motor1_vel_list[point_n]

                    self._convert_and_set_setpoint(motor0_pos,motor0_vel,motor1_pos,motor1_vel,packet_point_n)

                    packet_point_n += 1
                    point_n += 1

                self.USBPacketOut.EntryCount = packet_point_n
                self.USBPacketOut.EntryLocation = point_n - packet_point_n
                self._lookup_table_fill()

            if vel_move:
                self._lookup_table_vel_move()
            else:
                self._lookup_table_pos_move()

        state = self._return_state()
        return state

    def _return_state(self):
        motor0_position = self.USBPacketIn.State.Motor[0].Position
        motor0_velocity = self.USBPacketIn.State.Motor[0].Frequency
        motor1_position = self.USBPacketIn.State.Motor[1].Position
        motor1_velocity = self.USBPacketIn.State.Motor[1].Frequency
        motor2_position = self.USBPacketIn.State.Motor[2].Position
        motor2_velocity = self.USBPacketIn.State.Motor[2].Frequency
        bAllJointsInPosition = self.USBPacketIn.AllMotorsInPosition
        bMoveInProgress = self.USBPacketIn.LookupTableMoveInProgress
        bHomeInProgress = self.USBPacketIn.HomeInProgress
        bJointsAtHome = self.USBPacketIn.MotorsHomed

        return motor0_position,motor0_velocity,motor1_position,motor1_velocity,motor2_position,motor2_velocity,bAllJointsInPosition,bMoveInProgress,bHomeInProgress,bJointsAtHome

    def _convert_and_set_setpoint(self,motor0_pos,motor0_vel,motor1_pos,motor1_vel,entry_n=0):
        motor0_pos,motor0_vel,motor1_pos,motor1_vel = self._check_and_convert_setpoint(motor0_pos,motor0_vel,motor1_pos,motor1_vel)

        self._set_position(0,motor0_pos,entry_n)
        self._set_frequency(0,motor0_vel,entry_n)
        self._set_position(1,motor1_pos,entry_n)
        self._set_frequency(1,motor1_vel,entry_n)

    def _check_and_convert_setpoint(self,motor0_pos,motor0_vel,motor1_pos,motor1_vel):
        # Set position values
        if (motor0_pos is None) or (motor1_pos is None):
            if motor0_vel < 0:
                motor0_pos = self.position_min
            else:
                motor0_pos = self.position_max

            if motor1_vel < 0:
                motor1_pos = self.position_min
            else:
                motor1_pos = self.position_max
        else:
            if motor0_pos < self.position_min:
                motor0_pos = self.position_min
            elif self.position_max < motor0_pos:
                motor0_pos = self.position_max

            if motor1_pos < self.position_min:
                motor1_pos = self.position_min
            elif self.position_max < motor1_pos:
                motor1_pos = self.position_max

        # Set velocity values
        motor0_vel = abs(motor0_vel)
        motor1_vel = abs(motor1_vel)

        if self.frequency_max < motor0_vel:
            motor1_vel = (motor1_vel/motor0_vel)*self.frequency_max
            motor0_vel = self.frequency_max

        if self.frequency_max < motor1_vel:
            motor0_vel = (motor0_vel/motor1_vel)*self.frequency_max
            motor1_vel = self.frequency_max

        return motor0_pos,motor0_vel,motor1_pos,motor1_vel

    def _set_frequency(self,axis,freq,entry_n=0):
        self.USBPacketOut.Entry[entry_n].Motor[axis].Frequency = int(freq)

    def _set_position(self,axis,pos,entry_n=0):
        self.USBPacketOut.Entry[entry_n].Motor[axis].Position = int(pos)

    def _send_usb_cmd(self,cmd,data_in_out_packet):
        with self.reentrant_lock:
            if data_in_out_packet:
                outdata = [cmd, self.USBPacketOut]
            else:
                outdata = [cmd]
            intypes = [ctypes.c_uint8, USBPacketIn_t]
            val_list = self.usb_cmd(outdata,intypes)
            cmd_id = val_list[0]
            self._check_cmd_id(cmd,cmd_id)
            self.USBPacketIn = val_list[1]

    def _get_motor_state(self):
        self._send_usb_cmd(self.USB_CMD_GET_STATE,False)
        # self._print_usb_packet_out()

    def _set_motor_state(self):
        self.USBPacketOut.MotorUpdate = ctypes.c_uint8(7)
        self._send_usb_cmd(self.USB_CMD_SET_STATE,True)
        # self._print_usb_packet_out()
        # self._print_usb_packet_in()

    def _lookup_table_fill(self):
        # rospy.loginfo("sending lookup_table_fill to usb device")
        self._send_usb_cmd(self.USB_CMD_LOOKUP_TABLE_FILL,True)

    def _lookup_table_pos_move(self):
        # rospy.loginfo("sending lookup_table_pos_move to usb device")
        self.USBPacketOut.MotorUpdate = ctypes.c_uint8(7)
        self._send_usb_cmd(self.USB_CMD_LOOKUP_TABLE_POS_MOVE,True)

    def _lookup_table_vel_move(self):
        self.bMoveInProgress = True
        # rospy.loginfo("sending lookup_table_vel_move to usb device")
        self.USBPacketOut.MotorUpdate = ctypes.c_uint8(7)
        self._send_usb_cmd(self.USB_CMD_LOOKUP_TABLE_VEL_MOVE,True)
        # self._print_usb_packet_out()

    # def _print_usb_packet_out(self):
    #     # _fields_ =[('MotorUpdate', ctypes.c_uint8),
    #     #            ('EntryCount', ctypes.c_uint8),
    #     #            ('EntryLocation', ctypes.c_uint8),
    #     #            ('Entry', LookupTableRow_t * _entries_max)]
    #     # self.USBPacketOut.Entry[entry_n].Motor[axis].Frequency = int(freq)
    #     rospy.loginfo('*'*20)
    #     rospy.loginfo('USB Packet Out')
    #     rospy.loginfo("MotorUpdate = %s" % (str(self.USBPacketOut.MotorUpdate)))
    #     rospy.loginfo("EntryCount = %s" % (str(self.USBPacketOut.EntryCount)))
    #     rospy.loginfo("EntryLocation = %s" % (str(self.USBPacketOut.EntryLocation)))
    #     rospy.loginfo("Entry 0 Motor 0 Freq = %s" % (str(self.USBPacketOut.Entry[0].Motor[0].Frequency)))
    #     rospy.loginfo("Entry 0 Motor 0 Pos = %s" % (str(self.USBPacketOut.Entry[0].Motor[0].Position)))
    #     rospy.loginfo("Entry 0 Motor 1 Freq = %s" % (str(self.USBPacketOut.Entry[0].Motor[0].Frequency)))
    #     rospy.loginfo("Entry 0 Motor 1 Pos = %s" % (str(self.USBPacketOut.Entry[0].Motor[0].Position)))
    #     rospy.loginfo('*'*20)

    # def _print_usb_packet_in(self):
    #     rospy.loginfo('*'*20)
    #     rospy.loginfo('USB Packet Out')
    #     rospy.loginfo("AllMotorsInPosition = %s" % (str(self.USBPacketIn.AllMotorsInPosition)))
    #     rospy.loginfo("LookupTableMoveComplete = %s" % (str(self.USBPacketIn.LookupTableMoveComplete)))
    #     rospy.loginfo("MotorsHomed = %s" % (str(self.USBPacketIn.MotorsHomed)))
    #     rospy.loginfo('Frequency X = %s' % (str(self.USBPacketIn.State.Motor[self.axis_x].Frequency)))
    #     rospy.loginfo('Position X = %s' % (str(self.USBPacketIn.State.Motor[self.axis_x].Position)))
    #     rospy.loginfo('Frequency Y = %s' % (str(self.USBPacketIn.State.Motor[self.axis_y].Frequency)))
    #     rospy.loginfo('Position Y = %s' % (str(self.USBPacketIn.State.Motor[self.axis_y].Position)))
    #     rospy.loginfo('Frequency Theta = %s' % (str(self.USBPacketIn.State.Motor[self.axis_theta].Frequency)))
    #     rospy.loginfo('Position Theta = %s' % (str(self.USBPacketIn.State.Motor[self.axis_theta].Position)))
    #     rospy.loginfo('*'*20)

    def _check_cmd_id(self,expected_id,received_id):
        """
        Compares expected and received command ids.

        Arguments:
            expected_id = expected command id
            received_is = received command id

        Return: None
        """
        if not expected_id.value == received_id.value:
            msg = "received incorrect command ID %d expected %d"%(received_id.value,expected_id.value)
            rospy.logwarn(msg)
            # raise IOError, msg
        return

#-------------------------------------------------------------------------------------
if __name__ == '__main__':

    # rospy.loginfo("Opening motor controller device ...")
    dev = MotorControllerDevice()
    dev.print_values()
    dev.close()
    # rospy.loginfo("Motor controller device closed.")
