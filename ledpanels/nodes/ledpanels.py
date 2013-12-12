#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('ledpanels')
import rospy
import serial
import numpy as N

from ledpanels.msg import MsgPanelsCommand
from ledpanels.srv import *
import subprocess


#######################################################################################################
# ledpanels node receives message commands on the topic 'ledpanels/command', and communicates with
# the panel hardware.
#
# For example:
# rosrun ledpanels ledpanels.py
#
# and then:
# rostopic pub -1 ledpanels/command ledpanels/MsgPanelsCommand all_on 0 0 0 0 0 0
# rostopic pub -1 ledpanels/command ledpanels/MsgPanelsCommand set_pattern_id 1 0 0 0 0 0
# (panel commands take 0 to 6 parameters, but the rostopic command needs to see all six, hence the "all_on 0 0 0 0 0 0", etc).
# 
class LEDPanels():
    def __init__(self):
        self.initialized = False
        
        rospy.init_node('ledpanels')
        rospy.sleep(1)
        
        self.subPanelsCommand = rospy.Subscriber('ledpanels/command', MsgPanelsCommand, self.PanelsCommand_callback)
        rospy.on_shutdown(self.OnShutdown_callback)
        
        self.commands = {
                         # 1 byte commands:
                         'start':              {'id': 0x20, 'args': [], 'help': 'start(), Start display.'},
                         'stop':               {'id': 0x30, 'args': [], 'help': 'stop(), Stop display.'},
                         'start_w_trig':       {'id': 0x25, 'args': [], 'help': 'start_w_trig(), Start display w/ trigger.'},
                         'stop_w_trig':        {'id': 0x35, 'args': [], 'help': 'stop_w_trig(), Stop display w/ trigger'},
                         'clear':              {'id': 0xF0, 'args': [], 'help': 'clear(), Clear the flash'},
                         'all_off':            {'id': 0x00, 'args': [], 'help': 'all_off(), set all pixels on all panels to OFF'},
                         'all_on':             {'id': 0xFF, 'args': [], 'help': 'all_on(), set all pixels on all panels to ON;'},
                         'g_level_0':          {'id': 0x90, 'args': [], 'help': 'g_level_0(), set all panels to grey level 0;'},
                         'g_level_1':          {'id': 0x91, 'args': [], 'help': 'g_level_1(), set all panels to grey level 1;'},
                         'g_level_2':          {'id': 0x92, 'args': [], 'help': 'g_level_2(), set all panels to grey level 2;'},
                         'g_level_3':          {'id': 0x93, 'args': [], 'help': 'g_level_3(), set all panels to grey level 3;'},
                         'g_level_4':          {'id': 0x94, 'args': [], 'help': 'g_level_4(), set all panels to grey level 4;'},
                         'g_level_5':          {'id': 0x95, 'args': [], 'help': 'g_level_5(), set all panels to grey level 5;'},
                         'g_level_6':          {'id': 0x96, 'args': [], 'help': 'g_level_6(), set all panels to grey level 6;'},
                         'g_level_7':          {'id': 0x97, 'args': [], 'help': 'g_level_7(), set all panels to grey level 7;'},
                         'g_level_8':          {'id': 0x98, 'args': [], 'help': 'g_level_8(), set all panels to grey level 8;'},
                         'g_level_9':          {'id': 0x99, 'args': [], 'help': 'g_level_9(), set all panels to grey level 9;'},
                         'g_level_10':         {'id': 0x9A, 'args': [], 'help': 'g_level_10(), set all panels to grey level 10;'},
                         'g_level_11':         {'id': 0x9B, 'args': [], 'help': 'g_level_11(), set all panels to grey level 11;'},
                         'g_level_12':         {'id': 0x9C, 'args': [], 'help': 'g_level_12(), set all panels to grey level 12;'},
                         'g_level_13':         {'id': 0x9D, 'args': [], 'help': 'g_level_13(), set all panels to grey level 13;'},
                         'g_level_14':         {'id': 0x9E, 'args': [], 'help': 'g_level_14(), set all panels to grey level 14;'},
                         'g_level_15':         {'id': 0x9F, 'args': [], 'help': 'g_level_15(), set all panels to grey level 15;'},
                         'led_tog':            {'id': 0x50, 'args': [], 'help': 'led_tog(), toggles controller LED'},
                         'ctr_reset':          {'id': 0x60, 'args': [], 'help': 'ctr_reset(), resets the controller'},
                         'bench_pattern':      {'id': 0x70, 'args': [], 'help': 'bench_pattern(), run a benchmark on current pattern'},
                         'laser_on':           {'id': 0x10, 'args': [], 'help': 'laser_on(), enable laser trigger'},
                         'laser_off':          {'id': 0x11, 'args': [], 'help': 'laser_off(), enable laser trigger'},
                         'ident_compress_on':  {'id': 0x12, 'args': [], 'help': 'ident_compress_on(), enable compression for identical panel patches'},
                         'ident_compress_off': {'id': 0x13, 'args': [], 'help': 'ident_compress_off(), disable compression for identical panel patches'},
                         'sync_sd_info':       {'id': 0x14, 'args': [], 'help': 'sync_sd_info()'},
                         'get_version':        {'id': 0x15, 'args': [], 'help': 'get_version()'},     # Run as a service, not on a message topic.
                         'show_bus_number':    {'id': 0x16, 'args': [], 'help': 'show_bus_number()'},
                         'quiet_mode_on':      {'id': 0x17, 'args': [], 'help': 'quiet_mode_on(), In this mode, there is no feedback information sent from controller'},
                         'quiet_mode_off':     {'id': 0x18, 'args': [], 'help': 'quiet_mode_off(), In this mode, feedback information from controller will be shown on the GUI'},
                         'update_gui_info':    {'id': 0x19, 'args': [], 'help': 'update_gui_info(), Update gain, offset, and position information.'},
                         'controller_mode':    {'id': 0x21, 'args': [], 'help': 'controller_mode()'},
                         'pc_dumping_mode':    {'id': 0x22, 'args': [], 'help': 'pc_dumping_mode()'},
                         'enable_extern_trig': {'id': 0x23, 'args': [], 'help': 'reset_funccnt_x(), reset function x count'},
                         'disable_extern_trig':{'id': 0x24, 'args': [], 'help': 'reset_funccnt_y(), reset function y count'},
                         'read_and_set_max_voltage':{'id': 0x26, 'args': [], 'help': 'read_and_set_max_voltage()'},
                         

                         # *************************************************************************
                         # 2 byte commands:
                         'reset':              {'id': 0x01, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': True}], 
                                                'help': 'reset(panel_addr), Board reset'},
                         'display':            {'id': 0x02, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': True}], 
                                                'help': 'display(panel_addr), Display id'},
                         'set_pattern_id':     {'id': 0x03, 
                                                'args': [{'nbytes': 1, 'min': 1, 'max': 99, 'unsigned': True}], 
                                                'help': 'set_pattern_id(pattern_id), panel ID:  0x03, Panel_ID'},
                         'adc_test':           {'id': 0x04, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 7, 'unsigned': True}], 
                                                'help': 'adc_test(channel_addr), test ADC on controller board for certain channel, channel addr (0 - 7)'},
                         'dio_test':           {'id': 0x05, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 7, 'unsigned': True}], 
                                                'help': 'dio_test(channel_addr), test DIO on controller board for certain channel, channel addr (0 - 7)'},
                         'set_trigger_rate':   {'id': 0x06, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': True}], 
                                                'help': 'set_trigger_rate(hz), set the trigger rate on the controller; hz 0-255 (multiplied by 2 on the controller).'},
                         'flash_panel':        {'id': 0x07, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': True}], 
                                                'help': 'flash_panel(?)'},
                         'eeprom_panel':       {'id': 0x08, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': True}], 
                                                'help': 'eeprom_panel(?)'},
                         'set_config_id':      {'id': 0x09, 
                                                'args': [{'nbytes': 1, 'min': 1, 'max': 99, 'unsigned': True}], 
                                                'help': 'set_config_id(?)'},
                         'get_adc_value':      {'id': 0x10,     # Run as a service, not on a message topic.
                                                'args': [{'nbytes': 1, 'min': 1, 'max': 6, 'unsigned': True}], 
                                                'help': 'get_adc_value(channel_addr)'},
                         
                         # *************************************************************************
                         # 3 byte commands:
                         'set_mode':           {'id': 0x10, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 6, 'unsigned': True}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 6, 'unsigned': True}], 
                                                'help': 'set_mode(mode_x, mode_y), 0: xrate=funcx, 1:xrate=ch0, 2:xrate=ch0+funcx, 3:x=ch2, 4:x=x0+funcx, 5:debugx, 0x61:custom pos=f(adc,func), 0x62:custom vel=f(adc,func); similar for y but at adc ch1/ch3'},
                         'address':            {'id': 0xFF, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': True}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': True}], 
                                                'help': 'update address: 0xFF; current address, new address'},
                         'set_posfunc_id':     {'id': 0x15, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': True}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': True}], 
                                                'help': 'set_posfunc_id(channel, functionid), Set position function.  functionid==0 means use default function.'},
                         'set_velfunc_id':     {'id': 0x20, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': True}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': True}], 
                                                'help': 'set_velfunc_id(channel, functionid), Set velocity function'},
                         'set_funcx_freq':     {'id': 0x25, 
                                                'args': [{'nbytes': 2, 'min': 0, 'max': 500, 'unsigned': True}], 
                                                'help': 'set_funcx_freq(hz), function X update rate'},
                         'set_funcy_freq':     {'id': 0x30, 
                                                'args': [{'nbytes': 2, 'min': 0, 'max': 500, 'unsigned': True}], 
                                                'help': 'set_funcy_freq(hz), function Y update rate'},
                         'set_max_voltage':    {'id': 0x35, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 10, 'unsigned': True}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 10, 'unsigned': True}], 
                                                'help': 'set_max_voltage(xmax,ymax), Set max voltage for X and Y on range (0,10).'},
                         'set_voltage_range_adc':{'id': 0x62, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 7, 'unsigned': True}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 3, 'unsigned': True}], 
                                                'help': 'set_voltage_range_adc(channel, range), Set input voltage range on an ADC.  range = 0:[-10,+10], 1:[-5,+5], 2:[-2.5,+2.5], 3:[0,+10]'},


                         # *************************************************************************
                         # 4 byte commands:     
                         'set_ao':             {'id': 0x10, 
                                                'args': [{'nbytes': 1, 'min': 1, 'max': 4, 'unsigned': True}, 
                                                         {'nbytes': 2, 'min': -32767, 'max': 32767, 'unsigned': True}], # BUG: should this be signed?
                                                'help': 'set_AO(chan, val), val ranges on (-32767,32767) aka (-10v,+10v)'},

                         
                         # *************************************************************************
                         # 5 byte commands:
                         'set_position':       {'id': 0x70, 
                                                'args': [{'nbytes': 2, 'min': 0, 'max': 2047, 'unsigned': True}, 
                                                         {'nbytes': 2, 'min': 0, 'max': 2047, 'unsigned': True}], 
                                                'help': 'set_position(index_x, index_y), pattern position is 0-based.'},
                         'send_gain_bias':     {'id': 0x71, 
                                                'args': [{'nbytes': 1, 'min': -128, 'max': 127, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': -128, 'max': 127, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': -128, 'max': 127, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': -128, 'max': 127, 'unsigned': False}], 
                                                'help': 'set_gain_bias(gain_x, bias_x, gain_y, bias_y), with 8 bit arguments.'},


                         # *************************************************************************
                         # 7 byte commands:
                         'set_mode_pos_custom_x':{'id': 0x63, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}], 
                                                'help': 'set_mode_pos_custom_x(a0, a1, a2, a3, a4, a5), Set coefficients on input sources for x positions.  xpos = a0*adc0 + a1*adc1 + a2*adc2 + a3*adc3 + a4*funcx + a5*funcy; a_ valid on [-128,+127], and 10 corresponds to 1.0.'},
                         'set_mode_pos_custom_y':{'id': 0x64, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}], 
                                                'help': 'set_mode_pos_custom_y(a0, a1, a2, a3, a4, a5), Set coefficients on input sources for x positions.  ypos = a0*adc0 + a1*adc1 + a2*adc2 + a3*adc3 + a4*funcx + a5*funcy; a_ valid on [-128,+127], and 10 corresponds to 1.0.'},
                         'set_mode_vel_custom_x': {'id': 0x65, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}], 
                                                'help': 'set_mode_vel_custom_x(a0, a1, a2, a3, a4, a5), Set coefficients on input sources for x velocity.  xrate = a0*adc0 + a1*adc1 + a2*adc2 + a3*adc3 + a4*funcx + a5*funcy; a_ valid on [-128,+127], and 10 corresponds to 1.0.'},
                         'set_mode_vel_custom_y': {'id': 0x66, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': False}], 
                                                'help': 'set_mode_vel_custom_y(a0, a1, a2, a3, a4, a5), Set coefficients on input sources for y velocity.  yrate = a0*adc0 + a1*adc1 + a2*adc2 + a3*adc3 + a4*funcx + a5*funcy; a_ valid on [-128,+127], and 10 corresponds to 1.0.'},

                         
                         # *************************************************************************
                         # 9 byte commands:
                         'send_gain_bias_16':     {'id': 0x01, 
                                                'args': [{'nbytes': 2, 'min': -32768, 'max': 32767, 'unsigned': False}, 
                                                         {'nbytes': 2, 'min': -32768, 'max': 32767, 'unsigned': False}, 
                                                         {'nbytes': 2, 'min': -32768, 'max': 32767, 'unsigned': False}, 
                                                         {'nbytes': 2, 'min': -32768, 'max': 32767, 'unsigned': False}], 
                                                'help': 'set_gain_bias_16(gain_x, bias_x, gain_y, bias_y), with 16 bit arguments.'},


                         # *************************************************************************
                         # 129 byte commands:
                         'send_laser_pattern': {'id': 0x3E, 
                                                'args': [{'nbytes': 32, 'min': 0, 'max': 1.158e77, 'unsigned': True}, 
                                                         {'nbytes': 32, 'min': 0, 'max': 1.158e77, 'unsigned': True}, 
                                                         {'nbytes': 32, 'min': 0, 'max': 1.158e77, 'unsigned': True}, 
                                                         {'nbytes': 32, 'min': 0, 'max': 1.158e77, 'unsigned': True}], 
                                                'help': 'send_laser_pattern(), Four 256bit arguments to contain 1024 binary values.  Not yet supported.'},
                         }


        # A few commands need to run as services, since they return data back to the caller.
        self.services = {}
        self.services['get_version'] = self.srvGetVersion = rospy.Service('get_version', SrvGetVersion, self.GetVersion_callback)        
        self.services['get_adc_value'] = rospy.Service('get_adc_value', SrvGetADCValue, self.GetADCValue_callback)        

        self.serialport = self.DiscoverSerialPort() 
        rospy.logwarn ('ledpanels using %s' % self.serialport)
        if (self.serialport is not None):
            self.serial = serial.Serial(self.serialport, baudrate=921600, rtscts=False, dsrdtr=False, timeout=1) # 8N1
            self.initialized = True
        else:
            rospy.logerr('ledpanels serial port was not specified as a ROS parameter, nor was it found automatically.')
            

    # Figure out which serial port the panels controller is attached to.
    # If a ROS param is set, then use it.  Otherwise try to find it automatically.
    # The ideitifying characteristic is the string "FTDI USB Serial Device" in the dmesg output.
    def DiscoverSerialPort(self):
        serialport = rospy.get_param('ledpanels/serialport', 'unspecified')

        if (serialport=='unspecified'):
            p1 = subprocess.Popen(["dmesg"], stdout=subprocess.PIPE)
            p2 = subprocess.Popen(["grep", "FTDI USB Serial Device converter now attached"], stdin=p1.stdout, stdout=subprocess.PIPE)
            output = p2.communicate()[0]    
            output = output.replace('\n', ' ')
            
            # Go through all the ttyXXXXX tokens, and take the last one.
            serialport = None  
            for token in output.split(' '):
                if 'tty' in token:
                    serialport = '/dev/' + token
                
            
        return serialport
              
        
    def MakeSurePortIsOpen(self):
        while not self.serial.isOpen():
            try:
                rospy.logwarn ('ledpanels: Trying to open serial port %s ...' % self.serialport)
                self.serial.open()
            except serial.SerialException:
                rospy.sleep(1)
            else:
                rospy.logwarn ('ledpanels: Opened serial port %s' % self.serialport)
    
        
    # Sends the get_version command to the LED controller, and returns the response.
    def GetVersion_callback (self, req):
        version = None
        if self.initialized:
            serialbytes_list = self.SerialBytelistFromCommand('get_version', [])

            self.MakeSurePortIsOpen()
                        
            self.serial.write(''.join(serialbytes_list))
            version = self.serial.readline()
            
        return SrvGetVersionResponse(version=version)
            
            

    # Sends the get_adc_value command to the LED controller, and returns the response.
    def GetADCValue_callback (self, req):
        value = None
        if self.initialized:
            serialbytes_list = self.SerialBytelistFromCommand('get_adc_value', [req.channel])
            self.MakeSurePortIsOpen()
            self.serial.write(''.join(serialbytes_list))
            value = self.serial.readline()
            
        return SrvGetADCValueResponse(value=value)
            
            

    def NbytesFromCommand (self, command):
        if command in self.commands:
            nBytes = 1  # For the command id.
            for arg in self.commands[command]['args']:
                nBytes += arg['nbytes']
        else:
            nBytes = 0
        
        return nBytes
    
    
    def SerialBytelistFromCommand (self, command, args=[]):
        serialbytes_list = []
        
        if command in self.commands:
            nBytesCmd = self.NbytesFromCommand(command)
            serialbytes_list.append(chr(nBytesCmd))
            
            id = self.commands[command]['id']
            serialbytes_list.append(chr(id))

            args_dict_list = self.commands[command]['args']
            for iArg in range(len(args_dict_list)):
                arg = args[iArg]
                nBytesArg = args_dict_list[iArg]['nbytes']
                
                if (nBytesArg==1):
                    if (args_dict_list[iArg]['unsigned']==True):
                        serialbytes_list.append(chr(arg))
                         
                    if (args_dict_list[iArg]['unsigned']==False):
                        serialbytes_list.append(self.Signed_byte_to_chr(arg))
                     
                if (nBytesArg>1):
                    if (args_dict_list[iArg]['unsigned']==True):
                        serialbytes_list.extend(self.Dec2chr(arg,nBytesArg))
                         
                    if (args_dict_list[iArg]['unsigned']==False):
                        # Note: the following is a hack special case for set_ao.  Negative arg2 changes the command id, and sends abs(arg2).
                        if (command == 'set_ao'):
                            if arg>=0:
                                serialbytes_list.extend(self.Dec2chr(arg,nBytesArg))
                            else:
                                id = 0x11
                                serialbytes_list.extend(self.Dec2chr(N.abs(arg),nBytesArg))
                        else:        
                            rospy.logwarn('ledpanels: command argument type (signed int) not yet implemented.')
        else:
            rospy.logwarn('Unknown ledpanels command: %s' % command)
                        
        #rospy.logwarn ('ledpanels: sending %s' % serialbytes_list)
        return serialbytes_list
                    
        
    # Ported from Reiser's panel code.
    def Signed_byte_to_chr(self, byte):
        return chr((256+byte) % 256)
    

    # Ported from Reiser's panel code.
    def Dec2chr(self, num, nChars):
        # This functions makes an array of char values (0-255) from a decimal number.
        # This is listed in LSB first order.
        # Untested for negative numbers, probably wrong!
        # e.g. To decode a 3 char array:  ans = charArray(3)*2^16 + charArray(2)*2^8 + charArray(1)*2^0

        char_list = [0] * nChars
        if (num > 2**(8*nChars)):
            rospy.logerr('Dec2chr() overflow: Not enough characters for a number of this size.')
        if (num < 0 ):
            rospy.logerr('Dec2chr() out of range: This function does not handle negative numbers.' );
        
        num_rem = num
        for j in range(nChars,0,-1):
            tmp = N.floor((num_rem)/(2**(8*(j-1))))
            num_rem = num_rem - tmp*(2**(8*(j-1)))
            char_list[j-1] = chr(int(tmp))
        
        return char_list
    


    def OnShutdown_callback(self):
        if self.initialized:
            self.initialized = False
            if self.serial.isOpen():
                self.serial.close()


    def PanelsCommand_callback(self, panelcommand):
        if self.initialized:
            #rospy.logwarn('ledpanels: %s(%d,%d,%d,%d,%d,%d)' % (panelcommand.command, panelcommand.arg1, panelcommand.arg2, panelcommand.arg3, panelcommand.arg4, panelcommand.arg5, panelcommand.arg6))
            command = panelcommand.command.lower()
            serialbytes_list = self.SerialBytelistFromCommand(command, [panelcommand.arg1, panelcommand.arg2, panelcommand.arg3, panelcommand.arg4, panelcommand.arg5, panelcommand.arg6])
            self.MakeSurePortIsOpen()
                        
            try:
                self.serial.write(''.join(serialbytes_list))
            except serial.SerialException, e:
                rospy.logwarn('ledpanels: Serial exception, trying to reopen: %s' % e)
                if self.serial.isOpen():
                    self.serial.close()
                self.MakeSurePortIsOpen()
                try:
                    self.serial.write(''.join(serialbytes_list))
                except serial.SerialException, e:
                    rospy.logwarn ('ledpanels: Write FAILED to serial port %s: %s' % (self.serialport, e))
                else:
                    rospy.logwarn ('ledpanels: Write succeeded to serial port %s.' % self.serialport)

    def Main(self):
        panelcommand = MsgPanelsCommand(command='all_off')
        self.PanelsCommand_callback(panelcommand)
        rospy.spin()

#        # Shutdown all the services we offered.
#        for key in self.services:
#            self.services[key].shutdown()
            
            


if __name__ == '__main__':
    panels = LEDPanels()
    panels.Main()
    

