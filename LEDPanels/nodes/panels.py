#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('LEDPanels')
import rospy
import serial
import numpy as N

from LEDPanels.msg import MsgPanelsCommand
from LEDPanels.srv import SrvGetVersion, SrvGetADCValue


#######################################################################################################
class LEDPanels():
    def __init__(self):
        self.initialized = False
        
        rospy.init_node('LEDPanels')
        
        self.subPanelsCommand = rospy.Subscriber('LEDPanels/command', MsgPanelsCommand, self.PanelsCommand_callback)
        rospy.on_shutdown(self.OnShutdown_callback)
        self.serial = serial.Serial('/dev/ttyUSB0', baudrate=921600) # 8N1

        self.commands = {
                         'start':              {'id': 0x20, 'args': [], 'help': 'start()'},
                         'stop':               {'id': 0x30, 'args': [], 'help': 'stop()'},
                         'start_w_trig':       {'id': 0x25, 'args': [], 'help': 'start_w_trig()'},
                         'stop_w_trig':        {'id': 0x35, 'args': [], 'help': 'stop_w_trig()'},
                         'clear':              {'id': 0xF0, 'args': [], 'help': 'clear(), clear the flash'},
                         'all_off':            {'id': 0x00, 'args': [], 'help': 'all_off(), set all panels to OFF'},
                         'all_on':             {'id': 0xFF, 'args': [], 'help': 'all_on(), set all panels to ON;'},
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
                        #'get_version':        {'id': 0x15, 'args': [], 'help': 'get_version()'},     # Run as a service, not on a message topic.
                         'show_bus_number':    {'id': 0x16, 'args': [], 'help': 'show_bus_number()'},
                         'quiet_mode_on':      {'id': 0x17, 'args': [], 'help': 'quiet_mode_on(), In this mode, there is no feedback information sent from controller'},
                         'quiet_mode_off':     {'id': 0x18, 'args': [], 'help': 'quiet_mode_off(), In this mode, feedback information from controller will be shown on the GUI'},
                         'controller_mode':    {'id': 0x21, 'args': [], 'help': 'controller_mode()'},
                         'pc_dumping_mode':    {'id': 0x22, 'args': [], 'help': 'pc_dumping_mode()'},
                         'reset_funccnt_x':    {'id': 0x23, 'args': [], 'help': 'reset_funccnt_x(), reset function x count'},
                         'reset_funccnt_y':    {'id': 0x24, 'args': [], 'help': 'reset_funccnt_y(), reset function y count'},
                         
                         # two byte commands:
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
                        #'get_adc_value':      {'id': 0x10,     # Run as a service, not on a message topic.
                        #                       'args': [{'nbytes': 1, 'min': 1, 'max': 4, 'unsigned': True}], 
                        #                       'help': 'get_adc_value(channel_addr)'},
                         
                         # three byte commands:
                         'set_mode':           {'id': 0x10, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 5, 'unsigned': True}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 5, 'unsigned': True}], 
                                                'help': 'set_mode(mode_x, mode_y), 0=funcx, 1=ch1-ch2, 2=CL+funcx, 3=ch5_sets_x, 4=funcx_sets_ind, 5=debugx; similar for y'},
                         'address':            {'id': 0xFF, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': True}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': True}], 
                                                'help': 'update address: 0xFF; current address, new address'},
                         'set_posfunc_id':     {'id': 0x15, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': True}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': True}], 
                                                'help': 'set position'},
                         'set_velfunc_id':     {'id': 0x20, 
                                                'args': [{'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': True}, 
                                                         {'nbytes': 1, 'min': 0, 'max': 0xFF, 'unsigned': True}], 
                                                'help': 'set velocity function'},
                         'set_funcx_freq':     {'id': 0x25, 
                                                'args': [{'nbytes': 2, 'min': 0, 'max': 500, 'unsigned': True}], 
                                                'help': 'set_funcx_freq(hz), function X update rate'},
                         'set_funcy_freq':     {'id': 0x30, 
                                                'args': [{'nbytes': 2, 'min': 0, 'max': 500, 'unsigned': True}], 
                                                'help': 'set_funcy_freq(hz), function Y update rate'},
                         
                         # four byte commands:     
                         'set_ao':             {'id': 0x10, 
                                                'args': [{'nbytes': 1, 'min': 1, 'max': 4, 'unsigned': True}, 
                                                         {'nbytes': 2, 'min': 0, 'max': 2047, 'unsigned': True}], 
                                                'help': 'set_AO(chan, val)'},
                         
                         # five byte commands:
                         'set_position':       {'id': 0x70, 
                                                'args': [{'nbytes': 2, 'min': 0, 'max': 2047, 'unsigned': True}, 
                                                         {'nbytes': 2, 'min': 0, 'max': 2047, 'unsigned': True}], 
                                                'help': 'set_position(index_x, index_y), pattern position is 0-based.'},
                         'send_gain_bias':     {'id': 0x71, 
                                                'args': [{'nbytes': 1, 'min': -128, 'max': 127, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': -128, 'max': 127, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': -128, 'max': 127, 'unsigned': False}, 
                                                         {'nbytes': 1, 'min': -128, 'max': 127, 'unsigned': False}], 
                                                'help': 'set_gain_bias(gain_x, bias_x, gain_y, bias_y)'},
                         }


        # A few commands need to run as services, since they return data back to the caller.
        self.srvGetVersion = rospy.Service('GetVersion', SrvGetVersion, self.GetVersion_callback)        
        self.srvGetADCValue = rospy.Service('GetADCValue', SrvGetADCValue, self.GetADCValue_callback)        


        self.initialized = True


    # Sends the get_version command to the LED controller, and returns the response.
    def GetVersion_callback (self, req):
        version = None
        if self.initialized:
            serialbytes_list = self.SerialBytelistFromCommand('get_version', [])
            self.serial.write(''.join(serialbytes_list))
            version = self.serial.readline()
            
        return SrvGetVersionResponse(version=version)
            
            

    # Sends the get_adc_value command to the LED controller, and returns the response.
    def GetADCValue_callback (self, req):
        value = None
        if self.initialized:
            serialbytes_list = self.SerialBytelistFromCommand('get_adc_value', [req.channel])
            self.serial.write(''.join(serialbytes_list))
            value = self.serial.readline()
            
        return SrvGetADCValueResponse(value=value)
            
            

    def nbytesFromCommand (self, command):
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
            nBytesCmd = self.nbytesFromCommand(command)
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
                        serialbytes_list.append(self.signed_byte_to_chr(arg))
                     
                if (nBytesArg>1):
                    if (args_dict_list[iArg]['unsigned']==True):
                        serialbytes_list.extend(self.dec2chr(arg,nBytesArg))
                         
                    if (args_dict_list[iArg]['unsigned']==False):
                        rospy.logwarn('LEDPanels not yet implemented.')
        else:
            rospy.logerror('Unknown LEDPanels command: %s' % command)
                        
        #rospy.logwarn ('sending %s' % serialbytes_list)
        return serialbytes_list
                    
        
    # Ported from Reiser's panel code.
    def signed_byte_to_chr(self, byte):
        return chr((256+byte) % 256)
    

    # Ported from Reiser's panel code.
    def dec2chr(self, num, nChars):
        # This functions makes an array of char values (0-255) from a decimal number.
        # This is listed in LSB first order.
        # Untested for negative numbers, probably wrong!
        # e.g. To decode a 3 char array:  ans = charArray(3)*2^16 + charArray(2)*2^8 + charArray(1)*2^0

        char_list = [0] * nChars
        if (num > 2**(8*nChars)):
            rospy.logerror('dec2chr() overflow: Not enough characters for a number of this size.')
        if (num < 0 ):
            rospy.logerror('dec2chr() out of range: This function does not handle negative numbers.' );
        
        num_rem = num
        for j in range(nChars,0,-1):
            tmp = N.floor((num_rem)/(2**(8*(j-1))))
            num_rem = num_rem - tmp*(2**(8*(j-1)))
            char_list[j-1] = chr(int(tmp))
        
        return char_list
    


    def OnShutdown_callback(self):
        if self.initialized:
            self.serial.close()


    def PanelsCommand_callback(self, panelcommand):
        if self.initialized:
            command = panelcommand.command.lower()
            serialbytes_list = self.SerialBytelistFromCommand(command, [panelcommand.arg1, panelcommand.arg2, panelcommand.arg3, panelcommand.arg4])
            self.serial.write(''.join(serialbytes_list))
        


    def Main(self):
        rospy.spin()
            


if __name__ == '__main__':
    panels = LEDPanels()
    panels.Main()
    

