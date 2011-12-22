"""
-----------------------------------------------------------------------
USBDevice
Copyright (C) William Dickson, 2008.

wbd@caltech.edu
www.willdickson.com

Released under the LGPL Licence, Version 3

This file is part of simple_step.

simple_step is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

simple_step is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with simple_step.  If not, see
<http://www.gnu.org/licenses/>.

------------------------------------------------------------------------

Purpose: Provides and API for the at90usb*.

Author: William Dickson

------------------------------------------------------------------------
"""
import roslib; roslib.load_manifest('stage')
import rospy

from pythonmodules import pylibusb as usb
import ctypes
import sys
import time
import math
import struct

DEBUG = False

# USB Command IDs
USB_CMD_AVR_RESET = ctypes.c_uint8(200)
USB_CMD_AVR_DFU_MODE = ctypes.c_uint8(201)

# Python version check
BYREF_EXISTS_VNUM = 2.6

class USB_Device:

    """
    Example USB interface to the at90usb board.
    """

    def __init__(self,
                 usb_vendor_id,
                 usb_product_id,
                 usb_bulkout_ep_address,
                 usb_bulkin_ep_address,
                 usb_buffer_out_size,
                 usb_buffer_in_size,
                 usb_serial_number=None):

        self.bulkout_ep_address = usb_bulkout_ep_address
        self.bulkin_ep_address = usb_bulkin_ep_address
        self.buffer_out_size = usb_buffer_out_size
        self.buffer_in_size = usb_buffer_in_size

        """
        Open and initialize usb device.

        Arguments: None

        Return: None.
        """
        usb.init()

        # Get usb busses
        if not usb.get_busses():
            usb.find_busses()
            usb.find_devices()
        busses = usb.get_busses()

        # Find device by IDs
        found = False
        dev_list = []
        for bus in busses:
            for dev in bus.devices:
                if (dev.descriptor.idVendor == usb_vendor_id and
                    dev.descriptor.idProduct == usb_product_id):
                    dev_list.append(dev)
                    found = True
        if not found:
            raise RuntimeError("Cannot find device.")

        if usb_serial_number == None:
            # No serial number specified - take first device
            dev = dev_list[0]
            self.libusb_handle = usb.open(dev)
            self.dev = dev
        else:
            # Try and find device with specified serial number
            found = False
            for dev in dev_list:
                self.dev = dev
                self.libusb_handle = usb.open(dev)
                sn = self.get_serial_number()
                if sn == usb_serial_number:
                    found = True
                    break
                else:
                    ret = usb.close(self.libusb_handle)
            if not found:
                raise RuntimeError("Cannot find device w/ serial number %s."%(usb_serial_number,))

        self.interface_nr = 0
        if hasattr(usb,'get_driver_np'):
            # non-portable libusb function available
            name = usb.get_driver_np(self.libusb_handle,self.interface_nr)
            if name != '':
                debug_print("attached to kernel driver '%s', detaching."%name )
                usb.detach_kernel_driver_np(self.libusb_handle,self.interface_nr)

        if dev.descriptor.bNumConfigurations > 1:
            debug_print("WARNING: more than one configuration, choosing first")

        usb.set_configuration(self.libusb_handle, self.dev.config[0].bConfigurationValue)
        usb.claim_interface(self.libusb_handle, self.interface_nr)

        self.output_buffer = ctypes.create_string_buffer(usb_buffer_out_size)
        self.input_buffer = ctypes.create_string_buffer(usb_buffer_in_size)
        for i in range(usb_buffer_in_size):
            self.input_buffer[i] = chr(0x00)
        for i in range(usb_buffer_out_size):
            self.output_buffer[i] = chr(0x00)

        # Clear any possible halt on the endpoints
        ret = usb.clear_halt(self.libusb_handle,usb_bulkout_ep_address)
        ret = usb.clear_halt(self.libusb_handle,usb_bulkin_ep_address)

        # Buffer position marker for reading from and writing to buffer
        self.output_buffer_pos = 0
        self.input_buffer_pos = 0

        # Set bref function based on python version
        if get_python_vnum() >= BYREF_EXISTS_VNUM:
            self.byref = ctypes.byref
        else:
            self.byref = byref_via_pointer

    def close(self):
        """
        Close usb device.

        Arguments: None

        Return: None
        """
        ret = usb.release_interface(self.libusb_handle,self.interface_nr)
        ret = usb.close(self.libusb_handle)
        return

    # -------------------------------------------------------------------------
    # Methods for low level USB communication

    def __send_and_receive(self,in_timeout=200,out_timeout=9999):
        """
        Send bulkout and and receive bulkin as a response.

        Arguments: None

        Keywords:
            in_timeout  = bulkin timeout in ms
            out_timeout = bulkin timeout in ms

        Return: the data returned by the usb device.
        """
        done = False
        while not done:
            val = usb.bulk_write(
                    self.libusb_handle,
                    self.bulkout_ep_address,
                    self.output_buffer,
                    out_timeout
                    )

            if val < 0 :
                raise IOError, "error sending usb output"

            try:
                numbytes = usb.bulk_read(
                        self.libusb_handle,
                        self.bulkin_ep_address,
                        self.input_buffer,
                        in_timeout
                        )
                debug_print('usb SR bytes read: %d'%(numbytes,), comma=False)
                debug_print('output_buffer = ',comma=True)
                debug_print(str([ord(b) for b in self.output_buffer]))
                debug_print('input_buffer = ',comma=True)
                debug_print(str([ord(b) for b in self.input_buffer]))
                done = True
            except usb.USBNoDataAvailableError:
                debug_print('usb SR: fail', comma=False)
                sys.stdout.flush()

        return

    def __val_to_buffer(self,val):
        buf_ptr = self.byref(self.output_buffer,self.output_buffer_pos)
        val_ptr = ctypes.pointer(val)
        sz = ctypes.sizeof(val)
        if self.output_buffer_pos + sz >  self.buffer_out_size:
            raise ValueError, 'output_buffer_pos + sz greater than USB_BUFFER_OUT_SIZE'
        ctypes.memmove(buf_ptr,val_ptr,sz)
        self.output_buffer_pos += sz
        return

    def __val_from_buffer(self,val):
        buf_ptr = self.byref(self.input_buffer,self.input_buffer_pos)
        val_ptr = ctypes.pointer(val)
        sz = ctypes.sizeof(val)
        if self.input_buffer_pos + sz > self.buffer_in_size:
            raise ValueError, 'input_buffer_pos + sz greater than USB_BUFFER_IN_SIZE'
        ctypes.memmove(val_ptr,buf_ptr,sz)
        self.input_buffer_pos += sz

    def __write_to_buffer(self,outdata):
        """
        Write data list/array to output buffer.
        """

        # Check size of data array
        N = 0
        for d in outdata:
            N += ctypes.sizeof(d)

        if N > self.buffer_out_size:
            raise ValueError, 'data array larger than max length'

        # Set output buffer data to all zeros
        for i in range(self.buffer_out_size):
            self.output_buffer[i] = chr(0x00)

        self.output_buffer_pos = 0

        # Add data array to output buffer
        for d in outdata:
            self.__val_to_buffer(d)

    def __read_from_buffer(self,input_types):
        self.input_buffer_pos = 0
        val_list = []
        for ctypes_type in input_types:
            val = ctypes_type()
            self.__val_from_buffer(val)
            val_list.append(val)
        return val_list

    def usb_cmd(self,outdata,intypes):
        """
        Generic usb command.
        """
        self.__write_to_buffer(outdata)
        self.__send_and_receive()
        val_list = self.__read_from_buffer(intypes)
        return val_list

    def get_serial_number(self):
        """
        Get serial number of device.

        Arguments: None

        Return: serial number of device - a string
        """
        return  usb.get_string_simple(self.libusb_handle, self.dev.descriptor.iSerialNumber)

    def get_manufacturer(self):
        """
        Get manufacturer of device

        Arguments: None

        Return: manufacturer string
        """
        return usb.get_string_simple(self.libusb_handle, self.dev.descriptor.iManufacturer)

    def get_product(self):
        """
        Get decive product string

        Arguments: None

        Return: product string
        """
        return usb.get_string_simple(self.libusb_handle, self.dev.descriptor.iProduct)

    def get_vendor_id(self):
        """
        Get device vendor ID.
        """
        return self.dev.descriptor.idVendor

    def get_product_id(self):
        """
        Get device product ID.
        """
        return self.dev.descriptor.idProduct

    def enter_dfu_mode(self):
        """
        Places the at90usb device in programming mode for upgrading the
        firmware. Note, after entering dfu mode no further communications
        with the device will be possible.

        Arguments: None

        Return: None
        """
        self.output_buffer[0] = chr(USB_CMD_AVR_DFU_MODE%0x100)
        val = self.__send_output()
        return

    def reset_device(self):
        """
        Resets the at90usb device. Note, currently this function has
        some problems. First, after resetting the device no further
        usb communications with the device are possible. Second, after
        reset the device occasionally fails to enumerate correctly and
        the only way I have found which fixes this is to reset the linux
        usb system. It is probably best not to use this function.

        Arguments: None

        Return: None
        """
        ###############################
        # DEBUG - has issues, see above
        ###############################

        self.output_buffer[0] = chr(USB_CMD_AVR_RESET%0x100)
        val = self.__send_output()
        self.close()
        return

    def print_values(self):
        """
        Prints the current device values.

        Arguments: None

        Return None.
        """
        print
        print ' device information'
        print ' '+ '-'*35
        print '   manufacturer:', self.get_manufacturer()
        print '   product:', self.get_product()
        print '   vendor ID:', hex(self.get_vendor_id())
        print '   product ID:', hex(self.get_product_id())
        print '   serial number:',self.get_serial_number()


def debug_print(msg, comma=False):
    if DEBUG==True:
        rospy.logwarn(msg)
        # if comma==True:
        #     # print >> sys.stderr, msg,
        # else:
        # #     print >> sys.stderr, msg
        # # sys.stdout.flush()

def pointer_incr(ptr,offset):
    """
    Increments pointer by given offset. For use with python
    version < 2.6 where the byref function doesn't exist.
    """
    address = ctypes.addressof(ptr.contents) + offset
    new_ptr = ctypes.pointer(type(ptr.contents).from_address(address))
    return new_ptr

def byref_via_pointer(buf,offset):
    """
    Used for bref function w/ offset argument when working with
    versions of python < 2.6
    """
    buf_ptr = ctypes.pointer(buf)
    buf_ptr = pointer_incr(buf_ptr,offset)
    return buf_ptr

def get_python_vnum():
    """
    Returns python version number.
    """
    v = sys.version_info
    return v[0]+ 0.1*v[1]+0.01*v[2]

#-------------------------------------------------------------------------------------
if __name__ == '__main__':

    # USB parameters
    USB_VENDOR_ID = 0x0004
    USB_PRODUCT_ID = 0x0001
    USB_BULKOUT_EP_ADDRESS = 0x01
    USB_BULKIN_EP_ADDRESS = 0x82
    USB_BUFFER_OUT_SIZE = 64
    USB_BUFFER_IN_SIZE = 64

    # USB Command IDs
    USB_CMD_TEST8 = ctypes.c_uint8(100)
    USB_CMD_TEST16 = ctypes.c_uint8(101)
    USB_CMD_TEST32 = ctypes.c_uint8(102)
    USB_CMD_TEST_SET = ctypes.c_uint8(103)
    USB_CMD_TEST_GET = ctypes.c_uint8(104)
    USB_CMD_STRUCT_SET = ctypes.c_uint8(105)
    USB_CMD_STRUCT_GET = ctypes.c_uint8(106)
    USB_CMD_FLOAT_SET = ctypes.c_uint8(107)
    USB_CMD_FLOAT_GET = ctypes.c_uint8(108)

    dev = USB_Device()
    dev.print_values()
    print


    if 0:
        outdata = [USB_CMD_TEST8]
        intypes = [ctypes.c_uint8] + [ctypes.c_uint8]*60
        val_list = dev.usb_cmd(outdata,intypes)
        print val_list
        print

        outdata = [USB_CMD_TEST16]
        intypes = [ctypes.c_uint8] + [ctypes.c_uint16]*30
        val_list = dev.usb_cmd(outdata,intypes)
        print val_list
        print
        outdata = [USB_CMD_TEST32]
        intypes = [ctypes.c_uint8] + [ctypes.c_uint32]*15
        val_list = dev.usb_cmd(outdata,intypes)
        print val_list
        print

    if 0:
        outdata = [USB_CMD_TEST_GET]
        intypes = [ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint16, ctypes.c_uint32]
        val_list = dev.usb_cmd(outdata,intypes)
        print val_list
        print

        outdata = [USB_CMD_TEST_SET, ctypes.c_uint8(10), ctypes.c_uint16(20), ctypes.c_uint8(30)]
        intypes = [ctypes.c_uint8]
        val_list = dev.usb_cmd(outdata,intypes)
        print val_list
        print

        outdata = [USB_CMD_TEST_GET]
        intypes = [ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint16, ctypes.c_uint32]
        val_list = dev.usb_cmd(outdata,intypes)
        print val_list
        print

    if 0:
        class SysState_t(ctypes.Structure):
            _fields_ = [
                    ('Val8', ctypes.c_uint8),
                    ('Val16', ctypes.c_uint16),
                    ('Val32', ctypes.c_uint32),
                    ]

        print 'Getting current SysState structure'
        outdata = [USB_CMD_STRUCT_GET]
        intypes = [ctypes.c_uint8, SysState_t]
        val_list = dev.usb_cmd(outdata,intypes)
        cmd_id = val_list[0]
        SysState = val_list[1]
        print 'cmd_id:', cmd_id
        print 'SysState:', SysState.Val8, SysState.Val16, SysState.Val32
        print

        print 'Setting SysState Structure to new values'
        SysState.Val8 += 5
        SysState.Val16 += 5
        SysState.Val32 += 5
        outdata = [USB_CMD_STRUCT_SET, SysState]
        intypes = [ctypes.c_uint8]
        val_list = dev.usb_cmd(outdata,intypes)
        cmd_id = val_list[0]
        print 'cmd_id:', cmd_id
        print 'SysState:', SysState.Val8, SysState.Val16, SysState.Val32
        print

        print 'Getting current SysState structure'
        outdata = [USB_CMD_STRUCT_GET]
        intypes = [ctypes.c_uint8, SysState_t]
        val_list = dev.usb_cmd(outdata,intypes)
        cmd_id = val_list[0]
        SysState = val_list[1]
        print 'cmd_id:', cmd_id
        print 'SysState:', SysState.Val8, SysState.Val16, SysState.Val32
        print

    if 1:

        outdata = [USB_CMD_FLOAT_GET]
        intypes = [ctypes.c_uint8, ctypes.c_float]
        val_list = dev.usb_cmd(outdata,intypes)
        cmd_id = val_list[0]
        test_float = val_list[1]
        print 'cmd id:', cmd_id
        print 'test_float:', test_float
        print

        test_float = ctypes.c_float(test_float.value + 1.5)
        outdata = [USB_CMD_FLOAT_SET, test_float]
        intypes = [ctypes.c_uint8]
        val_list = dev.usb_cmd(outdata, intypes)
        print 'cmd id:', cmd_id
        print 'set float to:', test_float
        print

        outdata = [USB_CMD_FLOAT_GET]
        intypes = [ctypes.c_uint8, ctypes.c_float]
        val_list = dev.usb_cmd(outdata,intypes)
        cmd_id = val_list[0]
        test_float = val_list[1]
        print 'cmd id:', cmd_id
        print 'test_float:', test_float
        print


    dev.close()
