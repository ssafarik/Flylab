import sys
import ctypes

__all__ = ['USBError','USBNoDataAvailableError','bulk_read','bulk_write',
           'claim_interface', 'release_interface', 'close', 'find_busses',
           'find_devices','get_busses', 'get_string_simple', 'init','interrupt_read',
           'interrupt_write','open', 'set_configuration','clear_halt','set_debug']

if sys.platform.startswith('linux'):
    __all__.extend(['get_driver_np','detach_kernel_driver_np'])

packed_on_all = True

if sys.platform.startswith('win'):
    packed_on_windows_only = True
else:
    packed_on_windows_only = False

class USBError(RuntimeError):
    pass

class USBNoDataAvailableError(USBError):
    pass

if sys.platform.startswith('linux'):
    # c_libusb_shared_library = '/usr/lib/libusb.so'
    c_libusb_shared_library = '/usr/lib/libusb-0.1.so.4'
    c_libusb = ctypes.cdll.LoadLibrary(c_libusb_shared_library)
elif sys.platform.startswith('win'):
    c_libusb = ctypes.CDLL(r'C:\WINDOWS\system32\libusb0.dll')
elif sys.platform.startswith('darwin'):
    c_libusb_shared_library = '/Library/Frameworks/libusb.framework/Versions/Current/libusb'
    c_libusb = ctypes.cdll.LoadLibrary(c_libusb_shared_library)

#####################################
# typedefs and defines
if sys.platform.startswith('linux') or sys.platform.startswith('darwin'):
    PATH_MAX = 4096 # HACK! should get from header file...
    LIBUSB_PATH_MAX = PATH_MAX+1
elif sys.platform.startswith('win'):
    LIBUSB_PATH_MAX = 512 # From usb.h of win32 libusb source

if hasattr(ctypes,'c_uint8'):
    uint8 = ctypes.c_uint8
    uint16 = ctypes.c_uint16
else:
    uint8 = ctypes.c_ubyte
    uint16 = ctypes.c_ushort

# datatypes
class usb_device_descriptor(ctypes.Structure):
    _pack_ = packed_on_all
class usb_device(ctypes.Structure):
    _pack_ = packed_on_windows_only
class usb_bus(ctypes.Structure):
    _pack_ = packed_on_windows_only
class usb_config_descriptor(ctypes.Structure):
    _pack_ = packed_on_windows_only
class usb_interface_descriptor(ctypes.Structure):
    _pack_ = packed_on_windows_only
class usb_interface(ctypes.Structure):
    _pack_ = packed_on_windows_only
class usb_dev_handle(ctypes.Structure):
    _pack_ = packed_on_windows_only
class usb_endpoint_descriptor(ctypes.Structure):
    _pack_ = packed_on_windows_only

# Pointer types
usb_device_p = ctypes.POINTER(usb_device)
usb_bus_p = ctypes.POINTER(usb_bus)
usb_dev_handle_p = ctypes.POINTER(usb_dev_handle)
usb_config_descriptor_p = ctypes.POINTER(usb_config_descriptor)
usb_interface_p = ctypes.POINTER(usb_interface)
usb_interface_descriptor_p = ctypes.POINTER(usb_interface_descriptor)
usb_endpoint_descriptor_p = ctypes.POINTER(usb_endpoint_descriptor)

# structures
usb_device_descriptor._fields_ = [
    ('bLength',uint8),
    ('bDescriptorType',uint8),
    ('bcdUSB',uint16),
    ('bDeviceClass',uint8),
    ('bDeviceSubClass',uint8),
    ('bDeviceProtocol',uint8),
    ('bMaxPacketSize0',uint8),
    ('idVendor',uint16),
    ('idProduct',uint16),
    ('bcdDevice',uint16),
    ('iManufacturer',uint8),
    ('iProduct',uint8),
    ('iSerialNumber',uint8),
    ('bNumConfigurations',uint8)]

usb_device._fields_ = [
    ('next',usb_device_p),
    ('prev',usb_device_p),
    ('filename',ctypes.c_char*(LIBUSB_PATH_MAX)),
    ('bus',usb_bus_p),
    ('descriptor',usb_device_descriptor),
    ('config',usb_config_descriptor_p),
    ('dev',ctypes.c_void_p),
    ('devnum',uint8),
    ('num_children',uint8),
    ('children',ctypes.POINTER(usb_device_p))
    ]

usb_bus._fields_ = [
    ('next',usb_bus_p),
    ('prev',usb_bus_p),
    ('dirname',ctypes.c_char*(LIBUSB_PATH_MAX)),
    ('devices',usb_device_p),
    ('location',ctypes.c_ulong),
    ('root_dev',usb_device_p),
    ]

usb_config_descriptor._fields_ = [
    ('bLength',uint8),
    ('bDescriptorType',uint8),
    ('wTotalLength',uint16),
    ('bNumInterfaces',uint8),
    ('bConfigurationValue',uint8),
    ('iConfiguration',uint8),
    ('bmAttributes',uint8),
    ('MaxPower',uint8),
    ('interface',usb_interface_p),
    ('extra',ctypes.c_char_p),
    ('extralen',ctypes.c_int)
    ]

usb_interface_descriptor._fields_ = [
    ('bLength',uint8),
    ('bDescriptorType',uint8),
    ('bInterfaceNumber',uint8),
    ('bAlternateSetting',uint8),
    ('bNumEndpoints',uint8),
    ('bInterfaceClass',uint8),
    ('bInterfaceSubClass',uint8),
    ('bInterfaceProtocol',uint8),
    ('iInterface',uint8),
    ('endpoint',usb_endpoint_descriptor_p),
    ('extra',ctypes.c_char_p),
    ('extralen',ctypes.c_int),
    ]

usb_interface._fields_ = [
    ('altsetting',usb_interface_descriptor_p),
    ('num_altsetting',ctypes.c_int),
    ]

usb_endpoint_descriptor._fields_ = [
    ('bLength',uint8),
    ('bDescriptorType',uint8),
    ('bEndpointAddress',uint8),
    ('bmAttributes',uint8),
    ('wMaxPacketSize',uint16),
    ('bInterval',uint8),
    ('bRefresh',uint8),
    ('bSynchAddress',uint8),
    ('extra',ctypes.c_char_p),
    ('extralen',ctypes.c_int),
    ]

# structure wrappers
class _interface(object):
    """wraps struct usb_interface*"""
    def __init__(self,cval):
        if type(cval) != usb_interface_p:
            raise TypeError('need struct usb_interface*')
        self.cval = cval
    def get_num_altsetting(self):
        return self.cval.contents.num_altsetting
    num_altsetting = property(get_num_altsetting)
    def get_altsetting(self):
        result = []
        for i in range(self.num_altsetting):
            result.append( interface_descriptor( ctypes.pointer(self.cval.contents.altsetting[i]) ))
        return result
    altsetting = property(get_altsetting)

class _endpoint(object):
    """wraps struct usb_endpoint_descriptor*"""
    def __init__(self,cval):
        if type(cval) != usb_endpoint_descriptor_p:
            raise TypeError('need struct usb_endpoint_descriptor*')
        self.cval = cval

    def get_bLength(self):
        return self.cval.contents.bLength
    bLength = property(get_bLength)

    def get_bDescriptorType(self):
        return self.cval.contents.bDescriptorType
    bDescriptorType = property(get_bDescriptorType)

    def get_bEndpointAddress(self):
        return self.cval.contents.bEndpointAddress
    bEndpointAddress = property(get_bEndpointAddress)

    def get_bmAttributes(self):
        return self.cval.contents.bmAttributes
    bmAttributes = property(get_bmAttributes)

    def get_wMaxPacketSize(self):
        return self.cval.contents.wMaxPacketSize
    wMaxPacketSize = property(get_wMaxPacketSize)

    def get_bInterval(self):
        return self.cval.contents.bInterval
    bInterval = property(get_bInterval)

    def get_bRefresh(self):
        return self.cval.contents.bRefresh
    bRefresh = property(get_bRefresh)

    def get_bSynchAddress(self):
        return self.cval.contents.bSynchAddress
    bSynchAddress = property(get_bSynchAddress)

class interface_descriptor(object):
    """wraps struct usb_interface_descriptor*"""
    def __init__(self,cval):
        if type(cval) != usb_interface_descriptor_p:
            raise TypeError('need struct usb_interface_descriptor*')
        self.cval = cval

    def get_bLength(self):
        return self.cval.contents.bLength
    bLength = property(get_bLength)

    def get_bDescriptorType(self):
        return self.cval.contents.bDescriptorType
    bDescriptorType = property(get_bDescriptorType)

    def get_bInterfaceNumber(self):
        return self.cval.contents.bInterfaceNumber
    bInterfaceNumber = property(get_bInterfaceNumber)

    def get_bAlternateSetting(self):
        return self.cval.contents.bAlternateSetting
    bAlternateSetting = property(get_bAlternateSetting)

    def get_bNumEndpoints(self):
        return self.cval.contents.bNumEndpoints
    bNumEndpoints = property(get_bNumEndpoints)

    def get_bInterfaceClass(self):
        return self.cval.contents.bInterfaceClass
    bInterfaceClass = property(get_bInterfaceClass)

    def get_bInterfaceSubClass(self):
        return self.cval.contents.bInterfaceSubClass
    bInterfaceSubClass = property(get_bInterfaceSubClass)

    def get_bInterfaceProtocol(self):
        return self.cval.contents.bInterfaceProtocol
    bInterfaceProtocol = property(get_bInterfaceProtocol)

    def get_iInterface(self):
        return self.cval.contents.iInterface
    iInterface = property(get_iInterface)

    def get_endpoint(self):
        result = []
        for i in range(self.bNumEndpoints):
            result.append( _endpoint( ctypes.pointer(self.cval.contents.endpoint[i])) )
        return result
    endpoint = property(get_endpoint)

class config_descriptor(object):
    """wraps struct usb_config_descriptor"""
    def __init__(self,cval):
        if type(cval) != usb_config_descriptor:
            raise TypeError('need struct usb_config_descriptor')
        self.cval = cval

    def get_bLength(self):
        return self.cval.bLength
    bLength = property(get_bLength)

    def get_bDescriptorType(self):
        return self.cval.bDescriptorType
    bDescriptorType = property(get_bDescriptorType)

    def get_wTotalLength(self):
        return self.cval.wTotalLength
    wTotalLength = property(get_wTotalLength)

    def get_bNumInterfaces(self):
        return self.cval.bNumInterfaces
    bNumInterfaces = property(get_bNumInterfaces)

    def get_bConfigurationValue(self):
        return self.cval.bConfigurationValue
    bConfigurationValue = property(get_bConfigurationValue)

    def get_iConfiguration(self):
        return self.cval.iConfiguration
    iConfiguration = property(get_iConfiguration)

    def get_bmAttributes(self):
        return self.cval.bmAttributes
    bmAttributes = property(get_bmAttributes)

    def get_MaxPower(self):
        return self.cval.MaxPower
    MaxPower = property(get_MaxPower)

    def get_interface(self):
        result = []
        n_interfaces = self.bNumInterfaces
        for i in range(n_interfaces):
            result.append(_interface( ctypes.pointer(self.cval.interface[i])) )
        return result
    interface = property(get_interface)

class device_descriptor(object):
    """wraps usb_device_descriptor structure"""
    def __init__(self,cval):
        if type(cval) != usb_device_descriptor:
            raise TypeError('need struct usb_device_descriptor')
        self.cval = cval

    def get_idVendor(self):
        return self.cval.idVendor
    idVendor = property(get_idVendor)

    def get_idProduct(self):
        return self.cval.idProduct
    idProduct = property(get_idProduct)

    def get_bcdDevice(self):
        return self.cval.bcdDevice
    bcdDevice = property(get_bcdDevice)

    def get_iManufacturer(self):
        return self.cval.iManufacturer
    iManufacturer = property(get_iManufacturer)

    def get_iProduct(self):
        return self.cval.iProduct
    iProduct = property(get_iProduct)

    def get_iSerialNumber(self):
        return self.cval.iSerialNumber
    iSerialNumber = property(get_iSerialNumber)

    def get_bNumConfigurations(self):
        return self.cval.bNumConfigurations
    bNumConfigurations = property(get_bNumConfigurations)

class _device(object):
    """wraps pointer to usb_device structure"""
    def __init__(self,cval):
        if type(cval) != usb_device_p:
            raise TypeError('need pointer to struct usb_device')
        self.cval = cval
        self.next = self # prepare for iterating
        if not bool(cval):
            cval = None
            self.next = None
    def __iter__(self):
        return self
    def next(self):
        result = self.next
        if result is None:
            raise StopIteration
        # prepare for next iteration
        self.next = _CheckDevice(result.cval.contents.next)
        return result
    def get_descriptor(self):
        return device_descriptor(self.cval.contents.descriptor)
    descriptor = property(get_descriptor)
    def get_config(self):
        result = []
        n_configs = self.descriptor.bNumConfigurations
        for i in range(n_configs):
            result.append(config_descriptor( self.cval.contents.config[i] ))
        return result
    config = property(get_config)

class bus(object):
    """wraps pointer to usb_bus structure"""
    def __init__(self,cval):
        if type(cval) != usb_bus_p:
            raise TypeError('need pointer to struct usb_bus')
        self.cval = cval
        self.next = self # prepare for iterating
    def __iter__(self):
        return self
    def next(self):
        result = self.next
        if result is None:
            raise StopIteration
        # prepare for next iteration
        self.next = _CheckBus(result.cval.contents.next)
        return result
    def get_devices(self):
        devices = self.cval.contents.devices
        result = _device(devices)
        return result
    devices = property(get_devices)

def _CheckBus(b):
    if bool(b):
        return bus(b)
    else:
        return None

def _CheckDevice(b):
    if bool(b):
        return _device(b)
    else:
        return None

#####################################
# function definitions
c_libusb.usb_get_busses.restype = usb_bus_p
c_libusb.usb_open.restype = usb_dev_handle_p
c_libusb.usb_strerror.restype = ctypes.c_char_p

c_libusb.usb_close.restype = ctypes.c_int
c_libusb.usb_close.argtypes = [usb_dev_handle_p]

c_libusb.usb_get_string_simple.restype = ctypes.c_int
c_libusb.usb_get_string_simple.argtypes = [usb_dev_handle_p,
                                           ctypes.c_int,
                                           ctypes.c_char_p,
                                           ctypes.c_int]

c_libusb.usb_bulk_read.argtypes = [usb_dev_handle_p, ctypes.c_int,
                                        ctypes.c_char_p, ctypes.c_int, ctypes.c_int]
c_libusb.usb_bulk_write.argtypes = [usb_dev_handle_p, ctypes.c_int,
                                         ctypes.c_char_p, ctypes.c_int, ctypes.c_int]
c_libusb.usb_claim_interface.argtypes = [usb_dev_handle_p, ctypes.c_int]
c_libusb.usb_release_interface.argtypes = [usb_dev_handle_p, ctypes.c_int]
c_libusb.usb_interrupt_read.argtypes = [usb_dev_handle_p, ctypes.c_int,
                                        ctypes.c_char_p, ctypes.c_int, ctypes.c_int]
c_libusb.usb_interrupt_write.argtypes = [usb_dev_handle_p, ctypes.c_int,
                                         ctypes.c_char_p, ctypes.c_int, ctypes.c_int]
c_libusb.usb_set_configuration.argtypes = [usb_dev_handle_p, ctypes.c_int]

c_libusb.usb_clear_halt.argtypes = [usb_dev_handle_p, ctypes.c_int]

#####################################
# wrapper

def CHK(result):
    if result < 0:
        errstr = c_libusb.usb_strerror()
        if errstr == "could not get bound driver: No data available":
            exc_class = USBNoDataAvailableError
        else:
            exc_class = USBError
        raise exc_class("%d: %s"%(result,errstr))
    return result

def bulk_read(libusb_handle,endpoint,buf,timeout):
    if not isinstance(libusb_handle,usb_dev_handle_p):
        raise ValueError("expected instance of usb_dev_handle_p")
    return CHK(c_libusb.usb_bulk_read(libusb_handle, endpoint,
                                    buf, len(buf), timeout))

def bulk_write(libusb_handle,endpoint,buf,timeout):
    if not isinstance(libusb_handle,usb_dev_handle_p):
        raise ValueError("expected instance of usb_dev_handle_p")
    return CHK(c_libusb.usb_bulk_write(libusb_handle, endpoint,
                                    buf, len(buf), timeout))

def claim_interface(libusb_handle,value):
    if not isinstance(libusb_handle,usb_dev_handle_p):
        raise ValueError("expected instance of usb_dev_handle_p")
    return CHK(c_libusb.usb_claim_interface(libusb_handle, value))

def release_interface(libusb_handle,value):
    if not isinstance(libusb_handle,usb_dev_handle_p):
        raise ValueError("expected instance of usb_dev_handle_p")
    return CHK(c_libusb.usb_release_interface(libusb_handle, value))

def find_busses():
    c_libusb.usb_find_busses()

def find_devices():
    c_libusb.usb_find_devices()

def get_busses():
    return _CheckBus(c_libusb.usb_get_busses())

def init():
    c_libusb.usb_init()

def interrupt_read(libusb_handle,endpoint,buf,timeout):
    if not isinstance(libusb_handle,usb_dev_handle_p):
        raise ValueError("expected instance of usb_dev_handle_p")
    return CHK(c_libusb.usb_interrupt_read(libusb_handle, endpoint,
                                           buf, len(buf), timeout))

def interrupt_write(libusb_handle,endpoint,buf,timeout):
    if not isinstance(libusb_handle,usb_dev_handle_p):
        raise ValueError("expected instance of usb_dev_handle_p")
    return CHK(c_libusb.usb_interrupt_write(libusb_handle, endpoint,
                                            buf, len(buf), timeout))

def open(dev):
    if not isinstance(dev,_device):
        raise ValueError('open() must be called with pylibusb._device instance')
    libusb_handle = c_libusb.usb_open(dev.cval)
    if not bool(libusb_handle):
        raise USBError("could not open device '%s'"%str(dev))
    return libusb_handle

def close(libusb_handle):
    if not isinstance(libusb_handle,usb_dev_handle_p):
        raise ValueError("expected instance of usb_dev_handle_p")
    return CHK(c_libusb.usb_close(libusb_handle))

def get_string_simple(libusb_handle,index):
    if not isinstance(libusb_handle,usb_dev_handle_p):
        raise ValueError("expected instance of usb_dev_handle_p")
    buflen = 256
    buf = ctypes.create_string_buffer(buflen)
    CHK(c_libusb.usb_get_string_simple(libusb_handle,
                                       index,buf,buflen))
    return buf.value

def set_configuration(libusb_handle,value):
    if not isinstance(libusb_handle,usb_dev_handle_p):
        raise ValueError("expected instance of usb_dev_handle_p")
    return CHK(c_libusb.usb_set_configuration(libusb_handle, value))

def clear_halt(libusb_handle,value):
    if not isinstance(libusb_handle,usb_dev_handle_p):
        raise ValueError("expected instance of usb_dev_handle_p")
    return CHK(c_libusb.usb_clear_halt(libusb_handle, value))

def set_debug(val):
    c_libusb.usb_set_debug(val)

# Platform-specific (non-portable) additions

if sys.platform.startswith('linux'):
    def get_driver_np(libusb_handle,interface):
        if not isinstance(libusb_handle,usb_dev_handle_p):
            raise ValueError("expected instance of usb_dev_handle_p")
        LEN = 55
        name = ctypes.create_string_buffer(LEN)
        try:
            CHK(c_libusb.usb_get_driver_np(
                libusb_handle, interface, ctypes.byref(name), LEN))
        except USBNoDataAvailableError, err:
            return ''
        return name.value
    def detach_kernel_driver_np(libusb_handle,interface):
        if not isinstance(libusb_handle,usb_dev_handle_p):
            raise ValueError("expected instance of usb_dev_handle_p")
        return CHK(c_libusb.usb_detach_kernel_driver_np(libusb_handle, interface))


