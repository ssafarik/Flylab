import cv
import numpy

def im_to_array(cv_im):
    depth2dtype = {
        cv.IPL_DEPTH_8U: 'uint8',
        cv.IPL_DEPTH_8S: 'int8',
        cv.IPL_DEPTH_16U: 'uint16',
        cv.IPL_DEPTH_16S: 'int16',
        cv.IPL_DEPTH_32S: 'int32',
        cv.IPL_DEPTH_32F: 'float32',
        cv.IPL_DEPTH_64F: 'float64',
        }

    arrdtype = depth2dtype[cv_im.depth]
    a = numpy.fromstring(cv_im.tostring(),
                         dtype = arrdtype,
                         count = cv_im.width * cv_im.height * cv_im.nChannels)
    a.shape = (cv_im.height, cv_im.width, cv_im.nChannels)
    return a

def mat_to_array(cv_mat):
    cvtype_to_dtype = {
        cv.CV_8UC1: 'uint8',
        cv.CV_8SC1: 'int8',
        cv.CV_16UC1: 'uint16',
        cv.CV_16SC1: 'int16',
        cv.CV_32SC1: 'int32',
        cv.CV_32FC1: 'float32',
        cv.CV_64FC1: 'float64',
        cv.CV_8UC2: 'uint8',
        cv.CV_8SC2: 'int8',
        cv.CV_16UC2: 'uint16',
        cv.CV_16SC2: 'int16',
        cv.CV_32SC2: 'int32',
        cv.CV_32FC2: 'float32',
        cv.CV_64FC2: 'float64',
        cv.CV_8UC3: 'uint8',
        cv.CV_8SC3: 'int8',
        cv.CV_16UC3: 'uint16',
        cv.CV_16SC3: 'int16',
        cv.CV_32SC3: 'int32',
        cv.CV_32FC3: 'float32',
        cv.CV_64FC3: 'float64',
        cv.CV_8UC4: 'uint8',
        cv.CV_8SC4: 'int8',
        cv.CV_16UC4: 'uint16',
        cv.CV_16SC4: 'int16',
        cv.CV_32SC4: 'int32',
        cv.CV_32FC4: 'float32',
        cv.CV_64FC4: 'float64',
        }

    ch_count = {
        cv.CV_8UC1: 1,
        cv.CV_8SC1: 1,
        cv.CV_16UC1: 1,
        cv.CV_16SC1: 1,
        cv.CV_32SC1: 1,
        cv.CV_32FC1: 1,
        cv.CV_64FC1: 1,
        cv.CV_8UC2: 2,
        cv.CV_8SC2: 2,
        cv.CV_16UC2: 2,
        cv.CV_16SC2: 2,
        cv.CV_32SC2: 2,
        cv.CV_32FC2: 2,
        cv.CV_64FC2: 2,
        cv.CV_8UC3: 3,
        cv.CV_8SC3: 3,
        cv.CV_16UC3: 3,
        cv.CV_16SC3: 3,
        cv.CV_32SC3: 3,
        cv.CV_32FC3: 3,
        cv.CV_64FC3: 3,
        cv.CV_8UC4: 4,
        cv.CV_8SC4: 4,
        cv.CV_16UC4: 4,
        cv.CV_16SC4: 4,
        cv.CV_32SC4: 4,
        cv.CV_32FC4: 4,
        cv.CV_64FC4: 4,
        }

    arrdtype = cvtype_to_dtype[cv.GetElemType(cv_mat)]
    ch = ch_count[cv.GetElemType(cv_mat)]
    a = numpy.fromstring(cv_mat.tostring(),
                         dtype = arrdtype,
                         count = cv_mat.rows * cv_mat.cols * ch)

    if cv_mat.rows == 1:
        row_count = ch
        col_count = cv_mat.cols
    elif cv_mat.cols == 1:
        row_count = cv_mat.rows
        col_count = ch
    else:
        row_count = cv_mat.rows
        col_count = cv_mat.cols

    a.shape = (row_count, col_count)
    return a

def array_to_im(a):
    dtype2depth = {
        'uint8':   cv.IPL_DEPTH_8U,
        'int8':    cv.IPL_DEPTH_8S,
        'uint16':  cv.IPL_DEPTH_16U,
        'int16':   cv.IPL_DEPTH_16S,
        'int32':   cv.IPL_DEPTH_32S,
        'float32': cv.IPL_DEPTH_32F,
        'float64': cv.IPL_DEPTH_64F,
        }
    if a.dtype.name in 'int64':
        a = a.astype('int32')

    if a.ndim == 1:
        a = numpy.array([a])

    try:
        nChannels = a.shape[2]
    except:
        nChannels = 1
        cv_im = cv.CreateImageHeader((a.shape[1],a.shape[0]),
                                     dtype2depth[a.dtype.name],
                                     nChannels)
        cv.SetData(cv_im,
                     a.tostring('C'),
                     a.dtype.itemsize*nChannels*a.shape[1])
        return cv_im

def array_to_mat(a,ch=1,rowcol='r'):
    ch1 = {
        'uint8':   cv.CV_8UC1,
        'int8':    cv.CV_8SC1,
        'uint16':  cv.CV_16UC1,
        'int16':   cv.CV_16SC1,
        'int32':   cv.CV_32SC1,
        'float32': cv.CV_32FC1,
        'float64': cv.CV_64FC1,
        }
    ch2 = {
        'uint8':   cv.CV_8UC2,
        'int8':    cv.CV_8SC2,
        'uint16':  cv.CV_16UC2,
        'int16':   cv.CV_16SC2,
        'int32':   cv.CV_32SC2,
        'float32': cv.CV_32FC2,
        'float64': cv.CV_64FC2,
        }
    ch3 = {
        'uint8':   cv.CV_8UC3,
        'int8':    cv.CV_8SC3,
        'uint16':  cv.CV_16UC3,
        'int16':   cv.CV_16SC3,
        'int32':   cv.CV_32SC3,
        'float32': cv.CV_32FC3,
        'float64': cv.CV_64FC3,
        }
    ch4 = {
        'uint8':   cv.CV_8UC4,
        'int8':    cv.CV_8SC4,
        'uint16':  cv.CV_16UC4,
        'int16':   cv.CV_16SC4,
        'int32':   cv.CV_32SC4,
        'float32': cv.CV_32FC4,
        'float64': cv.CV_64FC4,
        }
    dtype_to_cvtype = [ch1,ch2,ch3,ch4]

    if a.dtype.name in 'int64':
        a = a.astype('int32')

    rowcol = rowcol.lower()

    if a.ndim == 1:
        a = numpy.array([a])
        if rowcol.startswith('c'):
            a = a.transpose()

    if ch == 1:
        row_count = a.shape[0]
        col_count = a.shape[1]
    else:
        if (a.shape[0] == ch) and (a.shape[1] == ch):
            if rowcol.startswith('c'):
                row_count = 1
                col_count = a.shape[1]
            else:
                row_count = a.shape[0]
                col_count = 1
        elif a.shape[0] == ch:
            row_count = 1
            col_count = a.shape[1]
        elif a.shape[1] == ch:
            row_count = a.shape[0]
            col_count = 1
        else:
            raise

    cv_mat = cv.CreateMat(row_count,
                          col_count,
                          dtype_to_cvtype[ch-1][a.dtype.name])
    cv.SetData(cv_mat,
               a.tostring('C'),
               a.dtype.itemsize*col_count*ch)
    return cv_mat
