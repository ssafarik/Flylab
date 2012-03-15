# import roslib
# roslib.load_manifest('calibration_tf')
import cv
import rospy

# OLD.  Now we grab the data from the camera_info message.
# Find camera intrinsic paramters
# Could also subscribe to appropriate camera_info topic.
#def intrinsic(image_frame):
#    image_frame.lower()
#
#    intrinsic_matrix = cv.CreateMat(3, 3, cv.CV_32FC1)
#    cv.SetZero(intrinsic_matrix)
#
#    distortion_coeffs = cv.CreateMat(5, 1, cv.CV_32FC1)
#    cv.SetZero(distortion_coeffs)
#    
#    KK_fx = rospy.get_param("KK_fx_" + image_frame)
#    KK_fy = rospy.get_param("KK_fy_" + image_frame)
#    KK_cx = rospy.get_param("KK_cx_" + image_frame)
#    KK_cy = rospy.get_param("KK_cy_" + image_frame)
#    kc_k1 = rospy.get_param("kc_k1_" + image_frame)
#    kc_k2 = rospy.get_param("kc_k2_" + image_frame)
#    kc_p1 = rospy.get_param("kc_p1_" + image_frame)
#    kc_p2 = rospy.get_param("kc_p2_" + image_frame)
#    
#    cv.SetReal2D(intrinsic_matrix,0,0,KK_fx)
#    cv.SetReal2D(intrinsic_matrix,0,2,KK_cx)
#    cv.SetReal2D(intrinsic_matrix,1,1,KK_fy)
#    cv.SetReal2D(intrinsic_matrix,1,2,KK_cy)
#    cv.SetReal2D(intrinsic_matrix,2,2,1)
#    
#    cv.SetReal2D(distortion_coeffs,0,0,kc_k1)
#    cv.SetReal2D(distortion_coeffs,1,0,kc_k2)
#    cv.SetReal2D(distortion_coeffs,2,0,kc_p1)
#    cv.SetReal2D(distortion_coeffs,3,0,kc_p2)
#    
#    return intrinsic_matrix, distortion_coeffs


def extrinsic(frame):
    frame.lower()
    rvec = cv.CreateMat(1,3,cv.CV_32FC1)
    tvec = cv.CreateMat(1,3,cv.CV_32FC1)
    cv.SetZero(rvec)
    cv.SetZero(tvec)
    rvec_0 = rospy.get_param("camera_" + frame + "_rvec_0")
    rvec_1 = rospy.get_param("camera_" + frame + "_rvec_1")
    rvec_2 = rospy.get_param("camera_" + frame + "_rvec_2")
    tvec_0 = rospy.get_param("camera_" + frame + "_tvec_0")
    tvec_1 = rospy.get_param("camera_" + frame + "_tvec_1")
    tvec_2 = rospy.get_param("camera_" + frame + "_tvec_2")
    cv.SetReal2D(rvec,0,0,rvec_0)
    cv.SetReal2D(rvec,0,1,rvec_1)
    cv.SetReal2D(rvec,0,2,rvec_2)
    cv.SetReal2D(tvec,0,0,tvec_0)
    cv.SetReal2D(tvec,0,1,tvec_1)
    cv.SetReal2D(tvec,0,2,tvec_2)
    
    return rvec, tvec

