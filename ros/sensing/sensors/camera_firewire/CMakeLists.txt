cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()
set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include(CheckIncludeFile)
include(CheckLibraryExists)
include(CheckFunctionExists)
include(CheckCXXSourceRuns)
include(CheckCXXCompilerFlag)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  set(SSE_FLAGS)

  set(CMAKE_REQUIRED_FLAGS "-msse2")
  check_cxx_source_runs("
    #include <emmintrin.h>

    int main()
    {
        __m128d a, b;
        double vals[2] = {0};
        a = _mm_loadu_pd(vals);
        b = _mm_add_pd(a,a);
        _mm_storeu_pd(vals,b);
        return 0;
     }"
     HAS_SSE2_EXTENSIONS)

  set(CMAKE_REQUIRED_FLAGS "-msse")
  check_cxx_source_runs("
    #include <xmmintrin.h>
    int main()
    {
        __m128 a, b;
        float vals[4] = {0};
        a = _mm_loadu_ps(vals);
        b = a;
        b = _mm_add_ps(a,b);
        _mm_storeu_ps(vals,b);
        return 0;
    }"
    HAS_SSE_EXTENSIONS)

  set(CMAKE_REQUIRED_FLAGS)

  if(HAS_SSE2_EXTENSIONS)
    message(STATUS "Using SSE2 extensions")
    set(SSE_FLAGS "-msse2 -mfpmath=sse")
  elseif(HAS_SSE_EXTENSIONS)
    message(STATUS "Using SSE extensions")
    set(SSE_FLAGS "-msse -mfpmath=sse")
  endif()

  add_definitions("-mpreferred-stack-boundary=4 ${SSE_FLAGS}")
elseif(MSVC)
  check_cxx_source_runs("
    #include <emmintrin.h>

    int main()
    {
        __m128d a, b;
        double vals[2] = {0};
        a = _mm_loadu_pd(vals);
        b = _mm_add_pd(a,a);
        _mm_storeu_pd(vals,b);
        return 0;
     }"
     HAS_SSE2_EXTENSIONS)
  if( HAS_SSE2_EXTENSIONS )
    message(STATUS "Using SSE2 extensions")
    add_definitions( "/arch:SSE2 /fp:fast -D__SSE__ -D__SSE2__" )
  endif()
endif()

rosbuild_add_library(dcam1394 src/dcam1394/dcam1394.cpp src/dcam1394/image_proc.cpp)

rosbuild_add_executable(imagescaler src/imagescaler.cpp)
rosbuild_add_executable(camera_firewire src/camera_firewire.cpp)
target_link_libraries(camera_firewire dcam1394)

# check for newer versions of opencv that support cvInitUndistortRectifyMap
# extract include dirs, libraries, and library dirs
rosbuild_invoke_rospack(camera_firewire camera_firewire cflags export --lang=cpp --attrib=cflags)
rosbuild_invoke_rospack(camera_firewire camera_firewire lflags export --lang=cpp --attrib=lflags)
foreach(arg ${camera_firewire_cflags})
  set(CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS} ${arg}")
endforeach()
foreach(arg ${camera_firewire_lflags})
  set(CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS} ${arg}")
  set(_used_lflags "${_used_lflags} ${arg}")
endforeach()
string(REGEX MATCHALL "(^| )-l([./+-_\\a-zA-Z]*)" _used_libs "${_used_lflags}")
string(REGEX REPLACE "(^| )-l" "" _used_libs "${_used_libs}")
set(CMAKE_REQUIRED_LIBRARIES ${_used_libs})
check_function_exists(cvInitUndistortRectifyMap HAVE_CV_UNDISTORT_RECTIFY_MAP)
if( HAVE_CV_UNDISTORT_RECTIFY_MAP )
  add_definitions("-DHAVE_CV_UNDISTORT_RECTIFY_MAP")
endif()
