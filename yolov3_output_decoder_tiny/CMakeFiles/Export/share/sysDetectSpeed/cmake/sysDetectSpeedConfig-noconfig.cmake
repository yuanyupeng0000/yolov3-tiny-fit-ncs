#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "sysDetectSpeed" for configuration ""
set_property(TARGET sysDetectSpeed APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(sysDetectSpeed PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "/usr/local/cuda-8.0/lib64/libcudart_static.a;-lpthread;dl;/usr/lib/x86_64-linux-gnu/librt.so;/data/ssd-caffe/caffe/build/lib/libcaffe.so;/usr/lib/x86_64-linux-gnu/libglog.so;/usr/lib/x86_64-linux-gnu/libgflags.so.2;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libGLEW.so.1.13;opencv_calib3d;opencv_core;opencv_cudaarithm;opencv_cudabgsegm;opencv_cudacodec;opencv_cudafeatures2d;opencv_cudafilters;opencv_cudaimgproc;opencv_cudalegacy;opencv_cudaobjdetect;opencv_cudaoptflow;opencv_cudastereo;opencv_cudawarping;opencv_cudev;opencv_dnn;opencv_features2d;opencv_flann;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_shape;opencv_stitching;opencv_superres;opencv_video;opencv_videoio;opencv_videostab"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/sysDetectSpeed/libsysDetectSpeed.so"
  IMPORTED_SONAME_NOCONFIG "libsysDetectSpeed.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS sysDetectSpeed )
list(APPEND _IMPORT_CHECK_FILES_FOR_sysDetectSpeed "${_IMPORT_PREFIX}/lib/sysDetectSpeed/libsysDetectSpeed.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
