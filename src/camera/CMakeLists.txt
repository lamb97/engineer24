add_library(camera SHARED
        src/uvc_camera.cpp
        src/uvc_camera_data.cpp
        src/uvc_camera_hardware_param.cpp
        src/uvc_camera_model_param.cpp
        src/uvc_camera_param.cpp
        src/uvc_camera_runtime_param.cpp)
target_link_libraries(camera
                      ${OpenCV_LIBS}
                      common
                      logger)

add_executable(test_uvc_camera_offline
        test/test_uvc_camera_offline.cpp)
target_link_libraries(test_uvc_camera_offline
                      camera)

add_executable(test_uvc_camera_online
        test/test_uvc_camera_online.cpp)
target_link_libraries(test_uvc_camera_online
                      camera)

add_executable(test_uvc_camera_param
        test/test_uvc_camera_param.cpp)
target_link_libraries(test_uvc_camera_param
                      camera)

add_executable(test_uvc_camera_reconnect
        test/test_uvc_camera_reconnect.cpp)
target_link_libraries(test_uvc_camera_reconnect
                      camera)

add_executable(test_uvc_camera_switch_hardware_param
        test/test_uvc_camera_switch_hardware_param.cpp)
target_link_libraries(test_uvc_camera_switch_hardware_param
                      camera)