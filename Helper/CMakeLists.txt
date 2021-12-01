# libraries
add_library(${robot_gpu} STATIC robot.cu robot.cuh)
add_library(${robot_cpu} STATIC robot.cpp robot.h)

# add open cv if IMAGES=1
if (IMAGES)
    message("Outputting images to each evaluation step")

    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})

    target_link_libraries(${robot_cpu} ${OpenCV_LIBS})
    target_compile_definitions(${robot_cpu} PRIVATE IMAGES)

    target_link_libraries(${robot_gpu} ${OpenCV_LIBS})
    target_compile_definitions(${robot_gpu} PRIVATE IMAGES)

endif ()