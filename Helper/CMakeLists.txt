# libraries
add_library(${robot_gpu} STATIC robot.cu robot.cuh)
add_library(${robot_cpu} STATIC robot.cpp robot.h)