cmake_minimum_required(VERSION 3.10)
project(mc_rtc_controller)

find_package(mc_rtc REQUIRED)

add_controller(
  "HarambeCoMPosture"
  "mc_com_posture_controller.cpp"
  "mc_com_posture_controller.h"
)
