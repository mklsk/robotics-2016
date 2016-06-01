cmake_minimum_required(VERSION 3.5)
project(Exam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES race.c)
set(SOURCE_FILES race_final.c)
set(SOURCE_FILES submission.c)
add_executable(Exam ${SOURCE_FILES})