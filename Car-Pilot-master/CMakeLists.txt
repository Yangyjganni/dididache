cmake_minimum_required(VERSION 3.12)
project(FindRoad C)

set(CMAKE_C_STANDARD 11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --std=c11")

add_executable(FindRoad main.c index_map.c FindRoad.c FindRoad.h decodemessage.h decodemessage.c)