cmake_minimum_required(VERSION 3.12)
project(FindRoad C)

set(CMAKE_C_STANDARD 11)

add_executable(FindRoad main.c index_map.c FindRoad.c FindRoad.h decodemessage.h decodemessage.c)