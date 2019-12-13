#
# Copyright (C) 2019  <Jungwoo Lee, KIRO, Republic of Korea>
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 2.8)

set (CMAKE_BUILD_TYPE Debug)        # Release / RelWithDebInfo
set (CMAKE_VERBOSE_MAKEFILE true)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g3 -Wall")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g3 -fPIC")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fPIC")

project (rgbdepth)

find_package (OpenCV REQUIRED 
    opencv_core opencv_highgui opencv_imgproc opencv_imgcodecs 
    opencv_imgcodecs opencv_video
)

include_directories (../../memsync/cpp)
link_directories (./)

add_executable (${CMAKE_PROJECT_NAME}_grabber
    src/grabber.cpp 
)

target_link_libraries(${CMAKE_PROJECT_NAME}_grabber
    realsense2 
    opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs
    memsync
)

add_executable (${CMAKE_PROJECT_NAME}_saver
    src/saver.cpp 
)

target_link_libraries(${CMAKE_PROJECT_NAME}_saver 
    stdc++fs 
    opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs  
    memsync 
)
