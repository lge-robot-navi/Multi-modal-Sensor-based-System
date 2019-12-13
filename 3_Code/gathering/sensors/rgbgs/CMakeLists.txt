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

set (CMAKE_BUILD_TYPE Debug)		# Release / RelWithDebInfo / MinSizeRel
set (CMAKE_VERBOSE_MAKEFILE true)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g3")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g3 -fPIC")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fPIC")

project (rgbgs)

find_package (OpenCV REQUIRED 
	opencv_core opencv_highgui opencv_imgproc opencv_video
)

set (PLEORA_BASE_DIR /opt/pleora/ebus_sdk/Ubuntu-14.04-x86_64/)
set (PLEORA_INCLUDE_DIR ${PLEORA_BASE_DIR}/include/)
set (PLEORA_LIBRARY_DIR ${PLEORA_BASE_DIR}/lib/)

set (PLEORA_GENICAM_ROOT ${PLEORA_BASE_DIR}/lib/genicam)

include_directories (${PLEORA_INCLUDE_DIR} ./include ../../memsync/cpp)
link_directories (${PLEORA_LIBRARY_DIR} ./ )

add_executable (${CMAKE_PROJECT_NAME}_grabber 
	src/gev_camera.cpp
	src/grabber.cpp 
)

target_link_libraries(${CMAKE_PROJECT_NAME}_grabber 
	PvBase PvBuffer PvStream PvDevice PvGenICam 
	opencv_core opencv_highgui 
	pthread
	memsync 
)

add_executable(${CMAKE_PROJECT_NAME}_saver 
	src/saver.cpp 
)

target_link_libraries(${CMAKE_PROJECT_NAME}_saver 
	stdc++fs 
	opencv_core opencv_highgui 
	memsync 
)

