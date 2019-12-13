#!/usr/bin/env python3
# -*- coding: <utf-8> -*-

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


import os 
import argparse
import signal 
from datetime import datetime 
from dateutil import tz
import time
import os 
import numpy as np 

import sys

exclude_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if exclude_path in sys.path:
    sys.path.remove(exclude_path)

import cv2

from memsync import MemSync

FLAGS = None
mc = None
main_running = True

def MemSyncInit():
    global mc
    
    memsync_id = "NightVision"
    memsync_id += FLAGS.camera_num if FLAGS is not None else "1"
    memsync_id += "_Image"
    
    mc = MemSync()
    mc.SetServerInfo("localhost", "11211")
    mc.SetID(memsync_id)
    mc.GetHandle()

def sigHandler(_signo, _stack_frame):
    global main_running
    main_running = False 


def main():
    global mc

    signal.signal(signal.SIGTERM, sigHandler)
    signal.signal(signal.SIGABRT, sigHandler)
    signal.signal(signal.SIGINT, sigHandler)

    camera_num = FLAGS.camera_num if FLAGS is not None else "1"

    output_root = FLAGS.output if FLAGS is not None else "/work/data"
    output_filepath = os.path.join(output_root, "nv" + camera_num)
    if os.path.isdir(output_filepath) is False:
        os.makedirs(output_filepath)
    
    MemSyncInit()

    img_width = 720
    img_height = 480
    img_channel = 3

    try:
        # wait to reach start time
        start_time = datetime.now().time()
        if FLAGS.starttime is not None:
            start_time = datetime.strptime(FLAGS.starttime, '%H%M%S').time()
        if FLAGS.hour is not None:
            start_time.hour = int(FLAGS.hour)
        if FLAGS.minute is not None:
            start_time.minute = int(FLAGS.minute)
        if FLAGS.second is not None:
            start_time.second = int(FLAGS.second)
        
        while main_running:
            if datetime.now().time() < start_time:
                if FLAGS is not None and FLAGS.debug is True:
                    sys.stdout.write("\rNightVision Saver: Wait to {}, current time is {}".format(start_time, datetime.now().time()))
                time.sleep(0.5)
            else:
                break
        
        # saving loop
        if FLAGS is not None and FLAGS.debug is True:
            print("\nsaving night vision image")
        
        p_time_hook = ((time.time() * 1000) % 1000) // (1000 // FLAGS.rate)

        while main_running:
            # run every 100msec (10Hz)
            c_time_hook = ((time.time() * 1000) % 1000) // (1000 // FLAGS.rate)
            if c_time_hook != p_time_hook:
                data, timestamp = mc.Read()
                if data is not None:
                    nv_frame = np.frombuffer(data, dtype=np.uint8, count=img_height*img_width*img_channel)
                    nv_frame = np.reshape(nv_frame, (img_height, img_width, img_channel))

                    output_filename = FLAGS.prefix + '_NV' + FLAGS.camera_num + '_' + '{:%Y%m%d_%H%M%S}'.format(datetime.now()) + '_{:02d}'.format(int(c_time_hook)) + '.png'
                    cv2.imwrite(os.path.join(output_filepath, output_filename), nv_frame)  
 
                    if FLAGS is not None and FLAGS.viewer is True:
                        cv2.imshow("nv_frame" + FLAGS.camera_num, nv_frame)
                        cv2.waitKey(10)

                p_time_hook = c_time_hook 
            else:
                time.sleep(0.0333)

        if FLAGS is not None and FLAGS.debug is True:
            print("close output file and release memsync")
        mc.ReleaseHandle()

    except KeyboardInterrupt:
        mc.ReleaseHandle()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera_num", type=str, default="1", help="Camera Number")
    parser.add_argument("--output", type=str, default="/work/data", help="Output Root Path")
    parser.add_argument("--prefix", type=str, default="FX01", help="prefix filename")
    parser.add_argument("--starttime", type=str, default=None, help="start time")
    parser.add_argument("--hour", type=str, default=None, help="start hour")
    parser.add_argument("--minute", type=str, default=None, help="start minute")
    parser.add_argument("--second", type=str, default=None, help="start second")
    parser.add_argument("--viewer", type=bool, default=False, help="Viewer")
    parser.add_argument('--debug', type=bool, default=False, help="output debug message")
    parser.add_argument("--rate", type=int, default=10, help="saving rate(Hz)")

    FLAGS, _ = parser.parse_known_args()

    main()
