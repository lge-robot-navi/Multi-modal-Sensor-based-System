#!/usr/bin/env python3


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
import time 
import threading 
import signal 
import numpy as np 

import sys

exclude_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if exclude_path in sys.path:
    sys.path.remove(exclude_path)

import cv2

from memsync import MemSync 

FLAGS = None 
nv_frame = None 
thread_lock = None
thread_running = True 
main_running = True
mc = None 

def initVideoDevice(video_num):
    camera = cv2.VideoCapture(video_num)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH,  720)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    camera.set(cv2.CAP_PROP_FPS, 30)
    return camera 

def grabVideoFrame(camera):
    grabbed = None
    if camera.grab():
        grabbed, frame = camera.retrieve()
    return frame if grabbed else None

def threadprocGrabFrame(video_num):
    global nv_frame 

    if FLAGS is not None and FLAGS.debug is True:
        print("Start GrabFrame threadproc")

    camera = initVideoDevice(video_num)
    if camera is None:
        print("Error:: camera handle is None")
        return

    while(thread_running):
        thread_lock.acquire()
        nv_frame = grabVideoFrame(camera)
        thread_lock.release()
        time.sleep(0.01)
    camera.release()

def initMemSync():
    global mc
    
    memsync_id = "NightVision"
    memsync_id += FLAGS.camera_num if FLAGS is not None else "1"
    memsync_id += "_Image"

    mc = MemSync()
    mc.SetServerInfo("localhost", "11211")
    mc.SetID(memsync_id)
    mc.GetHandle()

def sigHandler(_signo, _stack_frame):
    global thread_running
    global main_running

    thread_running = False
    main_running = False


def main():
    global nv_frame
    global thread_lock
    global thread_running
    global mc
    
    signal.signal(signal.SIGTERM, sigHandler)
    signal.signal(signal.SIGABRT, sigHandler)
    signal.signal(signal.SIGINT, sigHandler)

    thread_lock = threading.Lock()

    if FLAGS.type == 'local':
        video_num = FLAGS.video_num if FLAGS is not None else 0
        video_num = int(os.readlink("/dev/video_analog" + str(video_num))[5:])
        th = threading.Thread(target=threadprocGrabFrame, args=(video_num, ))
    else:
        video_uri = "http://view:view@{}/mjpg/video.mjpg".format(FLAGS.video_ip)
        th = threading.Thread(target=threadprocGrabFrame, args=(video_uri, ))
    th.daemon = False 
    th.start()

    initMemSync()

    try:
        while main_running:
            if nv_frame is None:
                print("failed to frame grabbing")
                time.sleep(1)
                continue 

            thread_lock.acquire()
            cur_frame = np.copy(nv_frame)
            thread_lock.release()

            if mc.ValidateHandle() is True:
                mc.Write(cur_frame.tobytes())

            if FLAGS is not None and FLAGS.viewer is True:            
                cv2.imshow("nv_frame", cur_frame)
                if cv2.waitKey(50) & 0xFF == ord('q'):
                    break
            else:
                time.sleep(0.05)
        
        if FLAGS is not None and FLAGS.debug is True:
            print("terminate thread and release memsync")

        thread_running = False 
        th.join()
        
        if FLAGS is not None and FLAGS.viewer is True:
            cv2.destroyAllWindows() 

        mc.ReleaseHandle()
    
    except KeyboardInterrupt:
        mc.ReleaseHandle()

    finally:
        if FLAGS is not None and FLAGS.debug is True:
            print("end")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--type', type=str, default='network', help="device type(local or network)")
    parser.add_argument('--video_ip', type=str, default='192.168.100.171', help="network camera ip")
    parser.add_argument('--video_num', type=int, default='1', help="Video Device Number(0..N)")
    parser.add_argument('--camera_num', type=str, default="1", help="NightVision Camera Number(1 or 2)")
    parser.add_argument('--viewer', type=bool, default=False, help="viewer(True or False)")
    parser.add_argument('--debug', type=bool, default=False, help="output debug message(True or False)")

    FLAGS, _ = parser.parse_known_args()

    main()
