import numpy as np
import cv2
import os
import threading
import time
from PyQt5 import QtGui
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *


class VideoRecorder(QObject):
    def __init__(self,path: str,framename: str,filename: str ):
        super().__init__()
        self.open = True
        self.path = path
        self.framename = framename
        self.filename = filename


    def cam(self):
        self.cap = cv2.VideoCapture(self.path)
        # Define the codec and create VideoWriter object
        start = time.time()
        end = 100

        local_path = os.getcwd()
        file_path = os.path.join(local_path,self.filename)
        #print(file_path)
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter(file_path,fourcc, 25.0, (720,1280))
        while(self.cap.isOpened()):
            ret, frame = self.cap.read()
            if ret==True:
                #frame = cv2.rotate(frame,ROTATE_90_CLOCKWISE)
                # write the flipped frame
                #frame = cv2.resize(frame, (1280,720))
                frame = cv2.rotate(frame, rotateCode = 2)
                self.out.write(frame)
                cv2.imshow(self.framename,frame)
                if (cv2.waitKey(1) & 0xFF == ord('q')) or (end - (time.time() - start) < 0):
                    break
            else:
                break
        # Release everything if job is finished
        self.cap.release()
        self.out.release()
        cv2.destroyAllWindows()

    def abort(self):
        self.cap.release()
        self.out.release()
        cv2.destroyAllWindows()
        self.sig_msg.emit('Worker #{} notified to abort'.format(self.__id))
        self.__abort = True


# def get_thread(port,framename,dataprint):
#     import threading
#     temp = VideoRecorder()
#     t = threading.Thread(target=temp.cam, args=(port,framename,dataprint))
#     t.start()
#     return t



# t1 = get_thread('rtsp://192.168.1.2:8553/PSIA/Streaming/channels/1?videoCodecType=MPEG4','frame_1','output_1.avi')
# t2 = get_thread('rtsp://192.168.1.5:8553/PSIA/Streaming/channels/1?videoCodecType=MPEG4','frame_2','output_2.avi')

# t1.join()
# t2.join()

# print('stop')
