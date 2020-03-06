import numpy as np
import cv2
import os
import threading


def cam(way,framename,filename):
    cap = cv2.VideoCapture(way)
    # Define the codec and create VideoWriter object

    local_path = os.getcwd()
    file_path = os.path.join(local_path,filename)
    #print(file_path)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter( file_path,fourcc, 25.0, (1280,720))
    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret==True:
            #frame = cv2.rotate(frame,ROTATE_90_CLOCKWISE)
            # write the flipped frame
            #frame = cv2.resize(frame, (1280,720))
            #frame = cv2.rotate(frame, rotateCode = 1)
            out.write(frame)
            frame = np.rot90(frame)
            cv2.imshow(framename,frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break
    # Release everything if job is finished
    cap.release()
    out.release()
    cv2.destroyAllWindows()

# start_time = time.time()
# end_time = 100


def get_thread(port,framename,dataprint):
    import threading
    t = threading.Thread(target=cam, args=(port,framename,dataprint))
    t.start()
    return t



t1 = get_thread('rtsp://192.168.1.2:8553/PSIA/Streaming/channels/1?videoCodecType=MPEG4','frame_1','output_1.avi')
t2 = get_thread('rtsp://192.168.1.5:8553/PSIA/Streaming/channels/1?videoCodecType=MPEG4','frame_2','output_2.avi')

t1.join()
t2.join()

print('stop')
