import sys
from PyQt5 import QtGui
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import vtk
#import math
import numpy as np
import time
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import socket
import traceback

#import re
#import threading
#import skinematics as skin
#import matplotlib.pyplot as plt
import select
#import vector
#from scipy import interpolate
#from scipy.signal import savgol_filter

from datetime import datetime



SO_BIND = 0
SO_CONNECT = 1


def init_socket(port: int, flag, host=''):

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    if flag == SO_BIND:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.bind((host, port))
    elif flag == SO_CONNECT:
        s.connect((host, port))
    else:
        # TODO: add exception 
        return s

    return s


class Worker(QObject):
    """
    Must derive from QObject in order to emit signals, connect slots to other signals, and operate in a QThread.
    """

    sig_shifts = pyqtSignal(int, list)  # worker id, step description: emitted every 5 steps through work() loop
    sig_qtr = pyqtSignal(int, list)  # worker id, step description: emitted every step through work() loop
    sig_status = pyqtSignal(int, int)  # worker id: emitted status()
    sig_msg = pyqtSignal(str)  # message to be shown to user

    

    def __init__(self, id: int, port: int):
        super().__init__()
        self.__id = id
        self.port = port
        self.__abort = False
        self.sckt_in = init_socket(self.port, SO_BIND)
        self.sckt_out = init_socket(self.port, SO_CONNECT)
        self.start = time.time()
        self.offset = 0
        self.delay = 0
        # TODO: create ip finder

    @pyqtSlot()
    def work(self):

        te = 100
        
        emptylist = list()

        """
        Pretend this worker method does work that takes a long time. During this time, the thread's
        event loop is blocked, except if the application processEvents() is called: this gives every
        thread (incl. main) a chance to process events, which in this sample means processing signals
        received from GUI (such as abort).
        """
        thread_name = QThread.currentThread().objectName()
        thread_id = int(QThread.currentThreadId())  # cast to int() is necessary
        self.sig_msg.emit('Running worker #{} from thread "{}" (#{})'.format(self.__id, thread_name, thread_id))

        # for step in range(100):
        #     time.sleep(0.1)
        ##################################################

        while 1:
            
            self.sync_time()
            self.recieve_data()

    def sync_time(self):
        
        res_time=1000000

        mes_test = 'time'
        message = ''
        t1_s = time.time()
        str_ms = str((t1_s - self.start)*res_time).split('.')[0]
        cur_mes_test = mes_test + ',' + str_ms
    
        self.sckt_out.send(cur_mes_test.encode())

        while 1:
            try:
                if message[:len(mes_test)] == mes_test:
                    
                    t1_r = int(message.split(',')[2])
                    t2_s = int(message.split(',')[3])
                    str_ms = str((t2_r - self.start)*res_time).split('.')[0]
                    cur_mes_test = message + ',' + str_ms
                    
                    time.sleep(0.01)
                    t3_s = time.time()
                    str_ms = str((t3_s - self.start)*res_time).split('.')[0]
                    cur_mes_test = message + ',' + str_ms
                    self.sckt_out.send(cur_mes_test.encode())
                    break
                else:
                    mes, address = self.sckt_out.recvfrom(8192)
                    message = mes.decode()
                    t2_r = time.time()
                
            except socket.timeout:
                t1_s = time.time()
                str_ms = str((t1_s - self.start)*res_time).split('.')[0]
                cur_mes_test = mes_test + ',' + str_ms
                self.sckt_out.send(cur_mes_test.encode())
            #TODO: add exception for message 
            except (KeyboardInterrupt, SystemExit):
                raise
            except:
                traceback.print_exc()
    
        t1_s = int((t1_s - self.start) * res_time)
        t2_r = int((t2_r - self.start) * res_time)
    
        self.offset = (t1_r - t1_s - t2_r + t2_s) / 2
        self.delay = (t1_r - t1_s + t2_r - t2_s) / 2

        return None
        
    def receive_data(self, time_end=100):
        
        flag = 1
        x = list()
        times = list()
        linacc = list()
        rotmat = list()
        xnew = list()
        mean_vel = list()
        integrated_lin_acc = list()
        gist_times = list()
        e = np.matrix([[1.,0.,0.],[0.,1.,0],[0.,0.,1.]])
        i = 0
        shifts = [0.,0.,0.]
        times.append(0.)
        temp_max = 0
        M = 0.

        beg = time.time()
        while 1:
            try:                
                ready = select.select([s], [], [], 1)
                if (ready[0] == []):
                    self.sig_status.emit(self.__id, 0)

                message, address = self.socket.recvfrom(8192)
                sig_time = times[-1]

                d = parse(message) #time and dictionary

                if d == 1: 
                    continue
                
                if flag: #start time flag
                    st = float(d['time'])
                    #temp = st
                    #print(st)                

                #print(d['time'])
                
                d['time'] = [round(float(d['time']) - st,6)]
                
                d['linacc'] = [float(item) for item in d['linacc']]

                #emptylist.append(str(d['time']) + str( d['rotvec']))

                

                diff = time.time() - beg

                emptylist.append(', '.join(el for el in numpy.concatenate([d['time'],d['acc'],d['gyr'],d['mag'],d['grav'], d['linacc'],d['rotvec'][0:4],d['rotmat'][0:3],d['rotmat'][4:7],d['rotmat'][8:11]])))

                linacc.append(d['linacc'])
                if flag: #start time flag
                    times[0] = d['time']
                    flag = 0
                else:
                    times.append(d['time'])
                    temp_max = times[-1][0] - times[-2][0]
                    gist_times.append(temp_max)

                if temp_max > M:
                    M = temp_max
                    #print(temp_max)

                qtr = [float(item) for item in d['rotvec'][0:4]]

                # qtr = [float(d['rotvec'][2]),float(d['rotvec'][0]),float(d['rotvec'][1]),float(d['rotvec'][3])]


                #print(qtr)
                self.sig_qtr.emit(self.__id,qtr)

                #i += 1
                #length = 25 #длина окна усреденения
                #dis = 5 #5 #изменение траектории каждые 5 знчений
                #self.sig_shifts.emit(self.__id, shifts)
                #print(len(times), " ", i)

                ############################################################################################################       calculating shifts           
            #     if (i % dis == 0) and (i > 25): #(i % dis == 0) or (i > 25): #(i == 199): # (i == 990): (i == int(te*100-2))
            #         # print(linacc)
            #         accnp = np.array(linacc)
            #         x = np.array(times)                
            #         #print(times)
            #         # y0 = accnp[:,0]

            #         y0 = savgol_filter(accnp[:,0], lenth, 3)
            #         y1 = savgol_filter(accnp[:,1], lenth, 3)
            #         y2 = savgol_filter(accnp[:,2], lenth, 3)

            #         # print(len(x))
            #         # print(len(y0))
            #         # print(y0)

            #         tck0 = interpolate.splrep(x, y0, s=0)
            #         tck1 = interpolate.splrep(x, y1, s=0)
            #         tck2 = interpolate.splrep(x, y2, s=0)
                                
            #         ynew = interpolate.splev(x, tck0, der=0)

            #         mean_vel.append(np.mean(ynew[-dis:]))
            #         x_mean_vel = np.linspace(0,5, num = len(mean_vel))

            #         #print(ynew)
            #         #print(y0)  

            #         #print(len(x), ' ', abs(int(np.mean(ynew[-dis:]) * 1000)))


            #         # check = np.mean(ynew[-dis:]) * 1000.
            #         # if not(np.isnan(check)):
            #         #     if (abs(int(check)) > 10 ):

            #         yint0 = integ(x, tck0)
            #         yint1 = integ(x, tck1)
            #         yint2 = integ(x, tck2)
                    
            #         tck00 = interpolate.splrep(x, yint0, s=0)
            #         tck10 = interpolate.splrep(x, yint1, s=0)
            #         tck20 = interpolate.splrep(x, yint2, s=0)

            #         yint00 = integ(x, tck00)
            #         yint10 = integ(x, tck10)
            #         yint20 = integ(x, tck20)

            #         shifts = [yint20[-1,0],yint10[-1,0],yint00[-1,0]]
            #         shifts = [0.,0.,0.]
            #         self.sig_shifts.emit(self.__id, shifts)

            #         ##################################################################################################################
                                            
            #             # else:
            #             #     for j in range(dis): # корректировка траектории если скорость была мала
                                
            #             #         linacc[-1-j] = [0.0,0.0,0.0]

            #     # if (i > 498):
            #     #     plt.figure()
            #     #     plt.plot(x, ynew, x, yint0, x, yint00, '--', x_mean_vel, mean_vel ) #yint00, yint0, y0 #, x, yint0, x, yint00, '--'
            #     #     plt.legend(['data','velocity', 'trajectory'])
            #     #     plt.axis([0.0, 3., -2, 2])
            #     #     plt.title('Integral estimation from spline')
            #     #     plt.show()

                if (1) and (sig_time != times[-1]): #shifts != [0.,0.,0.]
                    self.sig_status.emit(self.__id, 1)
                    #print(times[-1],times[-2])

            #     # times.append(d['time'])
            #     # linacc.append([float(item) for item in d['linacc'] ])
            #     # rotmat.append( [[float(item) for item in d['rotmat'][0:3]], [float(item) for item in d['rotmat'][4:7]], [float(item) for item in d['rotmat'][8:11]]] )

            #         # check if we need to abort the loop; need to process events to receive signals;
            #     App.processEvents()  # this could cause change to self.__abort
            #     if self.__abort:
            #         # note that "step" value will not necessarily be same for every thread
            #         self.sig_msg.emit('Worker #{} aborting work at step {}'.format(self.__id, step))
            #         break

                if diff >= time_end:
                    with open('test_sensor_data/' + str(self.port)+ str(time.strftime("_%d_%m_%Y_%H_%M_%S",time.gmtime(time.time()))) + '.csv', 'a') as the_file:
                         the_file.write('\n'.join(el for el in emptylist))
                    print(str(self.port), " - stoped" )  
                    print(M)
                     #print(gist_times)
                     #print(times)  
                    break           
    
            except (KeyboardInterrupt, SystemExit):
                raise
            except:
                traceback.print_exc()
        
        ##################################################

    def abort(self):
        self.sig_msg.emit('Worker #{} notified to abort'.format(self.__id))
        self.__abort = True

def parse(mes):
    mes = mes.decode("utf-8").replace(" ", "")
    if (mes[0] == "#") :
        print('Sent:', mes[1:], '; Received: ', 'in {}s'.format(1))
        d = 1
    else :
        tm = mes.split("#")[0]
        d = dict([(el.split(",")[0], el.split(",")[1:]) for el in mes.split("#")[1:]])
        d["time"] = tm
    
    return d