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
import Worker as Worker
import Draw
import QtrCalc as QC
import Camera_2 
from scipy.spatial.transform import Rotation as R
import network
import os
from scipy import interpolate
from utils import trap_exc_during_debug

import CalibIMUs


def get_files(directory):
    files = os.listdir(directory)
    for i in range(0,len(files)):
        files[i] = directory + files[i]
    return files

def integ(x, tck, constant = 10e-9):
    x = np.atleast_1d(x)
    out = np.zeros(x.shape, dtype=x.dtype)
    for n in range(len(out)):
        out[n] = interpolate.splint(0, x[n], tck)
    out += constant
    return out

def swap(self, i, j): #поменять местами два элемента массива
    s = self.a[i]
    self.a[i] = self.a[j]
    self.a[j] = s

def NextSet(self, n):                #операция перестановки
    
    j = n-2
    while ( (j!= -1) and (self.a[j] >= self.a[j+1]) ):
        j-=1
    if (j == -1):
        return False
    k = n - 1
    while (self.a[j] >= self.a[k]):
        k-=1    
    swap(self, j, k)   

    l = j + 1
    r = n - 1
    while (l < r):
        swap(self, l , r)
        l += 1
        r -= 1
    return True

class MainWindow(QMainWindow):   

    NUM_THREADS = 7 # number of phones
    sig_abort = pyqtSignal()
    sig_abort_workers = pyqtSignal()
    sig_abort_cams = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.title = "Система сбора, обработки и визуализации деятельности работников предприятия"
        self.top = 200
        self.left = 500
        self.width = 1000
        self.height = 800
        self.WorkerAdress = list()
        self.num_workers = 0

        self.rigth_list = [ 5559+2*i for i in range(self.NUM_THREADS)]

        mtxBnoAtVtk = np.array([[1,0,0], [0,0,-1], [0,1,0]])

        mtxBnoAtVtk_norm = np.array([[0,0,-1], [0,1,0], [1,0,0]])

        self.rotBnoAtVtk = R.from_matrix(mtxBnoAtVtk)
        self.rotBnoAtVtk_norm = R.from_matrix(mtxBnoAtVtk_norm)
        self.rot0 = R.from_matrix(np.identity(3))
        self.rot1 = self.rot0
        self.rotBox0AtIMU0 = self.rot0
        self.rotBox0AtIMU1 = self.rot1
        self.rotBox0AtIMU2 = self.rot1
        
        self.setWindowIcon(QtGui.QIcon("icon.png"))
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        
        self.createMenu()
        self.createSensorsDock()  

        self.createToolBar()  

        self.createExplorerDock()     
        
        self.createLogDockWidget()
        
        self.createCentralWidget()

        self.configureClicks()

        # TODO: initialize video recorder        

        self.log_text.append("Initialization Ok")
        #self.log_text.append()
        self.ci = CalibIMUs.CalibIMUs()
        self.iCalibStage = 0

        self.show()
    

    def clearLog(self):
        # command = self.log_text.toPlainText()
        self.log_text.clear()
        # self.log_text.setText(str(eval(command)))

    def configureClicks(self):
        self.clear_log.clicked.connect(self.clearLog)
        self.play.clicked.connect(self.vtkCall)
        self.stop.clicked.connect(self.vtkEndCall)
        self.tuning_n.clicked.connect(self.n_qtr_shift)
        self.tuning_z.clicked.connect(self.z_qtr_shift)
        self.tuning_t.clicked.connect(self.t_qtr_shift)
        
        # self.new_ren.connect(self.vtkEndCall)

        # прописывать обработку всех кликов по кнопкам

    def n_qtr_shift(self): #нужно вставаить соответствие между масивами актеров и потоками
        
        # print(len(self.qtrs))
        # for el in self.WorkerAdress:
        #     self.log_text.append("N - pose initialized with quaternions " + str(el['port']) + " = " + str(self.qtrs[el['port']]) )   # + " " + str(self.qtrs[1]) + " " + str(self.qtrs[2])                

        self.rot0 = self.qtrs[5563]
        self.rot1 = self.qtrs[5561]
        self.rot2 = self.qtrs[5557]

        temp_1 = self.qtrs[5561]
        temp_2 = self.qtrs[5563]

        # print(temp_2)
        self.log_text.append( ' qtr5561 = ' + str(self.RqtrTovtkqtr(temp_1.as_quat())) )
        self.log_text.append( ' qtr5563 = ' + str(self.RqtrTovtkqtr(temp_2.as_quat())) )

        self.rotBox0AtIMU0 = self.rot0.inv()
        self.rotBox1AtIMU1 = self.rot1.inv()
        self.rotBox1AtIMU2 = self.rot2.inv()

        self.iCalibStage = 1
        print("\n\n\n====== STEP 1 ========\n\n\n")
                
        # rot0 5563 - ref, rot1 5561 - arm
        self.ci.setN_calibImusAtGlob(self.rot0, self.rot1)

        # add arm as a segment 
        self.ci.addSegmentImu(R.from_matrix(np.identity(3)), self.rot1)
        self.ci.addSegmentImu(R.from_matrix(np.identity(3)), self.rot2)

        

    def t_qtr_shift(self):
        
        # self.log_text.append("T - pose initialized with quaternions = " + str(self.qtrs[5557]) + " " + str(self.qtrs[5561]) + " " + str(self.qtrs[5563]))
        # self.T_arm_pos = np.array([-0.7,-0.7,0.,0.]) #([-0.7,-0.7,0.,0.]) # [-0.7,-0.7,0.,0.]
        # self.N_arm_pos = np.array([0.95,0.,0.25,0.])

        temp_1 = self.qtrs[5561]
        temp_2 = self.qtrs[5563]

        self.log_text.append( ' qtr5561 = ' + str(self.RqtrTovtkqtr(temp_1.as_quat())) )
        self.log_text.append( ' qtr5563 = ' + str(self.RqtrTovtkqtr(temp_2.as_quat())) )

        self.rot1 = self.qtrs[5561]

        self.iCalibStage = 2
        print("\n\n\n====== STEP 2 ========\n\n\n")
        self.ci.doCalibration( self.rot1, 1,2)
        self.flag_norm = 1
        

    def qtr_to_mtx(self, qtr):
        end = np.identity(3)
        temp = vtk.vtkQuaterniond(qtr)        
        temp.ToMatrix3x3(end)
        return end

    def mtx_to_qtr(self,mtx):        
        end = vtk.vtkQuaterniond(np.array([1.,0.,0.,0.]))
        print(mtx.as_matrix())
        end.FromMatrix3x3(mtx)
        return end

    def z_qtr_shift(self):
        # self.log_text.append("Z - pose initialized with quaternions = " + str(self.qtrs[5557])  + " " + str(self.qtrs[5561]) + " " + str(self.qtrs[5563]) )      
        # temp_qtr = self.qtrs[5561]

        # self.scene.addVector( self.RqtrTovtkqtr(temp_qtr.as_quat()))
        # self.iren.Render()
        # self.log_text.append(str(self.RqtrTovtkqtr(temp_qtr.as_quat())))

        # print(self.shift_calculus(4,5))

        # self.const_1 = [ ]
        # self.const_2 = [ ]
        # n = 3 
        
        # for i in range(0, n): 
        #     ele = [float(input()), float(input()), float(input())] 
        #     self.const_2.append(ele)
        
        # self.norm_body = R.from_matrix(self.const_2)

        # self.log_text.append(str(self.const_2))

        # for i in range(0, n): 
        #     ele = [float(input()), float(input()), float(input())] 
        #     self.const_1.append(ele)
        
        if (len(self.qtrs['ports']) < self.NUM_THREADS):
            self.log_text.append('Waiting for ' + str(set(self.rigth_list) -  set(self.qtrs['ports'])))
            return

        self.flag_norm = 1
        for i,el in enumerate(self.WorkerAdress):
            self.start_worker(i, el['port'], el['ip'])

        self.start_video()

    def vtkCall(self):      
        self.play.setDisabled(True)
        self.stop.setEnabled(True)
        self.timer.start(self.timeStep)
        # TODO: start video recording

    def vtkEndCall(self):
        self.stop.setDisabled(True)
        self.play.setEnabled(True)
        self.timer.stop()
        # TODO: stop video recording

    def KeyPress(self,obj, event):
        import re
        key = obj.GetKeySym() #works fine
        # print(key)
        global k
        
        if ( re.match(r'\d', str(key) ) ):
            k = int(key) + 3
            self.log_text.append(self.obj_list[k])
            if (key == "Left"):
                self.scene.init_pos_actor (k,np.array([0.000,0.000,-0.001]))
                self.iren.Render()
            if (key == "Right"):
                self.scene.init_pos_actor (k,np.array([0.000,0.000,0.001]))
                self.iren.Render()
            if (key == "Up"):
                self.scene.init_pos_actor (k,np.array([0.001,0.000,0.000]))
                self.iren.Render()
            if (key == "Down"):
                self.scene.init_pos_actor (k,np.array([-0.001,0.000,0.000]))
                self.iren.Render()
            if (key == "a"):
                self.scene.init_pos_actor (k,np.array([0.000,0.001,0.000]))
                self.iren.Render()
            if (key == "d"):
                self.scene.init_pos_actor (k,np.array([0.000,-0.001,0.000]))
                self.iren.Render()
            if (key == "n"):
                    
                while (NextSet(self, 4)):
                    return 0
        if (key == "b"):
            self.z_qtr_shift()
        
        if (key == "Escape"):
            self.abort_workers()

        return 0
        
    def createCentralWidget(self):

        self.frame = QFrame()
        self.vl = QHBoxLayout()
        
        self.vtkWidget = QVTKRenderWindowInteractor(self.frame)
        
        self.visualWidget = QWidget(self)
        self.play = QPushButton("play", self.visualWidget)
        self.stop = QPushButton("stop", self.visualWidget)
        self.pause = QPushButton("pause", self.visualWidget)
        self.tuning_n = QPushButton("N - pose", self.visualWidget)
        self.tuning_t = QPushButton("T - pose", self.visualWidget)
        
        self.tuning_z = QPushButton("Recording", self.visualWidget)

        self.vl.addWidget(self.vtkWidget)
       
        buttons_layout = QVBoxLayout()
        buttons_layout.addStretch(1)
        buttons_layout.addWidget(self.play)
        buttons_layout.addWidget(self.stop)
        buttons_layout.addWidget(self.pause)
        buttons_layout.addStretch(1)
        buttons_layout.addWidget(self.tuning_n)
        buttons_layout.addWidget(self.tuning_t)
        buttons_layout.addStretch(1)        
        buttons_layout.addWidget(self.tuning_z)
        buttons_layout.addStretch(1)

        #Create
        self.scene = Draw.vtpDrawScene()
        directory = 'geometry/'
        obj = get_files(directory)
        rigth_list = ['geometry/hat_jaw.vtp', 'geometry/hat_skull.vtp', 'geometry/hat_spine.vtp', 'geometry/humerus.vtp', 'geometry/humerus_l.vtp', 'geometry/radius_lv.vtp', 'geometry/radius_rv.vtp', 'geometry/scapula.vtp', 'geometry/scapula_l.vtp', 'geometry/thorax.vtp', 'geometry/ulna_lv.vtp', 'geometry/ulna_rv.vtp']
        rigth_list.append(list(set(obj) ^ set(rigth_list)))
        self.obj_list = rigth_list
        self.ren = self.scene.initScene_qt(obj)
        self.initial_qtr_norm()
        

        #Settings
        self.ren.SetBackground(0.2, 0.2, 0.2)
        self.timeStep = 20 #ms
        self.total_t = 0
        
        #check for phones
        self.start_threads()
        #start video recording in threads

        #self.start_video()        

        # self.controller = threads_qt.Controller()
        # self.controller.app = QCoreApplication([])       #я вообще не понимаю зачем это тут

        self.renWin = self.vtkWidget.GetRenderWindow()
        self.renWin.AddRenderer(self.ren)  #02.04.2020        

        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()
        self.iren.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        self.iren.AddObserver("KeyPressEvent", self.KeyPress)
        
        self.visualWidget.setLayout(self.vl)
        self.visualWidget.layout().addLayout(buttons_layout)

        self.frame.setLayout(self.vl)
        self.setCentralWidget(self.frame)
        # self.show()
        self.iren.Initialize()

        # Create Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.timerCallback)
        # self.timer.start(self.timeStep)
    
    def timerCallback(self):
        #сюда преобразование координат qweqrty
        
        try:
            if (len(self.qtrs) > 0 ) and (self.flag_norm == 1):
                for el in range(len(self.scene.modelActor)):
                    # temp_qtr = QC.qtr_multiplication(self.X_qtr,self.qtrs[2])
                    # actor_qtr = np.array([1.,0.,0.,0.])
                    temp_shift = np.array([0.,0.,0.])
                    self.motion_flag = np.array([0,[1.,0.,0.,0.]])                   
                    temp_qtr =  self.ci.calcBodyAtGlob(self.rotBox0AtIMU0*self.qtrs[5563]) #* self.rotBox0AtIMU0                смена x и y- self.rotBnoAtVtk_norm                            калибровка self.rotBox0AtIMU0 * 
                    
                    if (el == 4) or (el == 13):
                        # temp_qtr =  self.ci.calcBodyAtGlob(self.rotBox0AtIMU0*self.qtrs[5563])                       
                        temp_qtr_2 = self.ci.calcSegmentAtBody(0,self.rotBox0AtIMU1*self.qtrs[5561],temp_qtr)
                        self.motion_flag =  [1, self.RqtrTovtkqtr(temp_qtr_2.as_quat())]

                    if (el == 5) or (el == 10):
                        # temp_qtr =  self.ci.calcBodyAtGlob(self.rotBox0AtIMU0*self.qtrs[5563])
                        #print(temp_shift)
                        temp_qtr_2 = self.ci.calcSegmentAtBody(1,self.rotBox1AtIMU2*self.qtrs[5557],temp_qtr)

                        temp_shift = self.shift_calculus(4,el,self.ci.calcSegmentAtBody(0,self.qtrs[5561],temp_qtr).as_matrix())
                        self.motion_flag =  [1, self.RqtrTovtkqtr(temp_qtr_2.as_quat())]
                    
                    # if (el == 4):
                    #     temp_qtr = self.qtrs[5561] * self.rotBox0AtIMU1

                        # actor_qtr = QC.qtr_multiplication(self.Y_qtr,self.qtrs[1])
                        # temp_shift = self.shift_calculus(4,el)
                        # self.motion_flag = np.array([1, QC.qtr_multiplication(QC.qtr_multiplication(self.Y_qtr,self.qtrs[5561]),np.conj(QC.qtr_multiplication(self.X_qtr,self.qtrs[5563])))])  #QC.qtr_multiplication(self.qtrs[1],np.conj(QC.qtr_multiplication(self.X_qtr,self.qtrs[2])))  
                    # print(self.rot0)
                    # print(self.rotBox0AtIMU0)

                    # QC.qtr_multiplication(actor_qtr,np.conj(QC.qtr_multiplication(self.X_qtr,self.qtrs[2])))
                    print(temp_qtr)
                    
                    # self.scene.SetRefQuatOrientation(self.RqtrTovtkqtr(temp_qtr.as_quat()), temp_shift, el, self.motion_flag )

                self.iren.Render() #NOT: self.ren.Render()
            else:
                # print("No data")
                pass
        except:
            print("No worker ", el) 
    
    def shift_calculus(self,i_from_actor,i_to_actor, matrix):
        # print(np.array(self.scene.modelActor[i_from_actor].GetPosition())) np.array(self.scene.modelActor[i_to_actor].GetPosition())  np.array(temp_2).dot(np.array(self.scene.initial_pos_actors[i_to_actor]) - np.array(self.scene.initial_pos_actors[i_from_actor])) 
        return  np.array(self.scene.modelActor[i_from_actor].GetPosition()) - np.array(self.scene.modelActor[i_to_actor].GetPosition()) + np.array(matrix).dot(np.array(self.scene.initial_pos_actors[i_to_actor]) - np.array(self.scene.initial_pos_actors[i_from_actor]))
    
    def RqtrTovtkqtr(self, qtr):
        qtr = np.array([qtr[3],qtr[0],qtr[1],qtr[2]])
        return qtr

    def initial_qtr_norm(self):
        self.n_pos_temp_qtr = np.array([0.95, 0., 0.25, 0.])
        self.t_pos_temp_qtr = np.array([0.5, 0.5, 0.5, 0.5])
        self.z_pos_temp_qtr = np.array([0.684329, 0., 0., 0.713657])
        self.X_qtr = np.array([1, 0., 0., 0.])
        self.Y_qtr = np.array([1, 0., 0., 0.])
        self.Z_qtr = np.array([1, 0., 0., 0.])
        self.ZYX_qtr = np.array([1, 0., 0., 0.])
        self.N_arm_imu = np.array([1, 0., 0., 0.])
        self.check = np.array([1.,0.,0.,0.])
        self.eps = 0.01
        self.flag_norm = 0
        self.motion_flag = list()
        

    def start_video(self):
        self.frame_name = 'frame_'
        self.file_name = 'output_'
        self.camera_paths = ['rtsp://192.168.1.2:8553/PSIA/Streaming/channels/1?videoCodecType=MPEG4','rtsp://192.168.1.168:8553/PSIA/Streaming/channels/1?videoCodecType=MPEG4']
        self.NUM_CAMERAS = 2
        for idx in range(self.NUM_CAMERAS):
            thread = QThread()
            thread.setObjectName('thread_' + str(-idx))
            cam = Camera_2.VideoRecorder(self.camera_paths[idx], self.frame_name + str(idx), str(time.strftime("%d_%m_%Y_%H_%M_%S_",time.gmtime(time.time()))) + str(idx) + '.avi', App)
            self.__threads.append((thread, cam))
            cam.moveToThread(thread)
            thread.sig_abort_cams.connect(cam.abort)
            thread.started.connect(temp.cam)
            thread.start()

    def start_threads(self):

        self.__workers_done = 0
        self.__threads = []
        self.__threadsstatus = []
        self.stat = []
        self.qtrs = {}
        self.qtrs['ports'] = []
        self.qtrs_pure = {}
        self.shifts = []
        self.a = [0,1,2,3]
        self.video_flag_on = 0
   
        broadcaster = network.Broadcaster()
        thread = QThread()
        thread.setObjectName('thread_' + str(0))
        self.__threads.append((thread, broadcaster))  # need to store worker too otherwise will be gc'd
        #self.__threadsstatus.append(self.network_stat) #нужно продумать как именно добавлять статусы многопоточности в виджите
        broadcaster.moveToThread(thread)

        broadcaster.sig_status.connect(self.on_broadcaster_status)
        broadcaster.sig_msg.connect(self.log_text.append)
        broadcaster.sig_ports.connect(self.on_worker_port)

        # control worker:
        self.sig_abort.connect(broadcaster.abort)

        # get read to start worker:
        # self.sig_start.connect(worker.work)  # needed due to PyCharm debugger bug (!); comment out next line
        thread.started.connect(broadcaster.work) #(self.port)
        thread.start()
        self.log_text.append("Broadcaster started")

    @pyqtSlot(int, int, str)
    def on_broadcaster_status(self, flag: int, port: int, ip: str):

        if flag == 1:
            if self.num_workers < self.NUM_THREADS:
                d = { "port" : port, 'ip': ip }
                self.WorkerAdress.append(d)
                # self.start_worker(self.num_workers, port, ip)
                self.num_workers += 1
            else:
                pass
        else:
            print("flag : ", flag)

    def start_worker(self, idx, port, ip):
            
        worker = Worker.Worker(idx, port, ip, App) 
        thread = QThread()
        thread.setObjectName('thread_' + str(idx + 1))
        self.__threads.append((thread, worker))  # need to store worker too otherwise will be gc'd
        self.__threadsstatus.append(self.sensor_im_1) #нужно продумать как именно добавлять статусы многопоточности в виджите
        self.stat.append(-1)
        self.qtrs[port] = [1.,0.,0.,0.]
        self.shifts.append([0.,0.,0.])
        worker.moveToThread(thread)
        
        worker.sig_shifts.connect(self.on_worker_shifts)
        worker.sig_qtr.connect(self.on_worker_qtr)
        worker.sig_status.connect(self.on_worker_status)
        worker.sig_msg.connect(self.log_text.append)
        self.sig_abort_workers.connect(worker.abort)

        # get read to start worker:
        # self.sig_start.connect(worker.work)  # needed due to PyCharm debugger bug (!); comment out next line
        thread.started.connect(worker.work) #(self.port)
        thread.start()
        self.log_text.append('Worker #' + str(idx) + ' started.') 

    @pyqtSlot(int, list)
    def on_worker_shifts(self, worker_id: int, data: list):
        # self.log_text.append('Worker #{}: {}'.format(worker_id, data))
        # self.progress.append('{}: {}'.format(worker_id, data))
        # print(data)
        try: 
            self.shifts[worker_id] = data
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            traceback.print_exc()
    
    @pyqtSlot(int, list)
    def on_worker_qtr(self, worker_id: int, data: list):
        # self.log.append('Worker #{}: {}'.format(worker_id, data))
        # self.progress.append('{}: {}'.format(worker_id, data))
        #print(data)
        try: 
            self.qtrs_pure[worker_id] = data
            # print(data)
            self.qtrs[worker_id] = self.rotBnoAtVtk.inv() * R.from_quat(np.array([data[1],data[2],data[3],data[0]])) * self.rotBnoAtVtk #cicle(self,data) data # self.rotBnoAtVtk.inv()

        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            traceback.print_exc()

    @pyqtSlot(int)
    def on_worker_port(self, port: int):
        # self.log.append('Worker #{}: {}'.format(worker_id, data))
        # self.progress.append('{}: {}'.format(worker_id, data))
        # print(port)
        try:  
            if self.qtrs['ports'].count(port) == 0:
                self.qtrs['ports'].append(port)

        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            traceback.print_exc()

    @pyqtSlot(int, int)
    def on_worker_status(self, worker_id: int, flag: int):
        #self.log_text.append('worker #{} done'.format(worker_id))
        # self.progress.append('-- Worker {} DONE'.format(worker_id))
        if (self.stat[worker_id] == flag):
            pass
        elif (flag == 1):
            self.stat[worker_id] = flag
            # self.__threadsstatus[worker_id].setText("Active")
            self.sensor_im_1.setText('Active')
            self.log_text.append('Sensor ' + str(worker_id) + ' is Active')
        else:
            self.stat[worker_id] = flag
            self.sensor_im_1.setText('Waiting')
            self.log_text.append('Connect sensor # ' + str(worker_id) + '. Waiting...')
            #self.qtrs[worker_id] = np.array([0.95,0.,0.25,0.]) #

        self.__workers_done += 1
        # if self.__workers_done == self.NUM_THREADS:
            # self.log.append('No more workers active')
            # self.__threads = None

    @pyqtSlot()
    def abort_workers(self):
        self.log_text.append('Asking each sensor to abort')
        self.sig_abort_workers.emit()
        # for thread, worker in self.__threads:  # note nice unpacking by Python, avoids indexing
        #     thread.quit()  # this will quit **as soon as thread event loop unblocks**            
        #     thread.wait()  # <- so you need to wait for it to *actually* quit
        #     print('unlocked')
    
    @pyqtSlot()   
    def abort(self):
        self.sig_abort.emit()
        # self.log.append('Asking each worker to abort')
        for thread, task in self.__threads:  # note nice unpacking by Python, avoids indexing
            thread.quit()  # this will quit **as soon as thread event loop unblocks**
            thread.wait()  # <- so you need to wait for it to *actually* quit

        # even though threads have exited, there may still be messages on the main thread's
        # queue (messages that threads emitted before the abort):
        # self.log.append('All threads exited')

    def exit(self):
        App.quit()

    def restoreWindows(self):
        self.explorer.close()
        self.sensors.close()
        self.log.close()
        # сохранить текст из лога и записать его обратно
        self.createExplorerDock()
        self.createSensorsDock()
        self.createLogDockWidget()

    def menuActionConnect(self, name, func):
        act = QAction(name, self)
        # можно добавить иконки QIcon..
        act.triggered.connect(func)
        return act

    def createMenu(self):
        menubar = self.menuBar()
        file = menubar.addMenu("Файл")
        file.addAction("Создать новый эксперимент")
        file.addAction("Загрузить существующий эксперимент")
        file.addAction(self.menuActionConnect("&Выйти из приложения", exit))

        window = menubar.addMenu("Окна")
        
        window.addAction(self.menuActionConnect("Восстановить окна по умолчанию", self.restoreWindows))

    def createSensorsDock(self):
        self.sensors = QDockWidget("Состояние датчиков", self)
        self.sensors.setAllowedAreas(Qt.BottomDockWidgetArea | Qt.TopDockWidgetArea)

        sensorsWidget = QWidget(self)

        self.sensor_im_1 = QLabel(sensorsWidget)
        self.sensor_im_1.setText("Image_1")
        self.sensor_im_2 = QLabel(sensorsWidget)
        self.sensor_im_2.setText("Image_2")
        self.sensor_im_3 = QLabel(sensorsWidget)
        self.sensor_im_3.setText("Image_3")

        sensorsHBox = QHBoxLayout()
        sensorsHBox.addWidget(self.sensor_im_1)
        sensorsHBox.addWidget(self.sensor_im_2)
        sensorsHBox.addWidget(self.sensor_im_3)

        namesHBox = QHBoxLayout()
        namesHBox.addWidget(QLabel("phone_1", sensorsWidget))
        namesHBox.addWidget(QLabel("phone_2", sensorsWidget))
        namesHBox.addWidget(QLabel("phone_3", sensorsWidget))

        vBox = QVBoxLayout()
        vBox.addLayout(sensorsHBox)
        vBox.addLayout(namesHBox)

        sensorsWidget.setLayout(vBox)
        
        self.sensors.setWidget(sensorsWidget)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.sensors)

    def setDirectoryToExplorer(self, directory):
        model = QFileSystemModel()
        model.setRootPath(directory)
        model.setNameFilterDisables(False)
        
        self.tree = QTreeView()
        self.tree.setModel(model)
        self.tree.setRootIndex(model.index(directory))
        self.tree.hideColumn(1)
        self.tree.hideColumn(2)
        self.tree.hideColumn(3)

    def createExplorerDock(self):
        self.explorer = QDockWidget("Файлы эксперимента", self)
        # self.explorer.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)

        sensorsWidget = QWidget(self)
        sensorsWidget.setMinimumWidth(self.width - (self.width / 1.1))

        self.setDirectoryToExplorer(QDir.currentPath())
        vBox = QVBoxLayout()
        vBox.addWidget(self.tree)

        sensorsWidget.setLayout(vBox)
        
        self.explorer.setWidget(sensorsWidget)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.explorer)


    def createToolBar(self):
        self.tools = QToolBar("Инструменты", self)
        self.tools.addSeparator()
        self.tools.setMovable(False)
        self.tools.setAllowedAreas(Qt.TopToolBarArea)
        self.tools.addAction("Добавить человека")
        self.tools.addAction("Заполнить параметры")
        self.addToolBar(Qt.TopToolBarArea, self.tools)


    def createLogDockWidget(self):
        self.log = QDockWidget("Лог", self)
        # self.log.setFeatures(QDockWidget.NoDockWidgetFeatures)
        # self.log.setAllowedAreas(Qt.NoDockWidgetArea)
        self.log.setAllowedAreas(Qt.BottomDockWidgetArea | Qt.TopDockWidgetArea)

        logWidget = QWidget(self)
        logWidget.setMinimumWidth(self.width / 1.1)
        
        self.log_text = QTextEdit(logWidget)
        # self.log_text.setEnabled(False)
        self.clear_log = QPushButton("Очистить лог", logWidget)
        logHBox = QHBoxLayout()
        logHBox.addStretch(1)
        logHBox.addWidget(self.clear_log)

        logVBox = QVBoxLayout()
        logVBox.addWidget(self.log_text)
        logVBox.addLayout(logHBox)
        logWidget.setLayout(logVBox)

        self.log.setWidget(logWidget)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.log)

        self.ip = '0'

        self.log_timer = QTimer()
        self.log_timer.timeout.connect(self.IP_Callback)
        self.log_timer.start(1000)
    
    def IP_Callback(self):
        host_name = socket.gethostname()
        host_ip = socket.gethostbyname(host_name)     
        if (self.ip != host_ip):
            self.ip = host_ip
            self.log_text.append('To connect IP : ' + str(host_ip) )
            self.log_text.append('Start from port: 5555')

global App
if __name__ == "__main__":
    # install exception hook: without this, uncaught exception would cause application to exit
    sys.excepthook = trap_exc_during_debug    
    App = QApplication(sys.argv)
    window = MainWindow()
    sys.exit(App.exec())
