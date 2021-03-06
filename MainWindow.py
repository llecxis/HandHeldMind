import sys
from PyQt5 import QtGui
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import vtk
import math
import numpy as np
import time
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import socket
import traceback

import Worker
import Draw




def trap_exc_during_debug(*args):
    # when app raises uncaught exception, print info
    print(args)

# install exception hook: without this, uncaught exception would cause application to exit
sys.excepthook = trap_exc_during_debug

def get_files(directory):
    import os
    files = os.listdir(directory)
    for i in range(0,len(files)):
        files[i] = directory + files[i]
    return files

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

def integ(x, tck, constant = 10e-9):
    from scipy import interpolate
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

    NUM_THREADS = 3 # number of phones
    sig_abort_workers = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.title = "Система сбора, обработки и визуализации деятельности работников предприятия"
        self.top = 200
        self.left = 500
        self.width = 1000
        self.height = 800
        
        self.setWindowIcon(QtGui.QIcon("icon.png"))
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        
        self.createMenu()
        self.createSensorsDock()  

        self.createCentralWidget()
        self.createToolBar()        

        self.createExplorerDock() 
        
        self.createLogDockWidget()

        self.configureClicks()
        

        self.log_text.append("Initialization Ok")
        #self.log_text.append()

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
        # temp_mat = np.array([[0,0,0],[0,0,0],[0,0,0]])
        # sol_mat = np.array([[0,0,0],[0,0,0],[0,0,0]])
        self.qtrs[2] = np.array([-1.0, -0.004, -0.017, 0.008])
        self.N_arm_pos = np.array([0.95,0.,0.25,0.]) 
        self.N_ref_imu = self.qtrs[2]
        self.N_arm_imu = self.qtrs[1]
        self.log_text.append("N - pose initialized with quaternions = " + str(self.qtrs[0]) + " " + str(self.qtrs[1]) + " " + str(self.qtrs[2]))
        self.X_qtr = self.three_qtr_solve(self.N_arm_pos, self.N_ref_imu, self.N_arm_imu)
        self.YX_qtr = self.qtr_multiplication(self.Y_qtr,self.X_qtr)
        self.log_text.append('Initial = ' + str(self.three_qtr_multiplication(self.X_qtr,self.N_ref_imu,self.N_arm_imu)))
        #self.log_text.append(str(self.X_qtr))

        # self.n_pos_temp_mat = self.scene.mtxRot[4]
        # self.n_pos_temp_qtr = self.qtr_un_calculus(self.qtrs[0],[0.95,0.,0.25,0.])
        # self.log_text.append('[1,0.,0.,0.] -> ' + str(self.n_pos_temp_qtr))


        #
    def t_qtr_shift(self):
        self.log_text.append("T - pose initialized with quaternions = " + str(self.qtrs[0]) + " " + str(self.qtrs[1]) + " " + str(self.qtrs[2]))
        self.qtrs[2] = np.array([-1.0, -0.004, -0.017, 0.008])
        self.T_arm_pos = np.array([0.5,0.5,0.5,0.5]) # [-0.7,-0.7,0.,0.]
        self.N_ref_imu = self.qtrs[2]
        #self.N_arm_imu = self.qtrs[1] #test variant
        #self.N_arm_imu is taken from the previous to save N pose
        self.N_arm_in_t_pos = self.three_qtr_multiplication(self.X_qtr, self.N_ref_imu, self.N_arm_imu)
        #self.T_arm_imu = self.qtr_un_calculus(self.N_arm_imu,self.qtrs[1])
        self.T_arm_imu = self.qtrs[1]
        self.Y_qtr = self.three_qtr_solve(self.T_arm_pos, self.N_arm_pos, self.T_arm_imu)
        self.log_text.append('Initial = ' + str(self.three_qtr_multiplication(self.Y_qtr,self.N_arm_pos,self.T_arm_imu)))
        #self.log_text.append(str(self.Y_qtr))
        
        self.YX_qtr = self.qtr_multiplication(self.X_qtr,self.Y_qtr)
        #self.log_text.append(str(self.YX_qtr))


    def z_qtr_shift(self):
        self.log_text.append("Z - pose initialized with quaternions = " + str(self.qtrs[0]) + " " + str(self.qtrs[1]))
        self.z_pos_temp_mat = self.scene.mtxRot[4]
        self.z_pos_temp_qtr = self.qtrs[1]
        self.log_text.append(str(self.z_pos_temp_mat))

    def vtkCall(self):      
        self.play.setDisabled(True)
        self.stop.setEnabled(True)
        self.timer.start(self.timeStep)

    def vtkEndCall(self):
        self.stop.setDisabled(True)
        self.play.setEnabled(True)
        self.timer.stop()

    def KeyPress(self,obj, event):
        import re
        key = obj.GetKeySym() #works fine
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
        self.tuning_z = QPushButton("Z - pose", self.visualWidget)
        self.tuning_t = QPushButton("T - pose", self.visualWidget)

        self.vl.addWidget(self.vtkWidget)
       
        buttons_layout = QVBoxLayout()
        buttons_layout.addStretch(1)
        buttons_layout.addWidget(self.play)
        buttons_layout.addWidget(self.stop)
        buttons_layout.addWidget(self.pause)
        buttons_layout.addStretch(1)
        buttons_layout.addWidget(self.tuning_n)
        buttons_layout.addWidget(self.tuning_z)
        buttons_layout.addWidget(self.tuning_t)
        buttons_layout.addStretch(1)

        #Create
        self.scene = Draw.vtpDrawScene()
        directory = 'geometry/'
        obj = get_files(directory)
        self.obj_list = obj
        self.ren = self.scene.initScene_qt(obj)
        self.initial_qtr_norm()

        #Settings
        self.ren.SetBackground(0.2, 0.2, 0.2)
        self.timeStep = 20 #ms
        self.total_t = 0

        #check for phones
        self.start_threads()

        # self.controller = threads_qt.Controller()
        # self.controller.app = QCoreApplication([])       #я вообще не понимаю зачем это тут

        self.renWin = self.vtkWidget.GetRenderWindow()
        self.renWin.AddRenderer(self.ren)
        

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
        
        i_actor = 4
        #self.temp_qtr = self.qtrs[1]
        self.temp_qtr = self.three_qtr_multiplication(self.YX_qtr,self.qtrs[2],self.qtrs[1])
        #self.temp_qtr = self.qtr_multiplication(self.temp_qtr, self.qtrs[1])
        self.scene.SetQuatOrientation(self.temp_qtr, self.shifts[1], i_actor)

        i_actor = 5
        #self.actors_shift = np.array(self.scene.initial_pos_actors[4]) + np.array(self.scene.mtxRot).dot(np.array(self.scene.initial_pos_actors[5]) - np.array(self.scene.initial_pos_actors[4]))
        self.shifts[0] = self.shift_calculus(4,i_actor)
        #self.temp_qtr = self.qtr_calculus(4,i_actor)
        self.temp_qtr = self.qtr_multiplication(self.qtrs[0], self.temp_qtr) # self.n_pos_temp_qtr
        # self.temp_qtr = self.qtr_multiplication(self.qtrs[0], np.array([0.,0.,1.,0.])) 
        # print(self.actors_shift,self.scene.initial_pos_actors[4])
        self.scene.SetQuatOrientation(self.temp_qtr, self.shifts[0], i_actor)

        i_actor = 10
        self.shifts[0] = self.shift_calculus(4,i_actor)
        #self.temp_qtr = self.qtr_calculus(4,i_actor)
        self.temp_qtr = self.qtr_multiplication(self.qtrs[0], self.temp_qtr) # self.n_pos_temp_qtr
        # self.temp_qtr = self.qtr_multiplication(self.qtrs[0], np.array([0.,0.,1.,0.]))
        self.scene.SetQuatOrientation(self.temp_qtr, self.shifts[0], i_actor)

        i_actor = 2
        self.shifts[2] = self.shift_calculus(2,i_actor)
        #self.temp_qtr = self.qtrs[2] #self.qtr_multiplication(self.qtrs[2], np.array([1.,0.,0.,0.]))
        self.temp_qtr = self.qtr_multiplication(self.qtrs[2], np.array([1.,0.,0.,0.]))
        #self.log_text.append(str(self.temp_qtr))
        self.scene.SetQuatOrientation(self.temp_qtr, self.shifts[2], i_actor)

        self.iren.Render() #NOT: self.ren.Render()
    
    def shift_calculus(self,i_from_actor,i_to_actor):
        return np.array(self.scene.initial_pos_actors[i_from_actor]) + np.array(self.scene.mtxRot[i_from_actor]).dot(np.array(self.scene.initial_pos_actors[i_to_actor]) - np.array(self.scene.initial_pos_actors[i_from_actor]))- np.array(self.scene.initial_pos_actors[i_to_actor])
    
    def qtr_calculus(self,i_from_actor,i_to_actor): #в последствии будте таблица соответствия актеров и потоков, пока остается константами        
        # print(vtk.vtkQuaterniond(self.qtrs[1]).ToMatrix3x3([[0,0,0],[0,0,0],[0,0,0]]))

        i_from_actor = 1 #в последствии будте таблица соответствия актеров и потоков, пока остается константами      
        i_to_actor = 0
        
        mod_qtr = self.qtrs[1][0]*self.qtrs[1][0] + self.qtrs[1][1]*self.qtrs[1][1] + self.qtrs[1][2]*self.qtrs[1][2] + self.qtrs[1][3]*self.qtrs[1][3] # mod for multiplication of quaternions
        
        # self.qtrs[i_from_actor] - a - quater
        # self.qtrs[i_to_actor]   - b - quater

        a1 = self.qtrs[i_from_actor][0]*self.qtrs[i_to_actor][0] - self.qtrs[i_from_actor][1]*self.qtrs[i_to_actor][1] - self.qtrs[i_from_actor][2]*self.qtrs[i_to_actor][2] - self.qtrs[i_from_actor][3]*self.qtrs[i_to_actor][3]
        a2 = self.qtrs[i_from_actor][1]*self.qtrs[i_to_actor][0] + self.qtrs[i_from_actor][0]*self.qtrs[i_to_actor][1] - self.qtrs[i_from_actor][3]*self.qtrs[i_to_actor][2] + self.qtrs[i_from_actor][2]*self.qtrs[i_to_actor][3]
        a3 = self.qtrs[i_from_actor][2]*self.qtrs[i_to_actor][0] + self.qtrs[i_from_actor][3]*self.qtrs[i_to_actor][1] + self.qtrs[i_from_actor][0]*self.qtrs[i_to_actor][2] - self.qtrs[i_from_actor][1]*self.qtrs[i_to_actor][3]
        a4 = self.qtrs[i_from_actor][3]*self.qtrs[i_to_actor][0] - self.qtrs[i_from_actor][2]*self.qtrs[i_to_actor][1] + self.qtrs[i_from_actor][1]*self.qtrs[i_to_actor][2] + self.qtrs[i_from_actor][0]*self.qtrs[i_to_actor][3]
        
        qtr_mult = np.array([a1/mod_qtr,a2/mod_qtr,a3/mod_qtr,a4/mod_qtr])
        # print(qtr_multiplication)
        return np.array(qtr_mult) #vtk.vtkQuaterniond(np.array(self.qtrs[1])+np.array(qtr)).Normalized()
    
    def qtr_multiplication(self,qtr_a, qtr_b): #в последствии будте таблица соответствия актеров и потоков, пока остается константами        
        # print(vtk.vtkQuaterniond(self.qtrs[1]).ToMatrix3x3([[0,0,0],[0,0,0],[0,0,0]]))
        qtr_a = np.array(qtr_a)
        qtr_b = np.array(qtr_b)
        mod_qtr = qtr_a[0]*qtr_a[0] + qtr_a[1]*qtr_a[1] + qtr_a[2]*qtr_a[2] + qtr_a[3]*qtr_a[3] # mod for multiplication of quaternions
        
        # self.qtrs[i_from_actor] - a - quater
        # self.qtrs[i_to_actor]   - b - quater

        a1 = qtr_a[0]*qtr_b[0] - qtr_a[1]*qtr_b[1] - qtr_a[2]*qtr_b[2] - qtr_a[3]*qtr_b[3]
        a2 = qtr_a[1]*qtr_b[0] + qtr_a[0]*qtr_b[1] - qtr_a[3]*qtr_b[2] + qtr_a[2]*qtr_b[3]
        a3 = qtr_a[2]*qtr_b[0] + qtr_a[3]*qtr_b[1] + qtr_a[0]*qtr_b[2] - qtr_a[1]*qtr_b[3]
        a4 = qtr_a[3]*qtr_b[0] - qtr_a[2]*qtr_b[1] + qtr_a[1]*qtr_b[2] + qtr_a[0]*qtr_b[3]
        
        qtr_mult = np.array([a1/mod_qtr,a2/mod_qtr,a3/mod_qtr,a4/mod_qtr])
        # print(qtr_multiplication)
        return qtr_mult

    def qtr_un_calculus(self,qtr_a,qtr_c): #to find qtr_b in equation qtr_a**qtr_b = qtr_c (where ** - quaterninal multiplication)
        #в последствии будте таблица соответствия актеров и потоков, пока остается константами        
        # print(vtk.vtkQuaterniond(self.qtrs[1]).ToMatrix3x3([[0,0,0],[0,0,0],[0,0,0]]))
        qtr_a = np.array(qtr_a)
        qtr_c = np.array(qtr_c)
        mod_qtr = -1*(qtr_a[0]*qtr_a[0] + qtr_a[1]*qtr_a[1] + qtr_a[2]*qtr_a[2] + qtr_a[3]*qtr_a[3]) # mod for multiplication of quaternions
        
        # self.qtrs[i_from_actor] - a - quater
        # self.qtrs[i_to_actor]   - c - quater

        a1 = -1*qtr_a[0]*qtr_c[0] - qtr_a[1]*qtr_c[1] - qtr_a[2]*qtr_c[2] - qtr_a[3]*qtr_c[3]
        a2 = qtr_a[1]*qtr_c[0] - qtr_a[0]*qtr_c[1] - qtr_a[3]*qtr_c[2] + qtr_a[2]*qtr_c[3]
        a3 = qtr_a[2]*qtr_c[0] + qtr_a[3]*qtr_c[1] - qtr_a[0]*qtr_c[2] - qtr_a[1]*qtr_c[3]
        a4 = qtr_a[3]*qtr_c[0] - qtr_a[2]*qtr_c[1] + qtr_a[1]*qtr_c[2] - qtr_a[0]*qtr_c[3]

        qtr_mult = np.array([a1/mod_qtr,a2/mod_qtr,a3/mod_qtr,a4/mod_qtr])
        return qtr_mult #vtk.vtkQuaterniond(np.array(self.qtrs[1])+np.array(qtr)).Normalized()

    def three_qtr_solve(self,qtr_c,qtr_a,qtr_b): # C = X*A*B (looking for X)

        a1 = qtr_a[0]
        a2 = qtr_a[1]
        a3 = qtr_a[2]
        a4 = qtr_a[3]

        b1 = qtr_b[0]
        b2 = qtr_b[1]
        b3 = qtr_b[2]
        b4 = qtr_b[3]

        mod_ab = (a1*a1+a2*a2+a3*a3+a4*a4)*(b1*b1+b2*b2+b3*b3+b4*b4)

        c1 = qtr_c[0]
        c2 = qtr_c[1]
        c3 = qtr_c[2]
        c4 = qtr_c[3]

        x01 = (-a3*b3*c1-a4*b4*c1-a4*b3*c2+a3*b4*c2+a3*b1*c3+a4*b2*c3+a4*b1*c4-a3*b2*c4+a2*(-b2*c1+b1*c2-b4*c3+b3*c4)+a1*(b1*c1+b2*c2+b3*c3+b4*c4))/mod_ab
                
        x02 = (a4*b3*c1-a3*b4*c1-a3*b3*c2-a4*b4*c2-a4*b1*c3+a3*b2*c3+a3*b1*c4+a4*b2*c4+a1*(-b2*c1+b1*c2-b4*c3+b3*c4)-a2*(b1*c1+b2*c2+b3*c3+b4*c4))/mod_ab
                
        x03 = (-a1*b3*c1+a2*b4*c1+a2*b3*c2+a1*b4*c2+a1*b1*c3-a2*b2*c3-a2*b1*c4-a1*b2*c4+a4*(-b2*c1+b1*c2-b4*c3+b3*c4)-a3*(b1*c1+b2*c2+b3*c3+b4*c4))/mod_ab
        
        x04 = (-a2*b3*c1-a1*b4*c1-a1*b3*c2+a2*b4*c2+a2*b1*c3+a1*b2*c3+a1*b1*c4-a2*b2*c4+a3*(b2*c1-b1*c2+b4*c3-b3*c4)-a4*(b1*c1+b2*c2+b3*c3+b4*c4))/mod_ab

        qtr_x = np.array([x01,x02,x03,x04])

        return qtr_x

    def three_qtr_multiplication(self,qtr_a,qtr_b,qtr_c): # X = A*B*C (looking for X)

        a1 = qtr_a[0]
        a2 = qtr_a[1]
        a3 = qtr_a[2]
        a4 = qtr_a[3]

        b1 = qtr_b[0]
        b2 = qtr_b[1]
        b3 = qtr_b[2]
        b4 = qtr_b[3]

        c1 = qtr_c[0]
        c2 = qtr_c[1]
        c3 = qtr_c[2]
        c4 = qtr_c[3]

        x01 = -a4*(b4*c1-b3*c2+b2*c3+b1*c4)-a3*(b3*c1+b4*c2+b1*c3-b2*c4)-a2*(b2*c1+b1*c2-b4*c3+b3*c4)+a1*(b1*c1-b2*c2-b3*c3-b4*c4)

        x02 = a3*(b4*c1-b3*c2+b2*c3+b1*c4)-a4*(b3*c1+b4*c2+b1*c3-b2*c4)+a1*(b2*c1+b1*c2-b4*c3+b3*c4)+a2*(b1*c1-b2*c2-b3*c3-b4*c4)

        x03 = -a2*(b4*c1-b3*c2+b2*c3+b1*c4)+a1*(b3*c1+b4*c2+b1*c3-b2*c4)+a4*(b2*c1+b1*c2-b4*c3+b3*c4)+a3*(b1*c1-b2*c2-b3*c3-b4*c4)

        x04 = a1*(b4*c1-b3*c2+b2*c3+b1*c4)+a2*(b3*c1+b4*c2+b1*c3-b2*c4)-a3*(b2*c1+b1*c2-b4*c3+b3*c4)+a4*(b1*c1-b2*c2-b3*c3-b4*c4)

        # x01 = (a1*b1-a2*b2-a3*b3-a4*b4)*c1-(a2*b1+a1*b2-a4*b3+a3*b4)*c2-(a3*b1+a4*b2+a1*b3-a2*b4)*c3-(a4*b1-a3*b2+a2*b3+a1*b4)*c4
            
        # x02 = (a2*b1+a1*b2-a4*b3+a3*b4)*c1+(a1*b1-a2*b2-a3*b3-a4*b4)*c2-(a4*b1-a3*b2+a2*b3+a1*b4)*c3+(a3*b1+a4*b2+a1*b3-a2*b4)*c4
            
        # x03 = (a3*b1+a4*b2+a1*b3-a2*b4)*c1+(a4*b1-a3*b2+a2*b3+a1*b4)*c2+(a1*b1-a2*b2-a3*b3-a4*b4)*c3-(a2*b1+a1*b2-a4*b3+a3*b4)*c4
            
        # x04 = (a4*b1-a3*b2+a2*b3+a1*b4)*c1-(a3*b1+a4*b2+a1*b3-a2*b4)*c2+(a2*b1+a1*b2-a4*b3+a3*b4)*c3+(a1*b1-a2*b2-a3*b3-a4*b4)*c4

        qtr_x = np.array([x01,x02,x03,x04])

        return qtr_x
    
    def initial_qtr_norm(self):
        self.n_pos_temp_qtr = np.array([0.95, 0., 0.25, 0.])
        self.t_pos_temp_qtr = np.array([-0.7, -0.7, 0., 0.])
        self.z_pos_temp_qtr = np.array([0.684329, 0., 0., 0.713657])
        self.X_qtr = np.array([1, 0., 0., 0.])
        self.Y_qtr = np.array([1, 0., 0., 0.])
        self.YX_qtr = np.array([1, 0., 0., 0.])
        self.N_arm_imu = np.array([1, 0., 0., 0.])

    def start_threads(self):

        self.__workers_done = 0
        self.__threads = []
        self.__threadsstatus = []
        self.stat = []
        self.qtrs = []
        self.shifts = []
        self.a = [0,1,2,3]
        port = 5555
        for idx in range(self.NUM_THREADS):
            worker = Worker.Worker(idx, port)
            thread = QThread()
            thread.setObjectName('thread_' + str(idx))
            self.__threads.append((thread, worker))  # need to store worker too otherwise will be gc'd
            self.__threadsstatus.append(self.sensor_im_1) #нужно продумать как именно добавлять статусы многопоточности в виджите
            self.stat.append(-1)
            self.qtrs.append([1.,0.,0.,0.])
            self.shifts.append([0.,0.,0.])
            worker.moveToThread(thread)

            worker.sig_shifts.connect(self.on_worker_shifts)
            worker.sig_qtr.connect(self.on_worker_qtr)
            worker.sig_status.connect(self.on_worker_status)
            # worker.sig_msg.connect(self.log_text.append)

            # control worker:
            self.sig_abort_workers.connect(worker.abort)

            # get read to start worker:
            # self.sig_start.connect(worker.work)  # needed due to PyCharm debugger bug (!); comment out next line
            thread.started.connect(worker.work) #(self.port)
            port = port + 1
            thread.start()  # this will emit 'started' and start thread event loop
    
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
            self.qtrs[worker_id] = data #cicle(self,data)
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            traceback.print_exc()

    @pyqtSlot(int, int)
    def on_worker_status(self, worker_id: int, flag: int):
        # self.log.append('worker #{} done'.format(worker_id))
        # self.progress.append('-- Worker {} DONE'.format(worker_id))
        if (self.stat[worker_id] == flag):
            pass
        elif (flag == 1):
            self.stat[worker_id] = flag
            # self.__threadsstatus[worker_id].setText("Active")
            self.sensor_im_1.setText('Active')
            self.log_text.append('Phone ' + str(worker_id) + ' is Active')
        else:
            self.stat[worker_id] = flag
            self.sensor_im_1.setText('Waiting')
            self.log_text.append('Connect phone # ' + str(worker_id) + '. Waiting...')
            #self.qtrs[worker_id] = np.array([0.95,0.,0.25,0.]) #

        self.__workers_done += 1
        # if self.__workers_done == self.NUM_THREADS:
            # self.log.append('No more workers active')
            # self.__threads = None

    @pyqtSlot()
    def abort_workers(self):
        self.sig_abort_workers.emit()
        # self.log.append('Asking each worker to abort')
        for thread, worker in self.__threads:  # note nice unpacking by Python, avoids indexing
            thread.quit()  # this will quit **as soon as thread event loop unblocks**
            thread.wait()  # <- so you need to wait for it to *actually* quit

        # even though threads have exited, there may still be messages on the main thread's
        # queue (messages that threads emitted before the abort):
        # self.log.append('All threads exited')


    def exit():
        qApp.quit()

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

App = QApplication(sys.argv)
window = MainWindow()
sys.exit(App.exec())

