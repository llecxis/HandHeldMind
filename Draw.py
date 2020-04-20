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
import QtrCalc as QC
#from scipy.spatial.transform import Rotation as R

class vtpDrawScene: 
    def SetQuatOrientation_old( self, quaternion, shift, i_actor ): #function to rotate one element around his own origine point
        if not self.iniOk_qt :
            raise Exception("vtpDrawScene not initialized. Call initScene() first")

        # Convert quat to the rotation matrix
        # self.mtxRot =  [[0,0,0],[0,0,0],[0,0,0]]

        #quat = QC.qtr_multiplication(quaternion,self.norm_qtr[i_actor])
        # self.mtxRot[i_actor] = self.modelActor[i_actor].GetMatrix()

        vtk.vtkMath().QuaternionToMatrix3x3(quaternion, self.mtxRot[i_actor])

        # mtxTr01 = self.modelActor[i_actor].GetMatrix()
        # mtx = np.identity(3)

        # for i in range(2):
        #     for j in range(2) :
        #         mtx[i][j] = mtxTr01.GetElement(i, j)        
    
        # norm matrix
        # self.mtxRot[i_actor] = mtx.dot(np.array(self.mtxRot[i_actor])) #np.array(self.mtxRot[i_actor]).dot(np.array(self.norm_mat[i_actor]))
        # Rotation: convert 3x3 to 4x4 matrix
        # mtxTr2 = vtk.vtkMatrix4x4() # identity mtx


        mtxTr2 = vtk.vtkMatrix4x4() # identity mtx
        for i in range(3):
            for j in range(3):
                mtxTr2.SetElement(i, j, self.mtxRot[i_actor][i][j])
        # print(mtxTr02)
        
        # mtxTr01 = self.modelActor[i_actor].GetMatrix()
        # vtk.vtkMatrix4x4().Multiply4x4(mtxTr01, mtxTr02, mtxTr2)
    
        # three transforms:
        # 1. move the object so the rotation center is in the coord center  
        tr = vtk.vtkTransform()
        origin = np.array(self.modelActor[i_actor].GetOrigin())
        
        position = np.array(self.modelActor[i_actor].GetPosition())

        # trans = origin
        # trans = origin + position
        trans = position
        # trans = np.array([0.,0.,0.])       
        tr.Translate(-trans)
        mtxTr1 = tr.GetMatrix()

        # 2. rotate around coord center using mtxTr2
        mtxTr12 = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4().Multiply4x4(mtxTr2, mtxTr1, mtxTr12)
        
        ## 3. move the object back
        tr = vtk.vtkTransform()

        tr.Translate(trans + np.array(shift))
        mtxTr3 = tr.GetMatrix()
        mtxTr123 = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4().Multiply4x4(mtxTr3, mtxTr12, mtxTr123)
        
        tr = vtk.vtkTransform()
        tr.PreMultiply() 
        tr.Concatenate(mtxTr123) 
        self.modelActor[i_actor].SetUserTransform(tr)


    
    def initScene_qt(self, obj): #initialize alll actors
        reader = list()
        modelMapper = list()
        self.modelActor = list()
        self.mtxRot = list()
        self.norm_qtr = list()

        self.ren = vtk.vtkRenderer()

        self.mtxGlob = np.array([[1.,0.,0.],[0.,0.,-1.],[0.,1.,0.]])
        # self.mtxGlob[1,1] = 0
        # mtxGlob[2,1] = 1
        # mtxGlob[1,2] = -1
        # mtxGlob[2,2] = 0

        # renCornerR = vtk.vtkRenderer()
        # self.ren.AddRenderer(renCornerR)
        # renCornerR.SetViewport(0.8, 0, 1.0, 0.2)

        for i in range(0,len(obj)):

            # Read the object data
            filename = obj[i]
            reader.append(vtk.vtkXMLPolyDataReader())
            reader[i].SetFileName(filename)
            reader[i].Update()
            
            # make mapper for the data
            modelMapper.append(vtk.vtkPolyDataMapper())
            modelMapper[i].SetInputData(reader[i].GetOutput())
            
            # create actor, set its mapper
            self.modelActor.append(vtk.vtkActor())
            
            self.modelActor[i].SetMapper(modelMapper[i])
            
            # Add the actor to the renderer, set the background and size.
            self.ren.AddActor(self.modelActor[i])
            
            # 
            self.mtxRot.append([[1,0,0],[0,1,0],[0,0,1]])
            # Get center of the bounding box           
            
            origin = self.modelActor[i].GetCenter()

            #  Set rotation center
            self.modelActor[i].SetOrigin(origin)

        #Initial conditions for bones
        self.initial_pos_actors = np.array([[0,0,0],[0,0,0],[0,0,0],[-0.016, -0.006, 0.163],[-0.016, -0.006, -0.163],[-0.01,  -0.312, -0.173],[-0.01,  -0.312, 0.173],[-0.01, 0.024, 0.166],[-0.01, 0.024, -0.166],[0,0,0],[-0.005, -0.296, -0.153],[-0.005, -0.296,  0.153]])
        for el in range(len(self.initial_pos_actors)):
            # tr = vtk.vtkTransform()            
            # tr.Translate(self.initial_pos_actors[el])
            self.norm_qtr.append(np.array([1.,0.,0.,0.]))
            self.modelActor[el].SetPosition(self.initial_pos_actors[el])           

            
        axes = vtk.vtkAxesActor()
        axes.SetNormalizedTipLength(0.05, 0.05, 0.05)
        axes.SetNormalizedShaftLength(1,1,1)
        self.ren.AddActor(axes)


        # arm_axes = vtk.vtkAxesActor()
        # arm_axes.SetNormalizedTipLength(0.05, 0.05, 0.05)
        # arm_axes.SetTotalLength(0.1,0.1,0.1)

        # transform = vtk.vtkTransform()
        # transform.Translate(-0.016,-0.006,-0.162)
        # arm_axes.SetUserTransform(transform)

        # self.ren.AddActor(arm_axes)       

        #  The axes are positioned with a user transform
        transform = vtk.vtkTransform()
        transform.Translate(0.0, 0.0, 0.0)
        axes.SetUserTransform(transform)

        self.ren.SetBackground(0.1, 0.2, 0.4)

        camera = vtk.vtkCamera()
        camera.SetPosition(2, 0.2, 2)
        camera.SetFocalPoint(0, 0, 0)
        self.ren.SetActiveCamera(camera)
  
        # SetInteractorStyle(MyInteractorStyle())
        #ren.ResetCamera()
        # ren.Render()
        self.iniOk_qt = True
        return self.ren

    # def SetQuatOrientation( self, quaternion, shift, i_actor ): #function to rotate one element around his own origine point
    #     if not self.iniOk_qt :
    #         raise Exception("vtpDrawScene not initialized. Call initScene() first")

    #     vtk.vtkMath().QuaternionToMatrix3x3(quaternion, self.mtxRot[i_actor])
    #     # Rotation: convert 3x3 to 4x4 matrix
    #     mtxTr2 = vtk.vtkMatrix4x4() # identity mtx
    #     for i in range(3):
    #         for j in range(3) :
    #             mtxTr2.SetElement(i, j, self.mtxRot[i_actor][i][j])

    def addVector(self,quaternion):
        # make mapper for the data
        arrowSource = vtk.vtkArrowSource()

        arrowSource.SetShaftRadius(0.01)
        arrowSource.SetTipLength(.1)
        # arrowSource.SetNormalizedTipLength(0.05, 0.05, 0.05)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(arrowSource.GetOutputPort())        
            
        # create actor, set its mapper
        self.modelActor.append(vtk.vtkActor())
        
        el = len(self.modelActor) - 1
        self.modelActor[el].SetMapper(mapper)
        
        # Add the actor to the renderer, set the background and size.
        self.ren.AddActor(self.modelActor[el])
        
        # 
        self.mtxRot.append([[0,0,0],[0,0,0],[0,0,0]])
        # Get center of the bounding box           
        
        origin = self.modelActor[el].GetCenter()

        #  Set rotation center
        self.modelActor[el].SetOrigin(origin)
        self.modelActor[el].SetPosition([-0.016,-0.006,-0.162])

        
        mtxRot = np.identity(3)
        vtk.vtkMath().QuaternionToMatrix3x3(quaternion, mtxRot)

        # Rotation: convert 3x3 to 4x4 matrix
        mtxTr = vtk.vtkMatrix4x4() # identity mtx
        for i in range(3):
            for j in range(3) :
                mtxTr.SetElement(i, j, mtxRot[i][j])

        tr = vtk.vtkTransform()
        tr.PreMultiply()  
        tr.Concatenate(mtxTr)
        self.modelActor[el].SetUserTransform(tr)



    def SetRefQuatOrientation( self, quaternion, shift, i_actor, motion_flag ): #function to rotate all elements around origine
        if not self.iniOk_qt :
            raise Exception("vtpDrawScene not initialized. Call initScene() first")

        mtxRot = np.identity(3)
        vtk.vtkMath().QuaternionToMatrix3x3(quaternion, mtxRot)       

        # rotation in vtk coordinates
        # mtxRot = mtxRot.dot(self.mtxGlob)


        # Rotation: convert 3x3 to 4x4 matrix
        mtxTr = vtk.vtkMatrix4x4() # identity mtx
        for i in range(3):
            for j in range(3) :
                mtxTr.SetElement(i, j, mtxRot[i][j])

        if (motion_flag[0] == 1):

            mtxRot = np.identity(3)
            vtk.vtkMath().QuaternionToMatrix3x3(motion_flag[1], mtxRot)

            # Rotation: convert 3x3 to 4x4 matrix
            mtxTr2 = vtk.vtkMatrix4x4() # identity mtx
            for i in range(3):
                for j in range(3) :
                    mtxTr2.SetElement(i, j, mtxRot[i][j])

            # three transforms:
            # 1. move the object so the rotation center is in the coord center  
            tr = vtk.vtkTransform()
            # origin = np.array(self.modelActor[i_actor].GetOrigin()) # use if we need rotation in geometric centre            
            position = np.array(self.modelActor[i_actor].GetPosition()) #use if we need rotation in local FOR
            # global_origin = np.array([0.,0.,0.]) #when we need rotation in global FOR
            # trans = origin
            # trans = origin + position
            trans = position
            # trans =  global_origin    
            tr.Translate(-trans)
            mtxTr1 = tr.GetMatrix()          

            # 2. rotate around coord center using mtxTr2
            mtxTr12 = vtk.vtkMatrix4x4()
            vtk.vtkMatrix4x4().Multiply4x4(mtxTr2, mtxTr1, mtxTr12)
        
            ## 3. move the object back
            tr = vtk.vtkTransform()
            tr.Translate(trans + np.array(shift))

            mtxTr3 = tr.GetMatrix()
            mtxTr123 = vtk.vtkMatrix4x4()
            vtk.vtkMatrix4x4().Multiply4x4(mtxTr3, mtxTr12, mtxTr123)
            # print('step 3', mtxTr)

            mtxTr0 = mtxTr
            mtxTr = vtk.vtkMatrix4x4()
            vtk.vtkMatrix4x4().Multiply4x4(mtxTr0, mtxTr123, mtxTr)

            
        tr = vtk.vtkTransform()
        tr.PreMultiply()  
        tr.Concatenate(mtxTr)
        self.modelActor[i_actor].SetUserTransform(tr)



    def __init__(self):
        self.iniOk = False

    # def __del__(self):
    #     self.ren.Finalize()
    
    def init_pos_actor( self, i_actor, mov):
        # if not self.iniOk :
        #     raise Exception("vtpDrawScene not initialized. Call initScene() first")
        # set initial pos for hands
        real_pos = np.array(self.modelActor[i_actor].GetPosition())
        new_pos = real_pos + mov
        print(new_pos)
        self.modelActor[i_actor].SetPosition(new_pos)
        
        # self.renWin.Render()
    # def reInitialize_actors(self, qtr):
    #     for el in range(len(self.modelActor)):
    #         # get matrix from vtk obj
    #         elemmtx = self.modelActor[el].GetMatrix()
    #         # temp_2 = np.array([0.,0.,0.,0.])
    #         # vtk.vtkMath().Matrix3x3ToQuaternion(self.modelActor[el].GetMatrix(), temp_2)
    #         temp = self.getRotFromVtkMtx(elemmtx)
    #         # print(temp_3, " ", temp_2)
    #         # get rotation that bring element in visualization to initial
    #         temp0 = QC.qtr_un_calculus(qtr,temp.as_quat())
    #         self.norm_qtr[el] = temp0
    #         #print(str(temp.as_quat()) + ' == ' + str(QC.qtr_multiplication(qtr,temp0)))

    # def getRotFromVtkMtx(self, vtkMtx):
    #     rotMtx = np.zeros((3,3))
    #     for i in range(3):
    #         for j in range(3):
    #             rotMtx[i, j] = vtkMtx.GetElement(i, j)
    #     # get rotation from matrix
    #     rot = R.from_dcm(rotMtx)

    # Автоматическое включение нужной сети
    # масштабирование сегментов
    # При отключении датчика увод его на перезагрузку
    # проверять закрытие сокетов и деструкторов
    # автоматическое кадрирование
    # оптимизация длинны звеньев
    # 


