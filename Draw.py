#import sys
from PyQt5 import QtGui
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import vtk
#import math
import numpy as np
#import time
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
#import socket
#import traceback

class vtpDrawScene: 
    def SetQuatOrientation( self, quaternion, shift, i_actor ):
        # if not self.iniOk :
        #     raise Exception("vtpDrawScene not initialized. Call initScene() first")

        # Convert quat to the rotation matrix
        # self.mtxRot =  [[0,0,0],[0,0,0],[0,0,0]]
        vtk.vtkMath().QuaternionToMatrix3x3(quaternion, self.mtxRot[i_actor])
        # print(self.mtxRot[i_actor])

        # norm matrix
        self.mtxRot[i_actor] = np.array(self.mtxRot[i_actor]).dot(np.array(self.norm_mat[i_actor]))

        # Rotation: convert 3x3 to 4x4 matrix
        mtxTr2 = vtk.vtkMatrix4x4() # identity mtx
        for i in range(3):
            for j in range(3) :
                mtxTr2.SetElement(i, j, self.mtxRot[i_actor][i][j])
      
        # three transforms:
        # 1. move the object so the rotation center is in the coord center  
        tr = vtk.vtkTransform()
        origin = np.array(self.modelActor[i_actor].GetOrigin())
        
        position = np.array(self.modelActor[i_actor].GetPosition())
        #trans = origin + position
        trans = position
        #trans = origin
        tr.Translate(-trans)
        mtxTr1 = tr.GetMatrix()
        
        # 2. rotate around coord center using mtxTr2
        mtxTr12 = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4().Multiply4x4 (mtxTr2, mtxTr1, mtxTr12)
        
        ## 3. move the object back
        tr = vtk.vtkTransform()
        tr.Translate(trans + np.array(shift))
        mtxTr3 = tr.GetMatrix()
        mtxTr123 = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4().Multiply4x4 (mtxTr3, mtxTr12, mtxTr123)
        
        tr = vtk.vtkTransform()
        tr.PreMultiply()  
    #    tr.PostMultiply()  
        tr.Concatenate(mtxTr123) 
        self.modelActor[i_actor].SetUserTransform(tr)

        # self.ren.Render()
        # return self.ren
        # self.renWin.Render()
    
    def initScene_qt(self, obj):
        reader = list()
        modelMapper = list()
        self.modelActor = list()
        self.mtxRot = list()
        self.norm_mat = list()

        self.ren = vtk.vtkRenderer()

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
            self.mtxRot.append([[0,0,0],[0,0,0],[0,0,0]])
            # Get center of the bounding box           
            
            origin = self.modelActor[i].GetCenter()
            
            #
            #  Set rotation center
            self.modelActor[i].SetOrigin(origin)
             
        #Initial conditions for bones
        self.initial_pos_actors = np.array([[0,0,0],[0,0,0],[0,0,0],[-0.016, -0.006, 0.163],[-0.016, -0.006, -0.163],[-0.01,  -0.312, -0.173],[-0.01,  -0.312, 0.173],[-0.01, 0.024, 0.166],[-0.01, 0.024, -0.166],[0,0,0],[-0.005, -0.296, -0.153],[-0.005, -0.296,  0.153]])
        for el in range(len(self.initial_pos_actors)):
            tr = vtk.vtkTransform()
            
            tr.Translate(self.initial_pos_actors[el])
            
            # mtxTr1 = tr.GetMatrix()
            
            # mtxTr2 = vtk.vtkMatrix4x4() # identity mtx
            # temp = np.array([[1.,0.,0.,0.],[0.,0.,-1.,0.],[0.,1.,0.,0.],[0.,0.,0.,1.]])
            # for i in range(3):
            #     for j in range(3) :
            #         mtxTr2.SetElement(i, j, temp[i][j])
            
            # mtxTr12 = vtk.vtkMatrix4x4()
            # #print(mtxTr2)
            # vtk.vtkMatrix4x4().Multiply4x4(mtxTr2, mtxTr1, mtxTr12)
            
            # tr = vtk.vtkTransform()
            # tr.PreMultiply()
            # tr.Concatenate(mtxTr12)
            
            # self.modelActor[el].SetUserTransform(tr)

            self.modelActor[el].SetPosition(self.initial_pos_actors[el])
            self.norm_mat.append([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])  #[[1.0, 1.0, 1.0], [1.0, 1.0, 1.0], [1.0, 1.0, 1.0]])

        # self.modelActor[3].SetPosition(-0.016, -0.006, 0.163)  
        # self.modelActor[4].SetPosition(-0.016, -0.006, -0.163)
        # self.modelActor[7].SetPosition(-0.01, 0.024, 0.166)
        # self.modelActor[8].SetPosition(-0.01, 0.024, -0.166)

        # self.modelActor[5].SetPosition(-0.01,  -0.308, -0.173)
        # self.modelActor[6].SetPosition(-0.01,  -0.308, 0.173)
        # self.modelActor[11].SetPosition(-0.005, -0.296,  0.153)
        # self.modelActor[10].SetPosition(-0.005, -0.296, -0.153)


        axes = vtk.vtkAxesActor()
        axes.SetNormalizedTipLength(0.05, 0.05, 0.05)
        axes.SetNormalizedShaftLength(1,1,1)
        self.ren.AddActor(axes)
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

    # def inter(self, flag):
    #     if (flag == 0):
    #         interactor = vtk.vtkRenderWindowInteractor()
    #         interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())        
    #         interactor.AddObserver("KeyPressEvent",KeyPress)
    #         interactor.SetRenderWindow(self.renWin)
    #         interactor.Start()
    #     if (flag == 1):
    #         interactor.ReInitialize()
    #         interactor.Start()            
    #     if (flag == -1):
    #         interactor = None
    #     return

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
