import vtk
import numpy as np
from scipy.spatial.transform import Rotation as R

def get_colored_box_mapper( size_x = 1, size_y = 4, size_z = 2 ):
    
    # Colored faces cube setup
    cube_source = vtk.vtkCubeSource()
#    cube_source.SetCenter(center_x, center_y, center_z)
    cube_source.SetXLength(size_x)
    cube_source.SetYLength(size_y)
    cube_source.SetZLength(size_z)
    
    cube_source.Update()
    
    face_colors = vtk.vtkUnsignedCharArray()
    face_colors.SetNumberOfComponents(3)
    face_x_plus = [255,0,0]
    face_x_minus = [160,0,0]
    face_y_plus = [0,255,0]
    face_y_minus = [0,160,0]
    face_z_plus = [0,0,255]
    face_z_minus = [0,0,160]
    face_colors.InsertNextTypedTuple(face_x_minus)
    face_colors.InsertNextTypedTuple(face_x_plus)
    face_colors.InsertNextTypedTuple(face_y_minus)
    face_colors.InsertNextTypedTuple(face_y_plus)
    face_colors.InsertNextTypedTuple(face_z_minus)
    face_colors.InsertNextTypedTuple(face_z_plus)
    
    cube_source.GetOutput().GetCellData().SetScalars(face_colors)
    cube_source.Update()
    
    cube_mapper = vtk.vtkPolyDataMapper()
    cube_mapper.SetInputData(cube_source.GetOutput())
    cube_mapper.Update()
    
    return cube_mapper

class InteractorStyle(vtk.vtkInteractorStyleTrackballActor):

    def __init__(self,parent=None):
        self.AddObserver("LeftButtonPressEvent",self.leftButtonPressEvent)
        self.AddObserver("LeftButtonReleaseEvent",self.leftButtonReleaseEvent)
        self.AddObserver("MouseMoveEvent",self.mouseMoveEvent)
        self.AddObserver("KeyPressEvent", self.keypressEvent)
        
        self._calibStage = 0  # actors are not connected
        self._imuArmOn = False  # is on when the left button is down 
        self._imuRefOn = False  # is on when the left button is down 
        self._armActor = None
        self._imuArmActor = None
        self._imuRefActor = None
        self._axesActor = None
        self._bodyArmActor = None
        self._textActor = None
        self._rotArmAtImuArm = None # arm rel to arm IMU
        self._rotImuArmAtImuRef = None # arm IMU rel to ref IMU
        self._N_rotImuArmAtGlob = None # IMU_arm at N-pose rel to glob
        self._N_rotImuRefAtGlob = None # IMU_ref at N-pose rel to glob
        self._rotBodyAtImuRef = None # calculated during calibration       
    
    def leftButtonPressEvent(self,obj,event):
#        print("Left Button pressed")
        self._controlOn = True
        clickPos = self.GetInteractor().GetEventPosition()

        picker = vtk.vtkPropPicker()
        picker.Pick(clickPos[0], clickPos[1], 0, self._ren)
        pickedActor = picker.GetActor()
#        if pickedActor is self._armActor:
#            print("Picked arm actor")
        if pickedActor is self._imuArmActor:
#            print("Picked IMU arm actor")
            self._imuArmOn = True
        elif pickedActor is self._imuRefActor:
#            print("Picked IMU ref actor")
            self._imuRefOn = True
            if self._calibStage == 2 :
                self.calcImuArmAtImuRef()
        self.OnLeftButtonDown()

    def leftButtonReleaseEvent(self,obj,event):
#        print("Left Button released")
        self._imuArmOn = False   
        self._imuRefOn = False   
        self.OnLeftButtonUp()
        
    def getRotFromVtkMtx(self, vtkMtx):
        rotMtx = np.zeros((3,3))
        for i in range(3):
            for j in range(3):
                rotMtx[i, j] = vtkMtx.GetElement(i, j)
        # get rotation from matrix
        rot = R.from_dcm(rotMtx)
        return rot
        
    def getVtkMtxFromRot(self, rot):
        rotMtx = np.identity(4)
        rotMtx[0:3,0:3] = rot.as_dcm()
        vtkMtx = vtk.vtkMatrix4x4()
        for i in range(4):
            for j in range(4):
                vtkMtx.SetElement(i, j, rotMtx[i, j])
        return vtkMtx
        
    def alignArmToImuArm(self):
        # Used to position Arm relative to IMU_Arm in the interactive mode
        # get IMU_arm@Glob rotation
        mtxImuArm = self._imuArmActor.GetMatrix()
        rotImuArmAtGlob = self.getRotFromVtkMtx(mtxImuArm)
        # this is the order required by scipy.Rotation:
        # IMU_Arm rotation followed by arm@arm_imu rotation
        rotArmAtGlob = rotImuArmAtGlob * self._rotArmAtImuArm
        vtkMtx = self.getVtkMtxFromRot(rotArmAtGlob)
        self._armActor.SetUserMatrix(vtkMtx)
        
    def alignImuArmToImuRef(self):
        # Used to position ImuArm relative to ImuRef in the interactive mode
        # get ImuRef@Glob rotation
        mtxImuRef = self._imuRefActor.GetMatrix()
        rotImuRefAtGlob = self.getRotFromVtkMtx(mtxImuRef)
        # this is the order required by scipy.Rotation:
        # IMU_Arm rotation followed by arm@arm_imu rotation
        rotImuArmAtGlob = rotImuRefAtGlob * self._rotImuArmAtImuRef 
        vtkMtx = self.getVtkMtxFromRot(rotImuArmAtGlob)
        self._imuArmActor.SetUserMatrix(vtkMtx)
        
    def updateAxesViewport(self):
        mtxImuRef = self._imuRefActor.GetMatrix()
        rotImuRefAtGlob = self.getRotFromVtkMtx(mtxImuRef)
        rotBodyAtGlob = rotImuRefAtGlob * self._rotBodyAtImuRef
        vtkMtx = self.getVtkMtxFromRot(rotBodyAtGlob)
        axesTrans = vtk.vtkTransform()
        axesTrans.SetMatrix(vtkMtx)
        self._axesActor.SetUserTransform(axesTrans)
 
        # draw arm in body frame
        mtxImuArmAtGlob = self._imuArmActor.GetMatrix()
        rotImuArmAtGlob = self.getRotFromVtkMtx(mtxImuArmAtGlob)
        rotArmAtGlob = rotImuArmAtGlob * self._rotArmAtImuArm
        rotArmAtBody = rotBodyAtGlob.inv() * rotArmAtGlob
        vtkMtx = self.getVtkMtxFromRot(rotArmAtBody)
        self._bodyArmActor.SetUserMatrix(vtkMtx)
        
    def calcArmAtImuArm(self):
        # calc arm @ imu_arm rotation
        mtxImu = self._imuArmActor.GetMatrix()
        rotImuArmAtGlob = self.getRotFromVtkMtx(mtxImu)
        mtxArm = self._armActor.GetMatrix()
        rotArmAtGlob = self.getRotFromVtkMtx(mtxArm)
        # this is the order required by scipy.Rotation:
        # inverse IMU rotation followed by Arm rotation
        self._rotArmAtImuArm = rotImuArmAtGlob.inv()  * rotArmAtGlob 
       
    def calcImuArmAtImuRef(self):
        # calc imu_ar_arm @ imu_ref rotation
        mtxImuArm = self._imuArmActor.GetMatrix()
        rotImuArmAtGlob = self.getRotFromVtkMtx(mtxImuArm)
        mtxImuRef = self._imuRefActor.GetMatrix()
        rotImuRefAtGlob = self.getRotFromVtkMtx(mtxImuRef)
        # this is the order required by scipy.Rotation:
        # inverse IMU rotation followed by Arm rotation
        self._rotImuArmAtImuRef = rotImuRefAtGlob.inv()  * rotImuArmAtGlob 
        
    def calcCalib_1(self):
        
        self.calcArmAtImuArm() # we need it for the interactive display, not calib
        
        # Get the IMU_arm orientation: N_IMU_arm@Glob
        mtxImuArm = self._imuArmActor.GetMatrix()
        self._N_rotImuArmAtGlob = self.getRotFromVtkMtx(mtxImuArm)
        # Get the IMU_ref orientation: N_IMU_ref@Glob
        mtxImuRef = self._imuRefActor.GetMatrix()
        self._N_rotImuRefAtGlob = self.getRotFromVtkMtx(mtxImuRef)
        
    def calcCalib_2(self):
        # Get the IMU_arm orientation: T_IMU_arm_@Glob
        mtxImuArmAtGlob = self._imuArmActor.GetMatrix()
        T_rotImuArmAtGlob = self.getRotFromVtkMtx(mtxImuArmAtGlob)
        # calc arm rot in T-pose using the offset rot from stage 1
        T_rotArmAtGlob = T_rotImuArmAtGlob * self._N_rotImuArmAtGlob.inv() 
        T_mtxArmAtGlob = T_rotArmAtGlob.as_dcm()
        # use second column of the matrix which is the Y direction in T-pose
        T_Y_dirArmAtGlob = T_mtxArmAtGlob[:,1] 
        # Compose body rot using 3 vec columns
        X_dirBodyAtGlob = T_Y_dirArmAtGlob
        Y_dirBodyAtGlob = np.array([0, 1, 0])  # y - axis dir
        # the third vec is the cross-product of the two
        Z_dirBodyAtGlob = np.cross(X_dirBodyAtGlob, Y_dirBodyAtGlob)
        # combine as rows and transpose
        mtxBodyAtGlob = np.array([X_dirBodyAtGlob, 
                                  Y_dirBodyAtGlob, 
                                  Z_dirBodyAtGlob]).transpose()
        rotBodyAtGlob = R.from_dcm(mtxBodyAtGlob)
        
        # calc body rot rel to the ImuRef
        mtxImuRefAtGlob = self._imuRefActor.GetMatrix()
        rotImuRefAtGlob = self.getRotFromVtkMtx(mtxImuRefAtGlob)
        self._rotBodyAtImuRef = rotImuRefAtGlob.inv()  * rotBodyAtGlob
        
        # calc arm rot rel to IMU arm
        # we assume an arm which frame is aligned to the body frame in N position
        # we use an identity mtx here, but we can use any other mtx describing 
        # rotation of a body part rel to the body frame
        N_rotArmAtBody = R.from_dcm(np.identity(3))
        
        rotBodyAtImuArm = self._N_rotImuArmAtGlob.inv() * rotBodyAtGlob 
        self._rotArmAtImuArm = rotBodyAtImuArm * N_rotArmAtBody 
       
        self.updateAxesViewport()
        
        print("N ImuArm@Glob\n", self._N_rotImuArmAtGlob.as_dcm())
        print("T ImuArm@Glob\n", T_rotImuArmAtGlob.as_dcm())
        print("T Arm@Glob\n", T_rotArmAtGlob.as_dcm())
        print("mtxBodyAtGlob\n", mtxBodyAtGlob)
        print("rotBodyAtGlob\n", rotBodyAtGlob.as_dcm())
        print("rotArmAtImuArm\n", self._rotArmAtImuArm.as_dcm())
        print("", flush=True) 

    def mouseMoveEvent(self,obj,event):
        if self._calibStage == 0 :
            self.OnMouseMove()   # call parent class handler
        
        elif self._calibStage == 1 :
            if self._imuArmOn :
                self.alignArmToImuArm()
                self.OnMouseMove()   # call parent class handler
                
        elif self._calibStage == 2 :
            self.updateAxesViewport()
            if self._imuArmOn :
                self.alignArmToImuArm()
                self.OnMouseMove()   # call parent class handler
            elif self._imuRefOn :
                self.alignImuArmToImuRef()
                self.alignArmToImuArm()
                self.OnMouseMove()   # call parent class handler
                
    
    def keypressEvent(self, obj, event):
        key = self._iren.GetKeySym()
            
        if key =="0":
            # print("Keypress 0", flush=True) 
            self._calibStage = 0
            self._textActor.SetInput("Calibration stage 0")
            self._renWin.Render()
            
        elif key =="1" and self._calibStage == 0:
            # print("Keypress 1", flush=True) 
            self._calibStage = 1
            self.calcCalib_1()
            self._textActor.SetInput("Calibration stage 1")
            self._renWin.Render()
            
        elif key =="2" and self._calibStage == 1:
            # print("Keypress 2", flush=True) 
            self._calibStage = 2
            self.calcCalib_2()
            self._textActor.SetInput("Calibration done")
            self._renWin.Render()
            
        # elif key =="d":
        #     print("Keypress d", flush=True) 
        #     mtxImu = self._imuArmActor.GetMatrix()
        #     mtxArm = self._armActor.GetMatrix()
        #     print(mtxImu)
        #     print(mtxArm)
            
        elif key =="q":
            print("Exit", flush=True) 
            self._iren.GetRenderWindow().Finalize()
            self._iren.TerminateApp()


    def SetActors(self, renWin, ren, iren, 
                  armActor, imuArmActor, imuRefActor, 
                  axesActor, bodyArmActor, textActor):
        self._renWin = renWin
        self._ren = ren
        self._iren = iren
        self._armActor = armActor
        self._imuArmActor = imuArmActor
        self._imuRefActor = imuRefActor
        self._axesActor = axesActor
        self._bodyArmActor = bodyArmActor
        self._textActor = textActor


# create a rendering window and renderers
ren = vtk.vtkRenderer()
renCornerR = vtk.vtkRenderer()
renCornerL = vtk.vtkRenderer()
renWin = vtk.vtkRenderWindow()
renWin.AddRenderer(ren)
renWin.AddRenderer(renCornerR)
renWin.AddRenderer(renCornerL)
renWin.SetSize(1024,768)
renCornerL.SetViewport(0, 0.02, 0.2, 0.22)
renCornerR.SetViewport(0.8, 0, 1.0, 0.2)

armActor = vtk.vtkActor()
armActor.SetMapper(get_colored_box_mapper(0.5,10,0.5))
vtkMtx = vtk.vtkMatrix4x4()
vtkMtx.Identity()
# needed to set user matrix for correct rel rotation (why?)
armActor.SetUserMatrix(vtkMtx)

imuArmActor = vtk.vtkActor()
imuArmActor.SetMapper(get_colored_box_mapper())
vtkMtx = vtk.vtkMatrix4x4()
vtkMtx.Identity()
# needed to set user matrix for correct rel rotation (why?)
imuArmActor.SetUserMatrix(vtkMtx)

imuRefActor = vtk.vtkActor()
imuRefActor.SetMapper(get_colored_box_mapper())
imuRefActor.SetPosition(4,0,0)

# Create a text actor.
txtActor = vtk.vtkTextActor()
txtActor.SetInput("Calibration stage 0")
txtProp = txtActor.GetTextProperty()
txtProp.SetFontFamilyToArial()
txtProp.SetFontSize(24)
txtProp.SetColor((1, 1, 0))
txtActor.SetDisplayPosition(400, 10)

# assign actors to the renderer
ren.AddActor(armActor)
ren.AddActor(imuArmActor)
ren.AddActor(imuRefActor)
ren.AddActor(txtActor)

# Right corner viewport
axesActor = vtk.vtkAxesActor()
axesActor.SetAxisLabels(1) # 0/1 to turn off/on axis labels
renCornerR.AddActor(axesActor)

bodyArmActor = vtk.vtkActor()
bodyArmActor.SetMapper(get_colored_box_mapper(0.5,2,0.5))
vtkMtx = vtk.vtkMatrix4x4()
vtkMtx.Identity()
# needed to set user matrix for correct rel rotation (why?)
bodyArmActor.SetUserMatrix(vtkMtx)
renCornerL.AddActor(bodyArmActor)

lightKit = vtk.vtkLightKit()
lightKit.SetKeyLightIntensity(1)
lightKit.SetKeyToBackRatio(3.5)
lightKit.AddLightsToRenderer(ren)

lightKitL = vtk.vtkLightKit()
lightKitL.SetKeyLightIntensity(1)
lightKitL.SetKeyToBackRatio(3.5)
lightKitL.AddLightsToRenderer(renCornerL)

ren.ResetCamera()
renCornerL.ResetCamera()
renCornerR.ResetCamera()

# create a renderwindowinteractor
iren = vtk.vtkRenderWindowInteractor()
style = InteractorStyle()    # here we create our own style
style.SetActors( renWin, ren, iren, 
                armActor, imuArmActor, imuRefActor, 
                axesActor, bodyArmActor, txtActor)
iren.SetInteractorStyle(style) 
iren.SetRenderWindow(renWin)

iren.Initialize()
iren.Start()

iren.GetRenderWindow().Finalize()
iren.TerminateApp()






