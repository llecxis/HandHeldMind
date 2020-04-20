import numpy as np
from scipy.spatial.transform import Rotation as R

class SegmentIMU:
    
    def __init__(self, N_segmentAtBody, N_imuAtGlob):
        self.N_segmentAtBody = N_segmentAtBody
        self.N_imuAtGlob = N_imuAtGlob
        self.segmentAtImu = None
        

class CalibIMUs:
    
    def __init__(self):
        self.__iCalibStage = 0
        self.__N_refImuAtGlob = None # measured at stage 1
        self.__N_armImuAtGlob = None # measured at stage 1
        self.__bodyAtRefImu = None # calculated at stage 2
        self.__segImus = []
        
        

    def __del__(self):
        pass
        
    
    # N_refImuAtGlob - measured ref IMU rotation in N-pose
    # N_armImuAtGlob - measured arm IMU rotation in N-pose
    def setN_calibImusAtGlob(self, N_refImuAtGlob, N_armImuAtGlob):
        # TODO: check that we are at stage 0
        # 
        self.__N_refImuAtGlob = N_refImuAtGlob
        self.__N_armImuAtGlob = N_armImuAtGlob
        self.__iCalibStage = 1


    # N_segmentAtBody - anatomical rotation of a segment in N-pose rel to body
    # N_imuAtGlob - measured segment IMU rotation in N-pose
    # Arm used in the calibration should also be added as a segment
    def addSegmentImu(self, N_segmentAtBody, N_imuAtGlob ):
        # TODO: check that we are at stage 0
        
        imu = SegmentIMU(N_segmentAtBody, N_imuAtGlob)
        self.__segImus.append( imu ) 
        
    
    # T_armImuAtGlob - measured arm IMU rotation in T-pose
    # For now we don't use second argument, 
    # we assume VTK FOR where Y-axis is pointing up
    def doCalibration(self, T_armImuAtGlob, dirUpVector ):
        # TODO: check that we are at stage 1
        # calc arm rot in T-pose using the offset rot from stage 1
        T_armAtGlob = T_armImuAtGlob * self.__N_armImuAtGlob.inv() 
        T_mtxArmAtGlob = T_armAtGlob.as_matrix()
        # use third column of the matrix which is the direction of
        # Y-axis of arm's FOR, it is horizontal now
        T_Z_dirArmAtGlob = T_mtxArmAtGlob[:,1] 
        # Compose body rot using 3 vec columns
        X_dirBodyAtGlob = T_Z_dirArmAtGlob
        Y_dirBodyAtGlob = np.array([0, 1, 0])  # Y - axis dir
        # the third vec is the cross-product of the two
        Z_dirBodyAtGlob = np.cross(X_dirBodyAtGlob, Y_dirBodyAtGlob)
        # combine as rows and transpose
        mtxBodyAtGlob = np.array([X_dirBodyAtGlob, 
                                  Y_dirBodyAtGlob, 
                                  Z_dirBodyAtGlob]).transpose()
        bodyAtGlob = R.from_matrix(mtxBodyAtGlob)
        
        # calc body rot rel to the ImuRef
        self.__bodyAtRefImu = self.__N_refImuAtGlob.inv()  * bodyAtGlob
        
        # calc segment rot rel to segment IMU 
        for seg in self.__segImus :
            bodyAtSegImu = seg.N_imuAtGlob.inv() * bodyAtGlob
            seg.segmentAtImu = bodyAtSegImu * seg.N_segmentAtBody
        
        self.__iCalibStage = 2
        
        
    def calcBodyAtGlob(self, refImuAtGlob ):
       # TODO: check that we are at stage 2
       # Body@Glob = Body@refImu * refImu@Glob
        return refImuAtGlob * self.__bodyAtRefImu

    def calcSegmentAtGlob(self, iSeg, segImuAtGlob):
        # TODO: check that we are at stage 2
        # Seg@Glob = Seg@SegImu * SegImu@Glob
        segAtImu = self.__segImus[iSeg].segmentAtImu
        segAtGlob = segImuAtGlob * segAtImu
        return segAtGlob
        
    def calcSegmentAtBody(self, iSeg, segImuAtGlob, refImuAtGlob):
        # TODO: check that we are at stage 2
        # Seg@Body = SegAtGlob * Body@Glob.inv()
        segAtGlob = self.calcSegmentAtGlob(iSeg, segImuAtGlob)
        bodyAtGlob = self.calcBodyAtGlob(refImuAtGlob)
        segAtBody = bodyAtGlob.inv() * segAtGlob
        return segAtBody
        
        
    
        
        
        
if __name__ == "__main__":
    
    calib = CalibIMUs()
    
    calib.setN_calibImusAtGlob(R.random(), R.random())
    for i in range(0, 4):
        calib.addSegmentImu(R.random(), R.random())
    
    calib.doCalibration(R.random(), np.array([0,0,1]))
    
    print(calib.calcBodyAtGlob(R.random()).as_quat() )
    for iStep in range (0, 1000):
        for i in range(0, 4):
            print(calib.calcSegmentAtGlob(1, R.random()).as_quat() )
            print(calib.calcSegmentAtBody(1, R.random(), R.random()).as_quat() )
    
    
    # input("Press Enter to exit")
    
#    del calib
        