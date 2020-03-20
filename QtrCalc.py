import math
import numpy as np

# def qtr_calculus(self,i_from_actor,i_to_actor): 

#         i_from_actor = 1 #в последствии будте таблица соответствия актеров и потоков, пока остается константами      
#         i_to_actor = 0
        
#         mod_qtr = self.qtrs[1][0]*self.qtrs[1][0] + self.qtrs[1][1]*self.qtrs[1][1] + self.qtrs[1][2]*self.qtrs[1][2] + self.qtrs[1][3]*self.qtrs[1][3] # mod for multiplication of quaternions
        
#         # self.qtrs[i_from_actor] - a - quater
#         # self.qtrs[i_to_actor]   - b - quater

#         a1 = self.qtrs[i_from_actor][0]*self.qtrs[i_to_actor][0] - self.qtrs[i_from_actor][1]*self.qtrs[i_to_actor][1] - self.qtrs[i_from_actor][2]*self.qtrs[i_to_actor][2] - self.qtrs[i_from_actor][3]*self.qtrs[i_to_actor][3]
#         a2 = self.qtrs[i_from_actor][1]*self.qtrs[i_to_actor][0] + self.qtrs[i_from_actor][0]*self.qtrs[i_to_actor][1] - self.qtrs[i_from_actor][3]*self.qtrs[i_to_actor][2] + self.qtrs[i_from_actor][2]*self.qtrs[i_to_actor][3]
#         a3 = self.qtrs[i_from_actor][2]*self.qtrs[i_to_actor][0] + self.qtrs[i_from_actor][3]*self.qtrs[i_to_actor][1] + self.qtrs[i_from_actor][0]*self.qtrs[i_to_actor][2] - self.qtrs[i_from_actor][1]*self.qtrs[i_to_actor][3]
#         a4 = self.qtrs[i_from_actor][3]*self.qtrs[i_to_actor][0] - self.qtrs[i_from_actor][2]*self.qtrs[i_to_actor][1] + self.qtrs[i_from_actor][1]*self.qtrs[i_to_actor][2] + self.qtrs[i_from_actor][0]*self.qtrs[i_to_actor][3]
        
#         qtr_mult = np.array([a1/mod_qtr,a2/mod_qtr,a3/mod_qtr,a4/mod_qtr])
#         # print(qtr_multiplication)
#         return qtr_mult #vtk.vtkQuaterniond(np.array(self.qtrs[1])+np.array(qtr)).Normalized()
    
def qtr_norm(temp_qtr):
    
    sum = temp_qtr[0]*temp_qtr[0] + temp_qtr[1]*temp_qtr[1] +  temp_qtr[2]*temp_qtr[2] +  temp_qtr[3]*temp_qtr[3]

    temp_qtr = np.array([temp_qtr[0]/sum,temp_qtr[1]/sum,temp_qtr[2]/sum,temp_qtr[3]/sum])

    return temp_qtr

def qtr_inv(qtr_a):

    a1 = qtr_a[0]
    a2 = qtr_a[1]
    a3 = qtr_a[2]
    a4 = qtr_a[3]

    mod = a1*a1 + a2*a2 + a3*a3 + a4*a4

    qtr_inf = np.array([a1/mod, - a2/mod,- a3/mod, - a4/mod])

    return qtr_inf

def qtr_multiplication(qtr_a, qtr_b): #Multiplication quaternions = A*B

    qtr_a = np.array(qtr_a)
    qtr_b = np.array(qtr_b)
    mod_qtr = qtr_a[0]*qtr_a[0] + qtr_a[1]*qtr_a[1] + qtr_a[2]*qtr_a[2] + qtr_a[3]*qtr_a[3] # mod for multiplication of quaternions
    
    a1 = qtr_a[0]*qtr_b[0] - qtr_a[1]*qtr_b[1] - qtr_a[2]*qtr_b[2] - qtr_a[3]*qtr_b[3]
    a2 = qtr_a[1]*qtr_b[0] + qtr_a[0]*qtr_b[1] + qtr_a[3]*qtr_b[2] - qtr_a[2]*qtr_b[3]
    a3 = qtr_a[2]*qtr_b[0] - qtr_a[3]*qtr_b[1] + qtr_a[0]*qtr_b[2] + qtr_a[1]*qtr_b[3]
    a4 = qtr_a[3]*qtr_b[0] + qtr_a[2]*qtr_b[1] - qtr_a[1]*qtr_b[2] + qtr_a[0]*qtr_b[3]
    
    qtr_mult = np.array([a1/mod_qtr,a2/mod_qtr,a3/mod_qtr,a4/mod_qtr])
    # print(qtr_multiplication)
    return qtr_mult

def qtr_un_calculus(qtr_a,qtr_c): #to find qtr_b in equation qtr_c = qtr_a**qtr_b  (where ** - quaterninal multiplication)
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

def qtr_calculus(qtr_a,qtr_b): #to find qtr_x in equation qtr_a = qtr_x**qtr_b  (where ** - quaterninal multiplication)     
    mod_qtr = qtr_b[0]*qtr_b[0] + qtr_b[1]*qtr_b[1] + qtr_b[2]*qtr_b[2] + qtr_b[3]*qtr_b[3] # mod^2 for multiplication of quaternions

    a1 = qtr_a[0]*qtr_b[0] + qtr_a[1]*qtr_b[1] + qtr_a[2]*qtr_b[2] + qtr_a[3]*qtr_b[3]
    a2 = qtr_a[1]*qtr_b[0] - qtr_a[0]*qtr_b[1] + qtr_a[3]*qtr_b[2] - qtr_a[2]*qtr_b[3]
    a3 = qtr_a[2]*qtr_b[0] - qtr_a[3]*qtr_b[1] - qtr_a[0]*qtr_b[2] + qtr_a[1]*qtr_b[3]
    a4 = qtr_a[3]*qtr_b[0] + qtr_a[2]*qtr_b[1] - qtr_a[1]*qtr_b[2] - qtr_a[0]*qtr_b[3]

    qtr_mult = np.array([a1/mod_qtr,a2/mod_qtr,a3/mod_qtr,a4/mod_qtr])
    return qtr_mult

def three_qtr_solve(qtr_c,qtr_a,qtr_b): # C = X*A*B (looking for X)

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

def three_qtr_multiplication(qtr_a,qtr_b,qtr_c): # X = A*B*C (looking for X)

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

