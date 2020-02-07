#import math
import numpy as np

def three_qtr_solve(qtr_a,qtr_b,qtr_c): # A = X*B*C (looking for X)

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

    # x01 = -((-(-((a3* b1 + a4* b2 + a1* b3 - a2* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (-a4* b1 + a3* b2 - a2* b3 - a1* b4)*(a1* b1 -a2* b2 - a3* b3 - a4* b4))*((a4* b1 - a3* b2 + a2* b3 + a1* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (-a3* b1 - a4* b2 - a1* b3 + a2* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4)) + (-(-a4* b1 + a3* b2 - a2* b3 - a1* b4)*(-a3* b1 -a4* b2 - a1* b3 + a2* b4) + (a2* b1 + a1* b2 - a4* b3 + a3* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((-a2* b1 - a1* b2 + a4* b3 - a3* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (a1* b1 - a2* b2 - a3* b3 -a4* b4)**2))*(((-a2* b1 - a1* b2 + a4* b3 - a3* b4)*(a2* b1 +a1* b2 - a4* b3 + a3* b4) - (a1* b1 - a2* b2 - a3* b3 - a4* b4)**2)*(-(a2* b1 + a1* b2 - a4* b3 + a3* b4)* c1 + (-a4* b1 + a3* b2 - a2* b3 -a1* b4)* c3) - ((-a3* b1 - a4* b2 - a1* b3 + a2* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (-a4* b1 + a3* b2 - a2* b3 -a1* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((a1* b1 -a2* b2 - a3* b3 - a4* b4)* c3 - (a2* b1 + a1* b2 - a4* b3 +a3* b4)* c4)) + (-((a3* b1 + a4* b2 + a1* b3 - a2* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (-a4* b1 + a3* b2 - a2* b3 - a1* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((-a3* b1 - a4* b2 - a1* b3 + a2* b4)*(a2* b1 +a1* b2 - a4* b3 + a3* b4) - (-a4* b1 + a3* b2 - a2* b3 -a1* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4)) + (-(-a4* b1 +a3* b2 - a2* b3 - a1* b4)**2 + (-a2* b1 - a1* b2 + a4* b3 -a3* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4))*((-a2* b1 -a1* b2 + a4* b3 - a3* b4)*(a2* b1 + a1* b2 - a4* b3 +a3* b4) - (a1* b1 - a2* b2 - a3* b3 - a4* b4)**2))*(((-a2* b1 - a1* b2 + a4* b3 - a3* b4)*(a2* b1 +a1* b2 - a4* b3 + a3* b4) - (a1* b1 - a2* b2 - a3* b3 - a4* b4)**2)*(-(a2* b1 + a1* b2 - a4* b3 + a3* b4)* c2 + (-a3* b1 - a4* b2 - a1* b3 +a2* b4)* c3) - ((a4* b1 - a3* b2 + a2* b3 + a1* b4)*(a2* b1 +a1* b2 - a4* b3 + a3* b4) - (-a3* b1 - a4* b2 - a1* b3 +a2* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((a1* b1 -a2* b2 - a3* b3 - a4* b4)* c3 - (a2* b1 + a1* b2 - a4* b3 +a3* b4)* c4)))/((-((a3* b1 + a4* b2 + a1* b3 - a2* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (-a4* b1 + a3* b2 - a2* b3 - a1* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((-a3* b1 - a4* b2 - a1* b3 + a2* b4)*(a2* b1 +a1* b2 - a4* b3 + a3* b4) - (-a4* b1 + a3* b2 - a2* b3 -a1* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4)) + (-(-a4* b1 +a3* b2 - a2* b3 - a1* b4)**2 + (-a2* b1 - a1* b2 + a4* b3 -a3* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4))*((-a2* b1 -a1* b2 + a4* b3 - a3* b4)*(a2* b1 + a1* b2 - a4* b3 +a3* b4) - (a1* b1 - a2* b2 - a3* b3 - a4* b4)**2))*(-((a4* b1 - a3* b2 + a2* b3 + a1* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (a3* b1 + a4* b2 + a1* b3 - a2* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((a4* b1 -a3* b2 + a2* b3 + a1* b4)*(a2* b1 + a1* b2 - a4* b3 +a3* b4) - (-a3* b1 - a4* b2 - a1* b3 + a2* b4)*(a1* b1 -a2* b2 - a3* b3 -a4* b4)) + (-(a3* b1 + a4* b2 + a1* b3 - a2* b4)*(-a3* b1 -a4* b2 - a1* b3 + a2* b4) + (a2* b1 + a1* b2 - a4* b3 + a3* b4)**2)*((-a2* b1 - a1* b2 + a4* b3 - a3* b4)*(a2* b1 +a1* b2 - a4* b3 + a3* b4) - (a1* b1 - a2* b2 - a3* b3 - a4* b4)**2)) - (-((-a3* b1 - a4* b2 - a1* b3 + a2* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (-a4* b1 + a3* b2 - a2* b3 - a1* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((a4* b1 - a3* b2 + a2* b3 + a1* b4)*(a2* b1 +a1* b2 - a4* b3 + a3* b4) - (a3* b1 + a4* b2 + a1* b3 -a2* b4)*(a1* b1 - a2* b2 - a3* b3 -a4* b4)) + (-(-a4* b1 + a3* b2 - a2* b3 - a1* b4)*(a3* b1 +a4* b2 + a1* b3 - a2* b4) + (a2* b1 + a1* b2 - a4* b3 +a3* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((-a2* b1 -a1* b2 + a4* b3 - a3* b4)*(a2* b1 + a1* b2 - a4* b3 +a3* b4) - (a1* b1 - a2* b2 - a3* b3 - a4* b4)**2))*(-((a3* b1 + a4* b2 + a1* b3 - a2* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (-a4* b1 + a3* b2 - a2* b3 - a1* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((a4* b1 -a3* b2 + a2* b3 + a1* b4)*(a2* b1 + a1* b2 - a4* b3 +a3* b4) - (-a3* b1 - a4* b2 - a1* b3 + a2* b4)*(a1* b1 -a2* b2 - a3* b3 -a4* b4)) + (-(-a4* b1 + a3* b2 - a2* b3 - a1* b4)*(-a3* b1 -a4* b2 - a1* b3 + a2* b4) + (a2* b1 + a1* b2 - a4* b3 +a3* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((-a2* b1 -a1* b2 + a4* b3 - a3* b4)*(a2* b1 + a1* b2 - a4* b3 +a3* b4) - (a1* b1 - a2* b2 - a3* b3 - a4* b4)**2))))
            
    # x02 = -((((-a2* b1 - a1* b2 + a4* b3 -a3* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (a1* b1 - a2* b2 -a3* b3 - a4* b4)**2)*(-(a2* b1 + a1* b2 - a4* b3 + a3* b4)* c1 + (-a4* b1 + a3* b2 - a2* b3 - a1* b4)* c3) - ((-a3* b1 - a4* b2 - a1* b3 +a2* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (-a4* b1 +a3* b2 - a2* b3 - a1* b4)*(a1* b1 - a2* b2 - a3* b3 -a4* b4))*((a1* b1 - a2* b2 - a3* b3 - a4* b4)* c3 - (a2* b1 +a1* b2 - a4* b3 +a3* b4)* c4))/(-((a3* b1 + a4* b2 + a1* b3 - a2* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (-a4* b1 + a3* b2 - a2* b3 - a1* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((-a3* b1 -a4* b2 - a1* b3 + a2* b4)*(a2* b1 + a1* b2 - a4* b3 +a3* b4) - (-a4* b1 + a3* b2 - a2* b3 - a1* b4)*(a1* b1 -a2* b2 - a3* b3 - a4* b4)) + (-(-a4* b1 + a3* b2 - a2* b3 -a1* b4)**2 + (-a2* b1 - a1* b2 + a4* b3 - a3* b4)*(a2* b1 +a1* b2 - a4* b3 + a3* b4))*((-a2* b1 - a1* b2 + a4* b3 -a3* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (a1* b1 - a2* b2 -a3* b3 - a4* b4)**2))) + ((-((-a3* b1 - a4* b2 - a1* b3 + a2* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (-a4* b1 + a3* b2 - a2* b3 - a1* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((a4* b1 - a3* b2 + a2* b3 + a1* b4)*(a2* b1 +a1* b2 - a4* b3 + a3* b4) - (a3* b1 + a4* b2 + a1* b3 -a2* b4)*(a1* b1 - a2* b2 - a3* b3 -a4* b4)) + (-(-a4* b1 + a3* b2 - a2* b3 - a1* b4)*(a3* b1 +a4* b2 + a1* b3 - a2* b4) + (a2* b1 + a1* b2 - a4* b3 +a3* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((-a2* b1 -a1* b2 + a4* b3 - a3* b4)*(a2* b1 + a1* b2 - a4* b3 +a3* b4) - (a1* b1 - a2* b2 - a3* b3 - a4* b4)**2))*(-(-((a3* b1 + a4* b2 + a1* b3 - a2* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (-a4* b1 + a3* b2 - a2* b3 - a1* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((a4* b1 -a3* b2 + a2* b3 + a1* b4)*(a2* b1 + a1* b2 - a4* b3 +a3* b4) - (-a3* b1 - a4* b2 - a1* b3 + a2* b4)*(a1* b1 -a2* b2 - a3* b3 -a4* b4)) + (-(-a4* b1 + a3* b2 - a2* b3 - a1* b4)*(-a3* b1 - a4* b2 - a1* b3 + a2* b4) + (a2* b1 +a1* b2 - a4* b3 + a3* b4)*(a1* b1 - a2* b2 - a3* b3 -a4* b4))*((-a2* b1 - a1* b2 + a4* b3 - a3* b4)*(a2* b1 +a1* b2 - a4* b3 + a3* b4) - (a1* b1 - a2* b2 - a3* b3 - a4* b4)**2))*(((-a2* b1 - a1* b2 + a4* b3 - a3* b4)*(a2* b1 +a1* b2 - a4* b3 + a3* b4) - (a1* b1 - a2* b2 - a3* b3 -a4* b4)**2)*(-(a2* b1 + a1* b2 - a4* b3 +a3* b4)* c1 + (-a4* b1 + a3* b2 - a2* b3 - a1* b4)* c3) - ((-a3* b1 - a4* b2 - a1* b3 + a2* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (-a4* b1 + a3* b2 - a2* b3 - a1* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((a1* b1 - a2* b2 - a3* b3 - a4* b4)* c3 - (a2* b1 +a1* b2 - a4* b3 + a3* b4)* c4)) + (-((a3* b1 + a4* b2 + a1* b3 -a2* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (-a4* b1 +a3* b2 - a2* b3 - a1* b4)*(a1* b1 - a2* b2 - a3* b3 -a4* b4))*((-a3* b1 - a4* b2 - a1* b3 + a2* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (-a4* b1 + a3* b2 - a2* b3 - a1* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4)) + (-(-a4* b1 + a3* b2 - a2* b3 - a1* b4)**2 + (-a2* b1 - a1* b2 + a4* b3 - a3* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4))*((-a2* b1 - a1* b2 + a4* b3 - a3* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (a1* b1 - a2* b2 - a3* b3 -a4* b4)**2))*(((-a2* b1 - a1* b2 + a4* b3 - a3* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (a1* b1 - a2* b2 - a3* b3 -a4* b4)**2)*(-(a2* b1 + a1* b2 - a4* b3 +a3* b4)* c2 + (-a3* b1 - a4* b2 - a1* b3 + a2* b4)* c3) - ((a4* b1 - a3* b2 + a2* b3 + a1* b4)*(a2* b1 +a1* b2 - a4* b3 + a3* b4) - (-a3* b1 - a4* b2 - a1* b3 + a2* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((a1* b1 - a2* b2 - a3* b3 - a4* b4)* c3 - (a2* b1 + a1* b2 - a4* b3 + a3* b4)* c4))))/((-((a3* b1 + a4* b2 + a1* b3 - a2* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (-a4* b1 + a3* b2 - a2* b3 - a1* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((-a3* b1 - a4* b2 - a1* b3 + a2* b4)*(a2* b1 +a1* b2 - a4* b3 + a3* b4) - (-a4* b1 + a3* b2 - a2* b3 -a1* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4)) + (-(-a4* b1 +a3* b2 - a2* b3 - a1* b4)**2 + (-a2* b1 - a1* b2 + a4* b3 -a3* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4))*((-a2* b1 -a1* b2 + a4* b3 - a3* b4)*(a2* b1 + a1* b2 - a4* b3 +a3* b4) - (a1* b1 - a2* b2 - a3* b3 - a4* b4)**2))*((-((a3* b1 + a4* b2 + a1* b3 - a2* b4)*(a2* b1 +a1* b2 - a4* b3 + a3* b4) - (-a4* b1 + a3* b2 - a2* b3 -a1* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((-a3* b1 - a4* b2 - a1* b3 + a2* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (-a4* b1 + a3* b2 - a2* b3 - a1* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4)) + (-(-a4* b1 + a3* b2 - a2* b3 - a1* b4)**2 + (-a2* b1 - a1* b2 + a4* b3 - a3* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4))*((-a2* b1 - a1* b2 + a4* b3 - a3* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (a1* b1 -a2* b2 - a3* b3 -a4* b4)**2))*(-((a4* b1 - a3* b2 + a2* b3 + a1* b4)*(a2* b1 +a1* b2 - a4* b3 + a3* b4) - (a3* b1 + a4* b2 + a1* b3 -a2* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((a4* b1 - a3* b2 + a2* b3 + a1* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (-a3* b1 - a4* b2 - a1* b3 + a2* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4)) + (-(a3* b1 + a4* b2 + a1* b3 - a2* b4)*(-a3* b1 - a4* b2 - a1* b3 + a2* b4) + (a2* b1 + a1* b2 - a4* b3 +a3* b4)**2)*((-a2* b1 - a1* b2 + a4* b3 - a3* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (a1* b1 - a2* b2 - a3* b3 -a4* b4)**2)) - (-((-a3* b1 - a4* b2 - a1* b3 +a2* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (-a4* b1 +a3* b2 - a2* b3 - a1* b4)*(a1* b1 - a2* b2 - a3* b3 -a4* b4))*((a4* b1 - a3* b2 + a2* b3 + a1* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (a3* b1 + a4* b2 + a1* b3 - a2* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4)) + (-(-a4* b1 + a3* b2 - a2* b3 - a1* b4)*(a3* b1 + a4* b2 + a1* b3 - a2* b4) + (a2* b1 + a1* b2 - a4* b3 + a3* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((-a2* b1 - a1* b2 + a4* b3 - a3* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (a1* b1 - a2* b2 - a3* b3 -a4* b4)**2))*(-((a3* b1 + a4* b2 + a1* b3 - a2* b4)*(a2* b1 +a1* b2 - a4* b3 + a3* b4) - (-a4* b1 + a3* b2 - a2* b3 -a1* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((a4* b1 - a3* b2 + a2* b3 + a1* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (-a3* b1 - a4* b2 - a1* b3 + a2* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4)) + (-(-a4* b1 + a3* b2 - a2* b3 - a1* b4)*(-a3* b1 -a4* b2 - a1* b3 + a2* b4) + (a2* b1 + a1* b2 - a4* b3 + a3* b4)*(a1* b1 - a2* b2 - a3* b3 - a4* b4))*((-a2* b1 - a1* b2 + a4* b3 - a3* b4)*(a2* b1 + a1* b2 - a4* b3 + a3* b4) - (a1* b1 - a2* b2 - a3* b3 -a4* b4)**2))))
            
    # x03 = -((a3* b1* c1 + a4* b2* c1 + a1* b3* c1 - a2* b4* c1 - a4* b1* c2 + a3* b2* c2 - a2* b3* c2 - a1* b4* c2 - a1* b1* c3 + a2* b2* c3 + a3* b3* c3 + a4* b4* c3 + a2* b1* c4 + a1* b2* c4 - a4* b3* c4 + a3* b4* c4)/((a1**2 + a2**2 + a3**2 + a4**2)*(b1**2 + b2**2 + b3**2 + b4**2)))
    
    # x04 = -((a4* b1* c1 - a3* b2* c1 + a2* b3* c1 + a1* b4* c1 + a3* b1* c2 + a4* b2* c2 + a1* b3* c2 - a2* b4* c2 - a2* b1* c3 - a1* b2* c3 + a4* b3* c3 - a3* b4* c3 - a1* b1* c4 + a2* b2* c4 + a3* b3* c4 + a4* b4* c4)/((a1**2 + a2**2 + a3**2 + a4**2)*(b1**2 + b2**2 + b3**2 + b4**2)))

    x01 = (a1*b1-a2*b2-a3*b3-a4*b4)*c1-(a2*b1+a1*b2-a4*b3+a3*b4)*c2-(a3*b1+a4*b2+a1*b3-a2*b4)*c3-(a4*b1-a3*b2+a2*b3+a1*b4)*c4
        
    x02 = (a2*b1+a1*b2-a4*b3+a3*b4)*c1+(a1*b1-a2*b2-a3*b3-a4*b4)*c2-(a4*b1-a3*b2+a2*b3+a1*b4)*c3+(a3*b1+a4*b2+a1*b3-a2*b4)*c4
        
    x03 = (a3*b1+a4*b2+a1*b3-a2*b4)*c1+(a4*b1-a3*b2+a2*b3+a1*b4)*c2+(a1*b1-a2*b2-a3*b3-a4*b4)*c3-(a2*b1+a1*b2-a4*b3+a3*b4)*c4
        
    x04 = (a4*b1-a3*b2+a2*b3+a1*b4)*c1-(a3*b1+a4*b2+a1*b3-a2*b4)*c2+(a2*b1+a1*b2-a4*b3+a3*b4)*c3+(a1*b1-a2*b2-a3*b3-a4*b4)*c4

    qtr_x = np.array([x01,x02,x03,x04])
    return qtr_x

print(three_qtr_solve([0.95,0.,0.25,0.],[0.3,0.2,0.25,0.2],[0.9,0.05,0.25,0.]))

# import math

# class QtrCalc:
#     def Two_Vector_Solve():







#         return 0
    
#     def Three_Vector_Solve(qtr_a,qtr_b,qtr_c,qtr_x):

#         x01 = -((-(-((a3 b1 + a4 b2 + a1 b3 - a2 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (-a4 b1 + a3 b2 - a2 b3 - a1 b4) (a1 b1 -a2 b2 - a3 b3 - a4 b4)) ((a4 b1 - a3 b2 + a2 b3 + a1 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (-a3 b1 - a4 b2 - a1 b3 + a2 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) + (-(-a4 b1 + a3 b2 - a2 b3 - a1 b4) (-a3 b1 -a4 b2 - a1 b3 + a2 b4) + (a2 b1 + a1 b2 - a4 b3 + a3 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((-a2 b1 - a1 b2 + a4 b3 - a3 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (a1 b1 - a2 b2 - a3 b3 -a4 b4)^2)) (((-a2 b1 - a1 b2 + a4 b3 - a3 b4) (a2 b1 +a1 b2 - a4 b3 + a3 b4) - (a1 b1 - a2 b2 - a3 b3 - a4 b4)^2) (-(a2 b1 + a1 b2 - a4 b3 + a3 b4) c1 + (-a4 b1 + a3 b2 - a2 b3 -a1 b4) c3) - ((-a3 b1 - a4 b2 - a1 b3 + a2 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (-a4 b1 + a3 b2 - a2 b3 -a1 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((a1 b1 -a2 b2 - a3 b3 - a4 b4) c3 - (a2 b1 + a1 b2 - a4 b3 +a3 b4) c4)) + (-((a3 b1 + a4 b2 + a1 b3 - a2 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (-a4 b1 + a3 b2 - a2 b3 - a1 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((-a3 b1 - a4 b2 - a1 b3 + a2 b4) (a2 b1 +a1 b2 - a4 b3 + a3 b4) - (-a4 b1 + a3 b2 - a2 b3 -a1 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) + (-(-a4 b1 +a3 b2 - a2 b3 - a1 b4)^2 + (-a2 b1 - a1 b2 + a4 b3 -a3 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4)) ((-a2 b1 -a1 b2 + a4 b3 - a3 b4) (a2 b1 + a1 b2 - a4 b3 +a3 b4) - (a1 b1 - a2 b2 - a3 b3 - a4 b4)^2)) (((-a2 b1 - a1 b2 + a4 b3 - a3 b4) (a2 b1 +a1 b2 - a4 b3 + a3 b4) - (a1 b1 - a2 b2 - a3 b3 - a4 b4)^2) (-(a2 b1 + a1 b2 - a4 b3 + a3 b4) c2 + (-a3 b1 - a4 b2 - a1 b3 +a2 b4) c3) - ((a4 b1 - a3 b2 + a2 b3 + a1 b4) (a2 b1 +a1 b2 - a4 b3 + a3 b4) - (-a3 b1 - a4 b2 - a1 b3 +a2 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((a1 b1 -a2 b2 - a3 b3 - a4 b4) c3 - (a2 b1 + a1 b2 - a4 b3 +a3 b4) c4)))/((-((a3 b1 + a4 b2 + a1 b3 - a2 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (-a4 b1 + a3 b2 - a2 b3 - a1 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((-a3 b1 - a4 b2 - a1 b3 + a2 b4) (a2 b1 +a1 b2 - a4 b3 + a3 b4) - (-a4 b1 + a3 b2 - a2 b3 -a1 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) + (-(-a4 b1 +a3 b2 - a2 b3 - a1 b4)^2 + (-a2 b1 - a1 b2 + a4 b3 -a3 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4)) ((-a2 b1 -a1 b2 + a4 b3 - a3 b4) (a2 b1 + a1 b2 - a4 b3 +a3 b4) - (a1 b1 - a2 b2 - a3 b3 - a4 b4)^2)) (-((a4 b1 - a3 b2 + a2 b3 + a1 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (a3 b1 + a4 b2 + a1 b3 - a2 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((a4 b1 -a3 b2 + a2 b3 + a1 b4) (a2 b1 + a1 b2 - a4 b3 +a3 b4) - (-a3 b1 - a4 b2 - a1 b3 + a2 b4) (a1 b1 -a2 b2 - a3 b3 -a4 b4)) + (-(a3 b1 + a4 b2 + a1 b3 - a2 b4) (-a3 b1 -a4 b2 - a1 b3 + a2 b4) + (a2 b1 + a1 b2 - a4 b3 + a3 b4)^2) ((-a2 b1 - a1 b2 + a4 b3 - a3 b4) (a2 b1 +a1 b2 - a4 b3 + a3 b4) - (a1 b1 - a2 b2 - a3 b3 - a4 b4)^2)) - (-((-a3 b1 - a4 b2 - a1 b3 + a2 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (-a4 b1 + a3 b2 - a2 b3 - a1 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((a4 b1 - a3 b2 + a2 b3 + a1 b4) (a2 b1 +a1 b2 - a4 b3 + a3 b4) - (a3 b1 + a4 b2 + a1 b3 -a2 b4) (a1 b1 - a2 b2 - a3 b3 -a4 b4)) + (-(-a4 b1 + a3 b2 - a2 b3 - a1 b4) (a3 b1 +a4 b2 + a1 b3 - a2 b4) + (a2 b1 + a1 b2 - a4 b3 +a3 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((-a2 b1 -a1 b2 + a4 b3 - a3 b4) (a2 b1 + a1 b2 - a4 b3 +a3 b4) - (a1 b1 - a2 b2 - a3 b3 - a4 b4)^2)) (-((a3 b1 + a4 b2 + a1 b3 - a2 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (-a4 b1 + a3 b2 - a2 b3 - a1 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((a4 b1 -a3 b2 + a2 b3 + a1 b4) (a2 b1 + a1 b2 - a4 b3 +a3 b4) - (-a3 b1 - a4 b2 - a1 b3 + a2 b4) (a1 b1 -a2 b2 - a3 b3 -a4 b4)) + (-(-a4 b1 + a3 b2 - a2 b3 - a1 b4) (-a3 b1 -a4 b2 - a1 b3 + a2 b4) + (a2 b1 + a1 b2 - a4 b3 +a3 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((-a2 b1 -a1 b2 + a4 b3 - a3 b4) (a2 b1 + a1 b2 - a4 b3 +a3 b4) - (a1 b1 - a2 b2 - a3 b3 - a4 b4)^2))))
              
#         x02 = -((((-a2 b1 - a1 b2 + a4 b3 -a3 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (a1 b1 - a2 b2 -a3 b3 - a4 b4)^2) (-(a2 b1 + a1 b2 - a4 b3 + a3 b4) c1 + (-a4 b1 + a3 b2 - a2 b3 - a1 b4) c3) - ((-a3 b1 - a4 b2 - a1 b3 +a2 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (-a4 b1 +a3 b2 - a2 b3 - a1 b4) (a1 b1 - a2 b2 - a3 b3 -a4 b4)) ((a1 b1 - a2 b2 - a3 b3 - a4 b4) c3 - (a2 b1 +a1 b2 - a4 b3 +a3 b4) c4))/(-((a3 b1 + a4 b2 + a1 b3 - a2 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (-a4 b1 + a3 b2 - a2 b3 - a1 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((-a3 b1 -a4 b2 - a1 b3 + a2 b4) (a2 b1 + a1 b2 - a4 b3 +a3 b4) - (-a4 b1 + a3 b2 - a2 b3 - a1 b4) (a1 b1 -a2 b2 - a3 b3 - a4 b4)) + (-(-a4 b1 + a3 b2 - a2 b3 -a1 b4)^2 + (-a2 b1 - a1 b2 + a4 b3 - a3 b4) (a2 b1 +a1 b2 - a4 b3 + a3 b4)) ((-a2 b1 - a1 b2 + a4 b3 -a3 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (a1 b1 - a2 b2 -a3 b3 - a4 b4)^2))) + ((-((-a3 b1 - a4 b2 - a1 b3 + a2 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (-a4 b1 + a3 b2 - a2 b3 - a1 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((a4 b1 - a3 b2 + a2 b3 + a1 b4) (a2 b1 +a1 b2 - a4 b3 + a3 b4) - (a3 b1 + a4 b2 + a1 b3 -a2 b4) (a1 b1 - a2 b2 - a3 b3 -a4 b4)) + (-(-a4 b1 + a3 b2 - a2 b3 - a1 b4) (a3 b1 +a4 b2 + a1 b3 - a2 b4) + (a2 b1 + a1 b2 - a4 b3 +a3 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((-a2 b1 -a1 b2 + a4 b3 - a3 b4) (a2 b1 + a1 b2 - a4 b3 +a3 b4) - (a1 b1 - a2 b2 - a3 b3 - a4 b4)^2)) (-(-((a3 b1 + a4 b2 + a1 b3 - a2 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (-a4 b1 + a3 b2 - a2 b3 - a1 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((a4 b1 -a3 b2 + a2 b3 + a1 b4) (a2 b1 + a1 b2 - a4 b3 +a3 b4) - (-a3 b1 - a4 b2 - a1 b3 + a2 b4) (a1 b1 -a2 b2 - a3 b3 -a4 b4)) + (-(-a4 b1 + a3 b2 - a2 b3 - a1 b4) (-a3 b1 - a4 b2 - a1 b3 + a2 b4) + (a2 b1 +a1 b2 - a4 b3 + a3 b4) (a1 b1 - a2 b2 - a3 b3 -a4 b4)) ((-a2 b1 - a1 b2 + a4 b3 - a3 b4) (a2 b1 +a1 b2 - a4 b3 + a3 b4) - (a1 b1 - a2 b2 - a3 b3 - a4 b4)^2)) (((-a2 b1 - a1 b2 + a4 b3 - a3 b4) (a2 b1 +a1 b2 - a4 b3 + a3 b4) - (a1 b1 - a2 b2 - a3 b3 -a4 b4)^2) (-(a2 b1 + a1 b2 - a4 b3 +a3 b4) c1 + (-a4 b1 + a3 b2 - a2 b3 - a1 b4) c3) - ((-a3 b1 - a4 b2 - a1 b3 + a2 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (-a4 b1 + a3 b2 - a2 b3 - a1 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((a1 b1 - a2 b2 - a3 b3 - a4 b4) c3 - (a2 b1 +a1 b2 - a4 b3 + a3 b4) c4)) + (-((a3 b1 + a4 b2 + a1 b3 -a2 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (-a4 b1 +a3 b2 - a2 b3 - a1 b4) (a1 b1 - a2 b2 - a3 b3 -a4 b4)) ((-a3 b1 - a4 b2 - a1 b3 + a2 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (-a4 b1 + a3 b2 - a2 b3 - a1 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) + (-(-a4 b1 + a3 b2 - a2 b3 - a1 b4)^2 + (-a2 b1 - a1 b2 + a4 b3 - a3 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4)) ((-a2 b1 - a1 b2 + a4 b3 - a3 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (a1 b1 - a2 b2 - a3 b3 -a4 b4)^2)) (((-a2 b1 - a1 b2 + a4 b3 - a3 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (a1 b1 - a2 b2 - a3 b3 -a4 b4)^2) (-(a2 b1 + a1 b2 - a4 b3 +a3 b4) c2 + (-a3 b1 - a4 b2 - a1 b3 + a2 b4) c3) - ((a4 b1 - a3 b2 + a2 b3 + a1 b4) (a2 b1 +a1 b2 - a4 b3 + a3 b4) - (-a3 b1 - a4 b2 - a1 b3 + a2 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((a1 b1 - a2 b2 - a3 b3 - a4 b4) c3 - (a2 b1 + a1 b2 - a4 b3 + a3 b4) c4))))/((-((a3 b1 + a4 b2 + a1 b3 - a2 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (-a4 b1 + a3 b2 - a2 b3 - a1 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((-a3 b1 - a4 b2 - a1 b3 + a2 b4) (a2 b1 +a1 b2 - a4 b3 + a3 b4) - (-a4 b1 + a3 b2 - a2 b3 -a1 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) + (-(-a4 b1 +a3 b2 - a2 b3 - a1 b4)^2 + (-a2 b1 - a1 b2 + a4 b3 -a3 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4)) ((-a2 b1 -a1 b2 + a4 b3 - a3 b4) (a2 b1 + a1 b2 - a4 b3 +a3 b4) - (a1 b1 - a2 b2 - a3 b3 - a4 b4)^2)) ((-((a3 b1 + a4 b2 + a1 b3 - a2 b4) (a2 b1 +a1 b2 - a4 b3 + a3 b4) - (-a4 b1 + a3 b2 - a2 b3 -a1 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((-a3 b1 - a4 b2 - a1 b3 + a2 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (-a4 b1 + a3 b2 - a2 b3 - a1 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) + (-(-a4 b1 + a3 b2 - a2 b3 - a1 b4)^2 + (-a2 b1 - a1 b2 + a4 b3 - a3 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4)) ((-a2 b1 - a1 b2 + a4 b3 - a3 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (a1 b1 -a2 b2 - a3 b3 -a4 b4)^2)) (-((a4 b1 - a3 b2 + a2 b3 + a1 b4) (a2 b1 +a1 b2 - a4 b3 + a3 b4) - (a3 b1 + a4 b2 + a1 b3 -a2 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((a4 b1 - a3 b2 + a2 b3 + a1 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (-a3 b1 - a4 b2 - a1 b3 + a2 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) + (-(a3 b1 + a4 b2 + a1 b3 - a2 b4) (-a3 b1 - a4 b2 - a1 b3 + a2 b4) + (a2 b1 + a1 b2 - a4 b3 +a3 b4)^2) ((-a2 b1 - a1 b2 + a4 b3 - a3 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (a1 b1 - a2 b2 - a3 b3 -a4 b4)^2)) - (-((-a3 b1 - a4 b2 - a1 b3 +a2 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (-a4 b1 +a3 b2 - a2 b3 - a1 b4) (a1 b1 - a2 b2 - a3 b3 -a4 b4)) ((a4 b1 - a3 b2 + a2 b3 + a1 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (a3 b1 + a4 b2 + a1 b3 - a2 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) + (-(-a4 b1 + a3 b2 - a2 b3 - a1 b4) (a3 b1 + a4 b2 + a1 b3 - a2 b4) + (a2 b1 + a1 b2 - a4 b3 + a3 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((-a2 b1 - a1 b2 + a4 b3 - a3 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (a1 b1 - a2 b2 - a3 b3 -a4 b4)^2)) (-((a3 b1 + a4 b2 + a1 b3 - a2 b4) (a2 b1 +a1 b2 - a4 b3 + a3 b4) - (-a4 b1 + a3 b2 - a2 b3 -a1 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((a4 b1 - a3 b2 + a2 b3 + a1 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (-a3 b1 - a4 b2 - a1 b3 + a2 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) + (-(-a4 b1 + a3 b2 - a2 b3 - a1 b4) (-a3 b1 -a4 b2 - a1 b3 + a2 b4) + (a2 b1 + a1 b2 - a4 b3 + a3 b4) (a1 b1 - a2 b2 - a3 b3 - a4 b4)) ((-a2 b1 - a1 b2 + a4 b3 - a3 b4) (a2 b1 + a1 b2 - a4 b3 + a3 b4) - (a1 b1 - a2 b2 - a3 b3 -a4 b4)^2))))
               
#         x03 = -((a3 b1 c1 + a4 b2 c1 + a1 b3 c1 - a2 b4 c1 - a4 b1 c2 + a3 b2 c2 - a2 b3 c2 - a1 b4 c2 - a1 b1 c3 + a2 b2 c3 + a3 b3 c3 + a4 b4 c3 + a2 b1 c4 + a1 b2 c4 - a4 b3 c4 + a3 b4 c4)/((a1^2 + a2^2 + a3^2 + a4^2) (b1^2 + b2^2 + b3^2 + b4^2)))
        
#         x04 = -((a4 b1 c1 - a3 b2 c1 + a2 b3 c1 + a1 b4 c1 + a3 b1 c2 + a4 b2 c2 + a1 b3 c2 - a2 b4 c2 - a2 b1 c3 - a1 b2 c3 + a4 b3 c3 - a3 b4 c3 - a1 b1 c4 + a2 b2 c4 + a3 b3 c4 + a4 b4 c4)/((a1^2 + a2^2 + a3^2 + a4^2) (b1^2 + b2^2 + b3^2 + b4^2)))

#         return {x01, x02, x03, x04}