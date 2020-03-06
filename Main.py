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

import MainWindow
import Worker
import Draw


App = QApplication(sys.argv)
window = MainWindow.MainWindow()
sys.exit(App.exec())

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