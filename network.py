#network procedures module

import socket
from PyQt5 import QtCore
import time

SO_BIND = 0
SO_CONNECT = 1
SO_BROADCAST = 2


def init_socket(port: int, flag, host='', timeout=10):

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    s.settimeout(timeout)

    if flag == SO_BIND:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.bind((host, port))
    elif flag == SO_CONNECT:
        s.connect((host, port))
    elif flag == SO_BROADCAST:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.bind((host, port))
    else:
        # TODO: add exception 
        return s

    return s

class Broadcaster(QtCore.QObject):
    """
    Must derive from QObject in order to emit signals, connect slots to other signals, and operate in a QThread.
    """

    sig_status = QtCore.pyqtSignal(int, int, str)  # broadcaster id: emitted status()
    sig_msg = QtCore.pyqtSignal(str)  # message to be shown to user
    sig_ports = QtCore.pyqtSignal(int)

    def __init__(self, port=5550):
        super().__init__()
        self.port = port   
        self.__abort = False
        self.sckt = init_socket(self.port, SO_BROADCAST)
        #self.sckt_bc = init_socket(self.port, SO_BROADCAST)
        #self.sckt_in = init_socket(self.port + 1, SO_BIND)
        self.msg = "BNO Tracker broadcasting"
        self.dest = ('<broadcast>', self.port)
        # TODO: print broadcasting messages when no devices connected
        self.found_devices = False

    def work(self):

        #self.sig_status.emit(1, 5555, '192.168.1.108')
        self.sckt.sendto(self.msg.encode(), self.dest)
        self.sig_msg.emit("Message sent: " + self.msg)

        while (self.__abort == False):
            try:
                msg_device, address = self.sckt.recvfrom(8192)
                # print(msg_device.decode())
                if msg_device.decode()[:len(self.msg)] == self.msg:
                    port = int(msg_device.decode().split('#')[1]) if '#' in msg_device.decode() else 0000
                    if port != 0000:
                        self.found_devices = True
                        self.sig_msg.emit("Connected to " + address[0] + ":" + str(port))
                        self.sig_status.emit(1, port, address[0])
                        self.sig_ports.emit(port)
            except socket.timeout:
                    self.sckt.sendto(self.msg.encode(), self.dest)
                    if not self.found_devices:
                        self.sig_msg.emit("Broadcasting devices...")
            except Exception as e:
                print("Unexpected error: ", e)
                pass
        

    def abort(self):
        print(1)
        # self.sig_msg.emit('Broadcaster notified to abort')
        self.__abort = True
        
    
    def __del__(self):

        self.sckt.shutdown(0)
        self.sckt.close()
        print('Worker sockets closed')
