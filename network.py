import socket

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
    elif: flag == SO_BROADCAST
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    else:
        # TODO: add exception 
        return s

    return s

class Broadcaster(QObject):
    """
    Must derive from QObject in order to emit signals, connect slots to other signals, and operate in a QThread.
    """

    sig_status = pyqtSignal(int, int)  # broadcaster id: emitted status()
    sig_msg = pyqtSignal(str)  # message to be shown to user

    def __init__(self, port=5550):
        super().__init__()
        self.port = port   
        self.__abort = False
        self.sckt_bc = init_socket(self.port, SO_BROADCAST)
        self.sckt_in = init_socket(self.port, SO_BIND)
        self.msg = socket.gethostbyname(socket.gethostname())
        self.dest = ('<broadcast>', self.port)

    def work(self):

        while 1:
            self.sckt.sendto(msg, dest)
            try:

            time.sleep(1)

    def abort(self):
        self.sig_msg.emit('Broadcaster notified to abort')
        self.__abort = True
