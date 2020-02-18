import socket

SO_BIND = 0
SO_CONNECT = 1


def init_socket(port: int, flag, host=''):

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    if flag == SO_BIND:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.bind((host, port))
    elif flag == SO_CONNECT:
        s.connect((host, port))
    else:
        # TODO: add exception 
        return s

    return s
