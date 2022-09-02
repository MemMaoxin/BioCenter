import socket
if __name__ == "__main__":
    BUFSIZE = 1024
    ip_port = ('0.0.0.0', 8765)
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # udp协议
    server.bind(ip_port)
    count = 0
    while True:
        data, client_addr = server.recvfrom(BUFSIZE)
        print(f'receive({count}):', client_addr, data)
        count+=1
        # server.sendto(data.upper(), client_addr)
    server.close()