import socket


UDP_IP = "192.168.0.12"; #socket.gethostbyname(socket.gethostname())
print "Receiver IP: ", UDP_IP

UDP_PORT = 12345; #int(raw_input ("Enter Port "))
print "Port: ", UDP_PORT   
sock = socket.socket(socket.AF_INET, # Internet
                    socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))


while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    #print "Data ", data;
    mylist = data.split(",");
    #print mylist
    #print mylist[3],":",mylist[4],":",mylist[5]
    oZ = int(float(mylist[3]));
    oX = int(float(mylist[4]));
    oY = int(float(mylist[5]));
    print "Z:",oZ,"  X:",oX,"  Y:",oY
