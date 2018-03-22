################################## 
## UDP Server to Talk to UE4 
################################## 
# UE4UDPLogger.py 
################################## 
### Imports 
import socket 
import sys 
import numpy as np 
from time import sleep 
from collections import deque 
from matplotlib import pyplot as plt 

### Constants 
HOST = '' 
PORT = 12345 
BUFFER_LENGTH = 1000

# class that holds analog data for N samples 
class UdpData: 
    # constr 
    def __init__(self, maxLen): 
        self.ax = deque([0.0]*maxLen) 
        self.ay = deque([0.0]*maxLen) 
        self.maxLen = maxLen 

    # ring buffer 
    def addToBuf(self, buf, val): 
        if len(buf) < self.maxLen: 
            buf.append(val) 
        else: 
            buf.popleft() 
            buf.append(val) 

    # add data 
    def add(self, data): 
        assert(len(data) == 2) 
        self.addToBuf(self.ax, data[0]) 
        self.addToBuf(self.ay, data[1]) 
    
# plot class 
class UdpPlot: 
    # constr 
    def __init__(self, udpData): 
        plt.ion() 
        self.line1, = plt.plot(udpData.ax, udpData.ay) 
        plt.ylim(-1, 1)
        plt.xlim(-BUFFER_LENGTH,0) 

    # update plot 
    def update(self, udpData): 
        self.line1.set_xdata(udpData.ax) 
        self.line1.set_ydata(udpData.ay) 
        ymin = min(udpData.ay)
        ymax = max(udpData.ay)
        if (ymax > ymin):
            plt.ylim(ymin, ymax) 
        xmin = min(udpData.ax)
        xmax = max(udpData.ax)
        if (xmax > xmin):
            plt.xlim(xmin, xmax) 
        plt.draw() 
        plt.pause(0.001)

### Variables 
udpData = UdpData(BUFFER_LENGTH) 
udpPlot = UdpPlot(udpData) 


# Create the UDP Socket 
try: 
    socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
    #socket.setblocking(false) 
    print("Socket Created") 
except: 
    print("Failed to create socket.") 
    sys.exit() 

# Bind socket to local host and port 
try: 
    socket.bind((HOST, PORT)) 
except: 
    print("Bind failed.") 
    sys.exit() 
    
print("Socket bind complete.") 

# Now keep talking to clients 

while 1: 

    try:
        # Receive data from client (data, addr) 
        d = socket.recvfrom(1024) 
        data = str(d[0], "utf-8") 
        addr = d[1] 

        # Print to the server who made a connection. 
        #print("{} wrote:".format(addr)) 
        #print(data) 

        # Now have our UDP handler handle the data 
        values = [float(val) for val in data.split(",")] 
        #print (values) 
        if(len(values) == 2): 
            udpData.add(values) 
            udpPlot.update(udpData) 

    except KeyboardInterrupt: 
        print ('exiting') 
        break 

    # Respond back 
    #print(myResponse) 
    #socket.sendto(bytes(myResponse, 'utf-8'), addr) 
    
socket.close()