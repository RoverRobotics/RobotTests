import numpy
import socket
import threading
import Queue
import time
class DataCollector:
    def __init__(self,ip,port) :
        self.ip_address = ip
        self.port = port
        self.data = None # return numpy array of data (N, data)
        self.left_data = Queue.Queue()
        self.right_data = Queue.Queue()
        self.buf = list()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        pass
    def connect(self):
        return self.socket.connect((self.ip_address,self.port))
    def start(self):
        '''connect the microcontroller and start data collection'''
        self.connection_error = self.connect()
        self.socket.send("s")
        self.thread_should_run = True
        self.thread = threading.Thread(target=self.parse_).start()


    def parse_(self):
        
        while self.connection_error is None:
            # self.buf.append(int(self.socket.recv(8).encode('hex'),16))
            data = self.socket.recv(256)
            data = [int(d.encode('hex'),16)for d in data]
            self.buf += data
            while len(self.buf) >= 8:
                if self.buf[0:4] == [255, 0, 0, 0]:
                    #print(256*self.buf[4] + self.buf[5])
                    self.left_data.put(256*self.buf[4] + self.buf[5])
                    self.right_data.put(256*self.buf[6] + self.buf[7])
                    del self.buf[0:8]
                else:
                    del self.buf[0]


    def stop(self):
        '''stop data collection'''
        # self.thread = None
        self.socket.send("f")
        time.sleep(5)

        # self.socket.close()
        temp1 = list()
        temp2 = list()
        while not self.left_data.empty():
            temp1.append(self.left_data.get_nowait())
        while not self.right_data.empty():
            temp2.append(self.right_data.get_nowait())
        self.data = numpy.array([temp1,temp2])
            # self.data = numpy.array([self.left_data,self.right_data.get()])
        pass
    def save_to_file(self,path):
        '''save collected data for later processing or inspection'''
        if len(self.data) == 0:
            print("Data is empty")
            pass
        else:
            with open(path, 'w') as f:
                for points in range(len(self.data[0])):
                    f.write(str(self.data[0][points]))
                    if points != (len(self.data[0]) - 1):
                        f.write(",")
                f.write("\r\n")
                for points in range(len(self.data[1])):
                    f.write(str(self.data[1][points]))
                    if points != (len(self.data[0]) - 1):
                        f.write(",")
                f.close()
                pass
    def load_from_file(self,path):
        '''load saved data'''
        pass
    def get_data(self):
        return self.data
