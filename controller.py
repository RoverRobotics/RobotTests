#data controller
#in charge of using the correct classes to generate data
from data_processor import DataProcessor
from data_collector import DataCollector
from movement_unit_tests import MovementManager
import matplotlib.pyplot as plt
import numpy as np
import time

class Controller:
    def __init__(self):
        self.collector = DataCollector("192.168.1.99", 80)
        self.processor = DataProcessor()
        self.testmanager = MovementManager()
    def set_ip(self,ip):
        self.ip = ip
    def set_port(self, port):
        self.port = int(port)
    def set_linear_velocity(self,meterpersec):
        self.linear = meterpersec

    def set_angular_velocity(self,radianpersec):
        self.angular = radianpersec

    def start_move_straight(self,distance):
        self.testmanager.set_max_linear_velocity(self.linear)
        error = 0
        if (error >= 0 ):
            self.processor.register_command(0)
            error = self.collector.start()

            self.processor.register_command(self.linear)
            self.testmanager.move_straight(distance)
            time.sleep(.5)
            self.collector.stop()
            self.testmanager = MovementManager()
            ideal = self.processor.register_command(0, True)
            data = self.collector.get_data()
            data = np.vstack((data[0], data[0])) #faking second laser
            p_data = self.processor.process_all(data)
            fig = self.processor.get_plots()
            fig.savefig("./temp.png")
            plt.clf()
            plt.plot(ideal)
            plt.savefig("./ideal.png")
            return 0
        else:
            self.processor.register_command(0, True)
            return -1


    def start_rotate(radian):
        self.collector.start()
        self.testmanager.set_max_angular_velocity(self.angular)
        self.testmanager.turn(radian)
        self.collector.stop()
        data = collector.get_data()
        p_data = processor.process_all(data)


#Temp code
if __name__ == "__main__":

    collector = DataCollector("192.168.1.99", 80)

    processor = DataProcessor()

    testmanager = MovementManager()
    testmanager.set_max_linear_velocity(0.75)
    processor.register_command(0)
    collector.start()
    processor.register_command(.75)
    testmanager.move_straight(5)
    time.sleep(.5)
    collector.stop()

    testmanager = None
    ideal = processor.register_command(0, True)
    data = collector.get_data()
    data = np.vstack((data[0], data[0])) #faking second laser
    p_data = processor.process_all(data)
    fig = processor.get_plots()
    fig.savefig("./temp.png")
    print(ideal)
    plt.plot(ideal)

    print(p_data)
    plt.plot(p_data[5, :])

    plt.clf()
    plt.plot(ideal)
    plt.savefig("./ideal.png")