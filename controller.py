#data controller
#in charge of using the correct classes to generate data
from data_processor import DataProcessor
from data_collector import DataCollector
from movement_unit_tests import MovementManager

class Controller:
    def __init__(self):
        self.collector = DataCollector("192.168.1.99", "80")
        self.processor = DataProcessor()
        self.testmanager = MovementManager()
    def set_ip(self,ip):
        self.ip = ip
    def set_port(self, port):
        self.port = port
    def set_linear_velocity(meterpersec):
        self.linear = meterpersec
    def set_angular_velocity(radianpersec):
        self.angular = radianpersec
    def start_move_straight(distance):
        self.collector.start()
        self.testmanager.set_max_linear_velocity(self.linear)
        self.testmanager.move_straight(distance)
        self.collector.stop()
        data = collector.get_data()
        p_data = processor.process_all(data)
        print(p_data)
    def start_rotate(radian):
        self.collector.start()
        self.testmanager.set_max_angular_velocity(self.angular)
        self.testmanager.turn(radian)
        self.collector.stop()
        data = collector.get_data()
        p_data = processor.process_all(data)


#Temp code

# collector = DataCollector("192.168.1.99", "80")

# processor = DataProcessor()

# testmanager = MovementManager()
# testmanager.set_max_linear_velocity(0.75)

# collector.start()
# testmanager.move_straight(5)
# collector.stop()
# data = collector.get_data()
# p_data = processor.process_all(data)
# print(p_data)