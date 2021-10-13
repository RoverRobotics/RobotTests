from data_processor import DataProcessor
from data_collector import DataCollector
from movement_unit_tests import MovementManager



collector = DataCollector("192.168.1.99", "80")

processor = DataProcessor()

testmanager = MovementManager()
testmanager.set_max_linear_velocity(0.75)

collector.start()
testmanager.move_straight(5)
collector.stop()
data = collector.get_data()
p_data = processor.process_all(data)
print(p_data)