from data_processor import DataProcessor
from data_collector import DataCollector
from movement_unit_tests import MovementManager
import matplotlib.pyplot as plt


collector = DataCollector("192.168.1.99", 80)

processor = DataProcessor()

testmanager = MovementManager()
testmanager.set_max_linear_velocity(0.75)

collector.start()
testmanager.move_straight(5)
collector.stop()
data = collector.get_data()
print(data)
p_data = processor.process_all(data)
print(p_data)
plot = processor.get_plots(debug=True)
plt.show()
