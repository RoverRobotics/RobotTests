class DataCollector:
    def __init__(self,data) :
        self.data = data # numpy array from datacollector
        self.left_laser = None
        self.right_laser = None
        self.filtered_data_left = None
        self.filtered_data_right = None
        self.edge_detected_data_left = None
        self.edge_detected_data_right = None
        pass
    def split_data_():
        '''split combined laser data into left(1) and right(2) laser data for easier processing '''
        pass
    def filter_():
        '''filter laser data to reduce noise'''
        pass
    def edge_detect_():
        '''use edge detect to count rising edge (i.e changes in wheel surface)'''
        pass
    def get_speed1_():
        '''get speed for motor 1 (left) '''
        pass
    def get_speed2_():
        '''get speed for motor 2 (right) '''
        pass
    def get_distance1_():
        '''get distance for motor 1 (left) '''
        pass
    def get_distance2_():
        '''get distance for motor 2 (right) '''
        pass
    def process_all():
        '''return tuple of speed_1, speed_2, distance_1, distance_2'''
        pass
    def plot(data):
        '''plot data for visualization'''
        pass
    def plot_all():
        pass
    def compare(expected_data, actual_data, ): #tuple of test_data(speed_1, speed_2, distance_1,distance_2) and similar for actual_data
        '''Compare list of test data with list of actual measured data and return an acuracy rating'''
        '''(N, expected data) (N, actual data)'''
        '''return avg error , '''
        # return acuracy ? 
        pass