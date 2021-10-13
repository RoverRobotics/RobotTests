import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

FILTER_PASSES = 20
CONV_PAD = 20


def nan_helper(y):
    """Helper to handle indices and logical indices of NaNs.

    Input:
        - y, 1d numpy array with possible NaNs
    Output:
        - nans, logical indices of NaNs
        - index, a function, with signature indices= index(logical_indices),
          to convert logical indices of NaNs to 'equivalent' indices
    Example:
        >>> # linear interpolation of NaNs
        >>> nans, x= nan_helper(y)
        >>> y[nans]= np.interp(x(nans), x(~nans), y[~nans])
    """

    return np.isnan(y), lambda z: z.nonzero()[0]

class DataProcessor:
    def __init__(self, sample_rate_hz=2000, wheel_circumference=0.797964534, edges_per_rot=36):
        self.pdata = None
        self.sample_rate_hz = sample_rate_hz
        self.wheel_circumference = wheel_circumference
        self.edges_per_rot = edges_per_rot

    def process_all(self, data):
        ''' Input: np.ndarray of shape (x, N) where N is number of samples, and x is number of laser channels.
            Returns: np.ndarray of shape (y, N) where y is x * (distances, speeds)'''
        filtered = [self.filter_(d) for d in data]
        edges = [self.edge_detect_(d) for d in filtered]
        distances = [self.get_distance_(d) for d in edges]
        speeds = [self.get_speed_(d) for d in distances]

        return np.vstack((distances, speeds))
    
    def filter_(self, data, filter=[.1,.2,.4,.2,.1], num_iter=FILTER_PASSES):
        '''1d convolution filter for low-pass effect'''

        for _ in range(num_iter):
            data = np.convolve(np.squeeze(data), filter, mode='same')

        # gets rid of weird convolution artifacts that are bad
        data[0:CONV_PAD] = data[CONV_PAD + 1]
        data[-CONV_PAD:] = data[-CONV_PAD - 1]

        return data
        
    def edge_detect_(self, data, kernel=[-1, 0, 1], threshold=0.3):
        '''use edge detect to count rising edge (i.e changes in wheel surface)'''

        # convolve with edge detector kernel
        data = np.convolve(np.squeeze(data), kernel, mode='same').reshape(-1, 1)

        # filter the data
        data = self.filter_(data)

        # clip negative values
        data[data < 0] = 0

        # apply threshold
        data = np.where(data > threshold * np.max(data), 1, 0)

        # find all transitions
        transitions = ~data & np.roll(data, -1)

        return transitions

    def get_speed_(self, distances, smoothing_iters=5000, filter=[.1,.2,.4,.2,.1], thresh=0.05):
        '''translate distances into speeds'''
        # compute speed
        speed = np.gradient(distances) * self.sample_rate_hz
        
        # apply smoothing
        speed = self.filter_(speed, num_iter=smoothing_iters)

        # clip low speeds
        speed[speed < thresh] = 0

        return speed

    def get_distance_(self, transitions):
        '''translate edge transitions into distances'''
        distances = list()
        distance = 0
        for i, t in enumerate(transitions):
            if t:
                distance += self.wheel_circumference / self.edges_per_rot
                distances.append(distance)
            else:
                distances.append(0)

        print(np.max(distances))

        # interpolate
        distances = pd.DataFrame(distances)
        distances.replace(0, np.NaN, inplace=True)
        distances = np.array(distances)
        nans, x = nan_helper(distances)
        distances[nans] = np.interp(x(nans), x(~nans), distances[~nans])
        distances = np.squeeze(distances)

        return distances

    def plot(self, data):
        '''plot data for visualization'''
        pass

    def plot_all(self):
        pass

    def compare(expected_data, actual_data, ): #tuple of test_data(speed_1, speed_2, distance_1,distance_2) and similar for actual_data
        '''Compare list of test data with list of actual measured data and return an acuracy rating'''
        '''(N, expected data) (N, actual data)'''
        '''return avg error , '''
        # return acuracy ? 
        pass