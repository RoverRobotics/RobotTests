import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from sklearn.mixture import GaussianMixture
import scipy.signal
from data_processor import DataProcessor
import time
# Hz
ROTATION_RATE = 2

FILTER_PASSES = 20
CONV_PAD = FILTER_PASSES

# on the tire
STUBS = 36
WHEEL_CIRCUMFERENCE = 0.797964534

# how wide are they within the repeating stub pattern
STUB_RATIO = 0.75
STUB_HEIGHT = 2
STUB_H_VARIANCE = 0.2
BASE_DISTANCE_TO_LASER = 40
BASE_DISTANCE_VAR = 0.01
BASE_DISTANCE_WARP = 10

EDGE_DETECTOR = [-1, 0, 1]
FILTER = [.1,.2,.4,.2,.1]

# Hz
SAMPLE_RATE = 2000
DATA_POINTS = 10000

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

def generate_sim_data():

    sim_measurements = list()
    for dp in range(DATA_POINTS):
        time = dp / SAMPLE_RATE
        pattern_period = 1 / (ROTATION_RATE * STUBS)
        stub_period = pattern_period * STUB_RATIO
        absolute_radians = (dp / SAMPLE_RATE) * ROTATION_RATE * 2 * np.pi % (2 * np.pi)
        
        # generate the base height and the stub height
        base_distance = np.random.normal(BASE_DISTANCE_TO_LASER, BASE_DISTANCE_VAR) + BASE_DISTANCE_WARP * np.sin(absolute_radians)
        stub_height = np.random.normal(STUB_HEIGHT, STUB_H_VARIANCE)

        # calculate the point in the pattern
        if (time % pattern_period) < stub_period:
            sim_measurements.append(base_distance + stub_height)
        else:
            sim_measurements.append(base_distance)
    return np.array(sim_measurements).reshape(-1, 1)

def generate_gt_data():

    sim_measurements = list()
    for dp in range(DATA_POINTS):
        time = dp / SAMPLE_RATE
        pattern_period = 1 / (ROTATION_RATE * STUBS)
        stub_period = pattern_period * STUB_RATIO

        # calculate the point in the pattern
        if (time % pattern_period) < stub_period:
            sim_measurements.append(0)
        else:
            sim_measurements.append(1)
    return np.array(sim_measurements).reshape(-1, 1)

def generate_model(measurements):
    return GaussianMixture(n_components=3).fit(measurements)


with open('readme_new.txt') as f:
    lines=f.readlines()
    for line in lines:
        myarray = np.fromstring(line, dtype=float, sep=',')

# sim_data = generate_sim_data()
sim_data = myarray
filt_data = np.convolve(np.squeeze(sim_data), FILTER, mode='same').reshape(-1, 1)

for i in range(FILTER_PASSES):
    filt_data = np.convolve(np.squeeze(filt_data), FILTER, mode='same').reshape(-1, 1)

filt_data[0:CONV_PAD] = filt_data[CONV_PAD + 1]
filt_data[-CONV_PAD:] = filt_data[-CONV_PAD - 1]

edge_data = np.convolve(np.squeeze(filt_data), EDGE_DETECTOR, mode='same').reshape(-1, 1)

for i in range(FILTER_PASSES):
    edge_data = np.convolve(np.squeeze(edge_data), FILTER, mode='same').reshape(-1, 1)

# clip negative values
edge_data[edge_data < 0] = 0

edge_data[0:CONV_PAD] = edge_data[CONV_PAD + 1]
edge_data[-CONV_PAD:] = edge_data[-CONV_PAD - 1]

edge_data = np.where(edge_data > 0.3 * np.max(edge_data), 1, 0)

# count the distance function
transitions = ~edge_data & np.roll(edge_data, -1)

distances = list()
distance = 0
last_transition = 0
for i, t in enumerate(transitions):
    if t:
        distance += WHEEL_CIRCUMFERENCE / STUBS
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

speed = np.gradient(distances) * SAMPLE_RATE
for _ in range(5000):
    speed = np.convolve(speed, FILTER, 'same')

speed[speed < 0.05] = 0

# generate ground-truth
gtdat = np.zeros(speed.shape)
gtdat[speed > 0] = 0.75

# compute error
error = np.zeros(speed.shape)
error = np.abs(gtdat - speed)
error = np.cumsum(error)





with open('readme_new.txt') as f:
    lines=f.readlines()
    for line in lines:
        myarray = np.fromstring(line, dtype=float, sep=',')

proc = DataProcessor()
procdata = proc.process_all(np.vstack((myarray[np.newaxis, ...], myarray[np.newaxis, ...])))
fig = proc.get_plots(debug=True)

fig.savefig("test.png", dpi=1000)
plt.show()

proc.register_command(0)
time.sleep(0.5)
proc.register_command(0.5)
time.sleep(0.5)
ideal = proc.register_command(0, terminate=True)

fig, axs = plt.subplots(2, sharex='col')
# axs[0].plot(sim_data)
# axs[1].plot(filt_data)
# axs[2].plot(edge_data)
# axs[3].plot(distances)
axs[0].plot(speed)
axs[1].plot(ideal, color='r')
# axs[5].plot(error, color='r')
plt.show()
#plt.close()

# print('accuracy: ', accuracy)