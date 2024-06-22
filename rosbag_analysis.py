import os, sys
import os.path as opath
import argparse

import rosbag

import matplotlib.pyplot as plt
import numpy as np


def get_dts(bag_file='', topic_='', cnt=500):
    assert opath.exists(bag_file), bag_file

    if cnt is None or cnt < 0:
        cnt = 1e9

    bag_reader = rosbag.Bag(bag_file)

    ts_msg, ts_record = [], []
    c = 0
    for topic, msg, t in bag_reader.read_messages(topics=topic_):
        # print(type(t), type(msg.header.stamp), t, t.to_sec())
        ts_record.append(t.to_sec())
        ts_msg.append(msg.header.stamp.to_sec())

        if c > cnt:
            break
        c += 1
    
    dts1, dts2 = [], []
    times_x1, times_x2 = [], []
    for i in range(1, len(ts_msg)):
        times_x1.append(ts_msg[i] - ts_msg[0])
        dts1.append((ts_msg[i] - ts_msg[i-1])*1000)

        times_x2.append(ts_record[i] - ts_record[0])
        dts2.append((ts_record[i] - ts_record[i-1])*1000)
    
    return ts_msg, times_x1, dts1, ts_record, times_x2, dts2
    
def plot_curves(times_x1, dts1, times_x2, dts2, topic_, plot_record=True):
    n = 2 if plot_record else 1
    fig, ax = plt.subplots(n, 1, sharex=True)
    if n == 1:
        ax.scatter(times_x1, dts1, marker='.')
        ax.set_ylabel('$\Delta t$/ms')
        ax.set_title(f'{topic_} - intervals')
    else:
        ax[0].scatter(times_x1, dts1, marker='.')
        ax[0].set_ylabel('$\Delta t$/ms')
        ax[0].set_title(f'{topic_} - msg intervals')

        ax[1].scatter(times_x2, dts2, marker='.')
        ax[1].set_ylabel('$\Delta t$/ms')
        ax[1].set_title(f'{topic_} - rosbag-record intervals')
    plt.xlabel('t/s')
    plt.show()    

def get_dts_and_curve(bag_file='', topic_='', cnt=500, plot_record=False):
    _, times_x1, dts1, _, times_x2, dts2 = get_dts(bag_file, topic_, cnt)
    plot_curves(times_x1, dts1, times_x2, dts2, topic_, plot_record)

parser = argparse.ArgumentParser('rosbag_modify.py', description='analyze msg timestamps in rosbag file')
parser.add_argument('input_bag', type=str, help='input ros bag file')
parser.add_argument('--topic', type=str, default='/imu0', help='topic name')
parser.add_argument('--cnt', type=int, help='number of msgs to get for analysis')
parser.add_argument('--record_st', action='store_true', help='draw rosbag recorded timestamps')
args = parser.parse_args()

get_dts_and_curve(args.input_bag, args.topic, -1, args.record_st)

def compare_timestamps(imu_cnt=200):
    ts1, times_x1, dts1, _, _, _ = get_dts(args.input_bag, '/imu0', imu_cnt)
    ts2, times_x2, dts2, _, _, _ = get_dts(args.input_bag, '/cam0/image_raw', imu_cnt//10)  # 10 = freq factor
    # print(times_x1[:20])
    # print(times_x2[:20])
    plt.figure()
    color = 'r'
    plt.stem(ts1, np.ones((len(ts1), )), linefmt=color, markerfmt=color)
    color = 'b'
    plt.stem(ts2, 2*np.ones((len(ts2), )), linefmt=color, markerfmt=color)
    plt.legend(['imu', 'image'])
    plt.show()

compare_timestamps(args.cnt)