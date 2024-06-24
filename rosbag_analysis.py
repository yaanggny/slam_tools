import os, sys
import os.path as opath
import argparse

import rosbag

import matplotlib.pyplot as plt
import numpy as np
import tkinter as tk 

cur_dir = opath.dirname(opath.abspath(__file__))

def get_all_topics(bag_file):
    assert opath.exists(bag_file), bag_file

    with rosbag.Bag(bag_file) as bag_reader:
        topics = []
        for topic, msg, t in bag_reader.read_messages():
            if topic not in topics:
                topics.append(topic)
    # print(topics)
    return topics

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
    ts_msg = [x - ts_msg[0] for x in ts_msg]
    ts_record = [x - ts_record[0] for x in ts_record]
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
parser.add_argument('--cam_freq', type=int, help='camera image frequency in Hz')
parser.add_argument('--record_st', action='store_true', help='draw rosbag recorded timestamps')
args = parser.parse_args()


basename_noext = opath.splitext(opath.basename(args.input_bag))[0]
save_dir = opath.join(cur_dir, basename_noext)
os.makedirs(save_dir, exist_ok=True)

get_dts_and_curve(args.input_bag, args.topic, -1, args.record_st)


# root = tk.Tk()
# SCREEN_H, SCREEN_W = root.winfo_screenheight(), root.winfo_screenwidth()
# root.destroy()
SCREEN_H, SCREEN_W = 1080, 1920
dpi = 300
print('Screen Size: ', SCREEN_H, SCREEN_W)

bag_topics = get_all_topics(args.input_bag)

def compare_timestamps(imu_cnt=200, imu_freq=200, cam_freq=15):
    imu_topic = '/imu0' if 'imu_april' in args.input_bag else '/zed2i/zed_node/imu/data_raw'
    img_topic = '/cam0/image_raw' if 'imu_april' in args.input_bag else '/zed2i/zed_node/left/image_rect_color'
    img_topic = img_topic if img_topic in bag_topics else '/zed2i/zed_node/left/image_rect_color/compressed'

    ts1, times_x1, dts1, _, _, _ = get_dts(args.input_bag, imu_topic, -1)
    ts2, times_x2, dts2, _, _, _ = get_dts(args.input_bag, img_topic, -1)

    # ts1, times_x1, dts1, _, _, _ = get_dts(args.input_bag, '/imu0', -1)
    # ts2, times_x2, dts2, _, _, _ = get_dts(args.input_bag, '/cam0/image_raw', -1)
    # print(times_x1[:20])
    # print(times_x2[:20])

    imu_n = len(ts1)
    cam_STEP = int(imu_cnt*cam_freq/imu_freq)


    for i in range(imu_n//imu_cnt - 1):
        if i >= 30: break

        iS = i*imu_cnt
        iE = iS + imu_cnt
        cS = i*cam_STEP
        cE = cS + cam_STEP

        fig = plt.figure(figsize=(SCREEN_W/100, SCREEN_H/100), dpi=dpi)

        color = 'r'
        plt.stem(ts1[iS:iE], np.ones((imu_cnt, )), linefmt=color, markerfmt=color)
        color = 'b'
        plt.stem(ts2[cS:cE], 2*np.ones((cam_STEP, )), linefmt=color, markerfmt=color)
        plt.legend(['imu', 'image'])
        
        fo = opath.join(save_dir, f'{i}_stamps.jpg')

        # fig_m = plt.get_current_fig_manager()
        # fig_m.full_screen_toggle()
        # fig.canvas.print_figure()

        plt.savefig(fo, bbox_inches='tight', pad_inches=0, dpi=dpi)
        plt.close(fig=fig)
        print(fo, iS, iE, cS, cE)
        # plt.show()

    # plt.figure()
    # color = 'r'
    # plt.stem(ts1, np.ones((len(ts1), )), linefmt=color, markerfmt=color)
    # color = 'b'
    # plt.stem(ts2, 2*np.ones((len(ts2), )), linefmt=color, markerfmt=color)
    # plt.legend(['imu', 'image'])
    # plt.show()

# compare_timestamps(args.cnt, cam_freq=args.cam_freq)