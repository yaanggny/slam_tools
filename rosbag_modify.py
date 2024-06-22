# ROS bag modify

import os, sys
import os.path as opath
import argparse

import random

from tqdm import tqdm

import rosbag
import rospy

def get_all_topics(bag_file):
    assert opath.exists(bag_file), bag_file

    with rosbag.Bag(bag_file) as bag_reader:
        topics = []
        for topic, msg, t in bag_reader.read_messages():
            if topic not in topics:
                topics.append(topic)
    print(topics)


def main():

    parser = argparse.ArgumentParser('rosbag_modify.py', description='change msg time stamps of a rosbag')
    parser.add_argument('input_bag', type=str, help='input ros bag file')
    parser.add_argument('-o', '--output_bag', type=str, help='output ros bag file')
    parser.add_argument('--topic', nargs='+', type=str, help='topic name')
    args = parser.parse_args()

    inputfile = args.input_bag
    assert opath.exists(inputfile), inputfile
    outputfile = args.output_bag

    dt = 1/1000*1e9  # 1ms in ns

    print(args)

    topics_tocopy = ['/imu0', '/cam0/image_raw', '/cam1/image_raw']
    topics_to_fix = args.topic
    print(topics_to_fix)


    bag_reader = rosbag.Bag(inputfile)
    outbag = rosbag.Bag(outputfile, 'w')
    for topic_ in topics_tocopy:
        c = 0
        to_fix = topic_ in topics_to_fix
        N_imu = 100
        MIN_STEP = 20 if 'image' in topic_ else N_imu

        for topic, msg, t in tqdm(bag_reader.read_messages(topics=topic_), ncols=80, desc=f'Fix timestamps {topic_} {MIN_STEP} '):
            # print(msg)

            t0 = msg.header.stamp
            STEP = MIN_STEP # random.randint(MIN_STEP, 12)
            if to_fix and c % STEP == 0:
                # msg.header.stamp = rospy.Time(secs=t0.secs, nsecs=int(t0.nsecs + random.randint(1e5, dt)))
                k = 3 if c % 2 == 0 else -2
                msg.header.stamp = rospy.Time(secs=t0.secs, nsecs=int(t0.nsecs + k*dt))
            # print(msg)
            # sys.exit(0)
            # print(topic, t0.to_sec() - msg.header.stamp.to_sec())
                
            outbag.write(topic, msg, msg.header.stamp)

            c += 1
    
    outbag.close()
    bag_reader.close()

    # print console footer
    print('Save: ', outputfile)

if __name__ == "__main__":
   main()