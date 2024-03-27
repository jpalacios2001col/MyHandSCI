#!/usr/bin/env python3

import rosbag

input_bag_path = 'force_experiments/ND_LF_T_2.11N_Trial_1.bag'
output_bag_path = 'force_experiments/ND_LF_T_2.11N_Trial_1_fixed.bag'
corruption_timestamp = 1706811934.792375802

with rosbag.Bag(input_bag_path, 'r') as in_bag, rosbag.Bag(output_bag_path, 'w') as out_bag:
    for topic, msg, t in in_bag.read_messages():
        if t.to_sec() <= corruption_timestamp:
            out_bag.write(topic, msg, t)
        else:
            break
