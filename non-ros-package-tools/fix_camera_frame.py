#It seems that the position reported by the GPS sometimes gets stuck
#The latitude, longitude and altitude remain exactly the same. However, the time stamp changes.
#This is extremely bad for the EKF
#This script filters out those bad values into a new rosbag

import rosbag
import glob, os

for file in sorted(glob.glob("*.bag")):
    print(file)
    with rosbag.Bag('output/' + file, 'w') as outbag:
        outbag._set_compression('lz4')
        for topic, msg, t in rosbag.Bag(file).read_messages():
            if topic == '/zed/right/image_rect_color':
                msg.header.frame_id = 'zed_right_camera'
            outbag.write(topic, msg, msg.header.stamp)
