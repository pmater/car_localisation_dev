#It seems that the position reported by the GPS sometimes gets stuck
#The latitude, longitude and altitude remain exactly the same. However, the time stamp changes.
#This is extremely bad for the EKF
#This script filters out those bad values into a new rosbag

import rosbag
import glob, os

latitudePrevious = -99

for file in sorted(glob.glob("*.bag")):
    print(file)
    with rosbag.Bag('output/' + file, 'w') as outbag:
        outbag._set_compression('lz4')
        for topic, msg, t in rosbag.Bag(file).read_messages():
            if topic == '/gps/fix':
                if abs(latitudePrevious - msg.latitude) > 0.0000000000001: #If there's a slight difference in latitude, it's valid
                    outbag.write(topic, msg, msg.header.stamp)
                else:
                    print('Invalid GPS detected: ' + str(msg.latitude) + ' vs ' + str(latitudePrevious))
                latitudePrevious = msg.latitude
            else:
                outbag.write(topic, msg, msg.header.stamp)




