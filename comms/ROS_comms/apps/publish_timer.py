#!/usr/bin/env python3

import sys, pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parents[1]))

from roscomms import ROS, PublisherFactory, QOS_SENSOR
import random

with ROS("temp_timer_pub") as ros:
    factory = PublisherFactory(ros)
    temp = factory.from_string("/temperature", "std_msgs/Float32", QOS_SENSOR)

    ros.timer(0.5, lambda: temp(random.uniform(25.0, 30.0)))
    ros.log.info("Publishing /temperature @0.5s for 5s")
    ros.run(30.0)
