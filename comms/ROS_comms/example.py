from publisher import ROS
import random
with ROS("temp_pub") as ros:
    temp = ros.publishers.Float32("/temperature")
    ros.timer(0.5, lambda: temp(random.uniform(25.0, 30.0)))
    ros.run(3000000000.0)
