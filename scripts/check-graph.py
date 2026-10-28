import json,time
import rclpy
rclpy.init()
n=rclpy.create_node('deployment_observer')
for _ in range(30):
    rclpy.spin_once(n,timeout_sec=.1)
print('nodes',json.dumps(n.get_node_names()))
for i in (1,2,3):
    for topic in (f'/rslidar_points_{i}',f'/detection_result_{i}'):
        print(topic,json.dumps({'publishers':n.count_publishers(topic),'subscribers':n.count_subscribers(topic)}))
n.destroy_node();rclpy.shutdown()
