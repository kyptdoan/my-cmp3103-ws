import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import json
import math

class SavePose(Node):
    def __init__(self):
        super().__init__('save_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0, self.save)

    def save(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = {
                'x': t.transform.translation.x,
                'y': t.transform.translation.y,
                'z': t.transform.rotation.z,
                'w': t.transform.rotation.w
            }
            with open('saved_pose.json', 'w') as f:
                json.dump(pose, f)
            print(f"Pose saved: {pose}")
        except Exception as e:
            print(f"TF error: {e}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SavePose())

if __name__ == '__main__':
    main()