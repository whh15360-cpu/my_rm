import rclpy
from rclpy.node import Node
from ultralytics import YOLO
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import yaml

from std_msgs.msg import Header
from vision_msgs.msg import BoundingBox2D, BoundingBox2DArray
from armor_interfaces.msg import ArmorShoot

class ArmorDetectNode(Node):
    def __init__(self):
        super().__init__("armor_detect_node")
        self.get_logger().info("启动Python版本装甲板检测节点")

        self.pkg_path = get_package_share_directory("armor1_pose_solver")
        self.load_config()
        self.load_model()
        self.load_video()

        self.armor_pub = self.create_publisher(BoundingBox2DArray, "/armor_detection_result", 10)
        self.shoot_pub = self.create_publisher(ArmorShoot, "/armor_shoot_info", 10)

        self.run_video_detect()

    def load_config(self):
        yaml_path = os.path.join(self.pkg_path, "config", "armor_params.yaml")
        try:
            with open(yaml_path, 'r') as f:
                lines = f.readlines()
                lines = [line for line in lines if not line.startswith('%YAML')]
                config = yaml.load("\n".join(lines), Loader=yaml.FullLoader)

            cam_mat_config = config["cameraMatrix"]
            self.cameraMatrix = np.array(cam_mat_config["data"], dtype=np.float32).reshape(cam_mat_config["rows"], cam_mat_config["cols"])

            dist_coeffs_config = config["distCoeffs"]
            self.distCoeffs = np.array(dist_coeffs_config["data"], dtype=np.float32).reshape(dist_coeffs_config["rows"], -1)
            self.get_logger().info("成功加载相机标定文件")

        except Exception as e:
            self.get_logger().warn(f"标定文件读取失败：{str(e)}，使用默认相机参数")
            self.cameraMatrix = np.array([[1500, 0, 640], [0, 1500, 360], [0, 0, 1]], dtype=np.float32)
            self.distCoeffs = np.zeros((5, 1), dtype=np.float32)

        self.armor_width = 130.0
        self.armor_height = 50.0
        self.armor_3d = np.array([
            [-self.armor_width/2, -self.armor_height/2, 0],
            [self.armor_width/2,  -self.armor_height/2, 0],
            [self.armor_width/2,   self.armor_height/2, 0],
            [-self.armor_width/2,  self.armor_height/2, 0]
        ], dtype=np.float32)

    def load_model(self):
        self.model_path = os.path.join(self.pkg_path, "config", "best.pt")
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"模型文件不存在：{self.model_path}")
            return
        self.model = YOLO(self.model_path)
        self.get_logger().info(f"成功加载YOLO模型：{self.model_path}")

    def load_video(self):
        self.video_path = os.path.join(self.pkg_path, "config", "armor7red.mp4")
        self.cap = cv2.VideoCapture(self.video_path)

        if not self.cap.isOpened():
            self.get_logger().error(f"无法打开视频文件：{self.video_path}")
            return

        self.video_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.video_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        self.get_logger().info(f"视频加载成功，尺寸：{self.video_w}x{self.video_h}，帧率：{self.fps}FPS")

    def solve_armor_pose(self, bbox_xyxy):
        x1, y1, x2, y2 = bbox_xyxy
        armor_2d = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32)

        ret, rvec, tvec = cv2.solvePnP(
            self.armor_3d, armor_2d,
            self.cameraMatrix, self.distCoeffs
        )
        if ret:
            return rvec, tvec
        else:
            return None, None

    def run_video_detect(self):
        paused = False
        frame_count = 0
        res_frame = np.zeros((self.video_h, self.video_w, 3), dtype=np.uint8)

        while rclpy.ok():
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                if paused:
                    self.get_logger().info("按Q键退出，程序结束")
                    break
                else:
                    paused = True
                    self.get_logger().info("按Q键暂停，再次按Q退出，按空格继续播放")
            elif key == ord(' '):
                paused = False
                self.get_logger().info("按空格继续播放")

            if not paused:
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().info("视频播放完毕，按Q键退出")
                    paused = True
                    continue

                frame_count += 1

                results = self.model(
                    frame,
                    imgsz=640,
                    conf=0.6,
                    iou=0.4,
                    verbose=False,
                    device="cpu"
                )

                armor_num = len(results[0].boxes)
                res_frame = frame.copy()

                armor_array = BoundingBox2DArray()
                armor_array.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="camera")

                for i, box in enumerate(results[0].boxes):
                    conf = round(box.conf[0].item(), 2)
                    cls_id = int(box.cls[0].item())  #  真实编号
                    bbox_xyxy = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = map(int, bbox_xyxy)
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2

                    rvec, tvec = self.solve_armor_pose(bbox_xyxy)
                    if rvec is not None:
                        tx, ty, tz = round(tvec[0][0], 1), round(tvec[1][0], 1), round(tvec[2][0], 1)
                        if frame_count % 1 == 0:
                            #  这里改成显示真实编号
                            self.get_logger().info(f"帧{frame_count}：编号={cls_id}，置信度={conf}，中心({center_x},{center_y})，距离{tz}mm")
                    else:
                        if frame_count % 2 == 0:
                            self.get_logger().info(f"帧{frame_count}：编号={cls_id}，置信度={conf}，位姿解算失败")

                    cv2.rectangle(res_frame, (x1, y1), (x2, y2), (0,255,0), 2)
                    cv2.putText(res_frame, f"ID:{cls_id} Conf:{conf}", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
                    cv2.circle(res_frame, (center_x, center_y), 4, (255,0,0), -1)

                    armor_bbox = BoundingBox2D()
                    armor_bbox.center.position.x = float(center_x)
                    armor_bbox.center.position.y = float(center_y)
                    armor_bbox.size_x = float(x2 - x1)
                    armor_bbox.size_y = float(y2 - y1)
                    armor_array.boxes.append(armor_bbox)

                self.armor_pub.publish(armor_array)

            cv2.imshow("Armor Video Detection", res_frame)
            cv2.resizeWindow("Armor Video Detection", self.video_w, self.video_h)

        self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info("资源释放完成，程序退出")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = ArmorDetectNode()

if __name__ == "__main__":
    main()
