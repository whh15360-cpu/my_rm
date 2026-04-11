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

# 卡尔曼滤波（单目标）
class Kalman:
    def __init__(self, dt=0.066, angular_velocity=7.33):  #原dt为0.033
        self.dt = dt
        self.angular_velocity = angular_velocity
        self.kf = cv2.KalmanFilter(8, 4, 0)
        self.kf.transitionMatrix = np.eye(8, dtype=np.float64)
        self.kf.measurementMatrix = np.zeros((4, 8), dtype=np.float64)
        self.kf.processNoiseCov = np.zeros((8, 8), dtype=np.float64)
        self.kf.measurementNoiseCov = np.zeros((4, 4), dtype=np.float64)

        self.kf.transitionMatrix[0, 3] = dt
        self.kf.transitionMatrix[1, 4] = dt
        self.kf.transitionMatrix[2, 5] = dt
        self.kf.transitionMatrix[6, 7] = dt

        self.kf.measurementMatrix[0, 0] = 1.0
        self.kf.measurementMatrix[1, 1] = 1.0
        self.kf.measurementMatrix[2, 2] = 1.0
        self.kf.measurementMatrix[3, 6] = 1.0

        # 噪声调小，更稳定
        self.kf.processNoiseCov[0,0] = 1e-4
        self.kf.processNoiseCov[1,1] = 1e-4
        self.kf.processNoiseCov[2,2] = 1e-4
        self.kf.processNoiseCov[3,3] = 5e-3
        self.kf.processNoiseCov[4,4] = 5e-3
        self.kf.processNoiseCov[5,5] = 5e-3
        self.kf.processNoiseCov[6,6] = 5e-3
        self.kf.processNoiseCov[7,7] = 1e-2

        self.kf.measurementNoiseCov[0,0] = 1e-2
        self.kf.measurementNoiseCov[1,1] = 1e-2
        self.kf.measurementNoiseCov[2,2] = 1e-2
        self.kf.measurementNoiseCov[3,3] = (0.5 * np.pi / 180.0) ** 2

        self.kf.errorCovPost = np.eye(8, dtype=np.float64) * 1.0
        self.inited = False

    def init(self, position, yaw=0.0):
        state = np.zeros(8, dtype=np.float64)
        state[0] = position[0]
        state[1] = position[1]
        state[2] = position[2]
        state[3] = 0.0
        state[4] = 0.0
        state[5] = 0.0
        state[6] = yaw
        state[7] = self.angular_velocity
        self.kf.statePost = state
        self.inited = True

    def predict(self):
        return self.kf.predict()

    def update(self, position, yaw=0.0):
        meas = np.zeros(4, dtype=np.float64)
        meas[0] = position[0]
        meas[1] = position[1]
        meas[2] = position[2]
        meas[3] = yaw
        self.kf.correct(meas)

    def getPosition(self):
        x = self.kf.statePost[0]
        y = self.kf.statePost[1]
        z = self.kf.statePost[2]
        return (float(x), float(y), float(z))

# 主节点
class ArmorDetectNode(Node):
    def __init__(self):
        super().__init__("armor_detect_node")
        self.get_logger().info("启动Python版本装甲板检测节点 + 稳定卡尔曼")

        self.pkg_path = get_package_share_directory("armor1_pose_solver")
        self.load_config()
        self.load_model()
        self.load_video()

        self.kalman_dict = {}
        self.last_center = {}

        self.armor_pub = self.create_publisher(BoundingBox2DArray, "/armor_detection_result", 10)
        self.run_video_detect()

    def load_config(self):
        yaml_path = os.path.join(self.pkg_path, "config", "armor_params.yaml")
        try:
            with open(yaml_path, 'r') as f:
                lines = [l for l in f if not l.startswith('%')]
                config = yaml.load("\n".join(lines), Loader=yaml.FullLoader)

            cam = config["cameraMatrix"]
            self.cameraMatrix = np.array(cam["data"], dtype=np.float64).reshape(cam["rows"], cam["cols"])
            dist = config["distCoeffs"]
            self.distCoeffs = np.array(dist["data"], dtype=np.float64).reshape(dist["rows"], -1)
            self.get_logger().info("相机标定加载成功")
        except:
            self.cameraMatrix = np.array([[1500,0,640],[0,1500,360],[0,0,1]], np.float64)
            self.distCoeffs = np.zeros((5,1), np.float64)
            
        self.armor_width  = 141.0
        self.armor_height = 125.0
        self.armor_3d = np.array([
            [-self.armor_width/2,  self.armor_height/2, 0],   # 左上
            [-self.armor_width/2, -self.armor_height/2, 0],   # 左下
            [ self.armor_width/2, -self.armor_height/2, 0],   # 右下
            [ self.armor_width/2,  self.armor_height/2, 0],   # 右上
        ], dtype=np.float64)

    def load_model(self):
        self.model = YOLO(os.path.join(self.pkg_path, "config", "best.pt"))
        self.get_logger().info("YOLO模型加载成功")

    def load_video(self):
        self.cap = cv2.VideoCapture(os.path.join(self.pkg_path, "config", "new_red9.mp4"))
        self.video_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.video_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        self.get_logger().info(f"视频：{self.video_w}x{self.video_h} {self.fps}fps")

    def solve_armor_pose(self, x1, y1, x2, y2):
        # 2D点：左上 → 右上 → 右下 → 左下
        armor_2d = np.array([
            [x1, y1],
            [x2, y1],
            [x2, y2],
            [x1, y2]
        ], dtype=np.float64)

        ret, rvec, tvec = cv2.solvePnP(
            self.armor_3d, armor_2d,
            self.cameraMatrix, self.distCoeffs,
            flags=cv2.SOLVEPNP_IPPE
        )
        if not ret:
            return None

        x, y, z = tvec[0,0], tvec[1,0], tvec[2,0]

        # 距离过滤      
        if z < 50 or z > 5000:
            return None

        return (x, y, z)

    def run_video_detect(self):
        frame_count = 0
        while rclpy.ok():
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            ret, frame = self.cap.read()
            if not ret:
                break
            frame_count += 1

            results = self.model(frame, imgsz=640, conf=0.75, iou=0.4, verbose=False, device="cpu")
            out = frame.copy()
            current_boxes = []

            for box in results[0].boxes:
                conf = round(box.conf[0].item(), 2)
                cls_id = int(box.cls[0].item())
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                pose = self.solve_armor_pose(x1, y1, x2, y2)
                if pose is None:
                    continue
                x, y, z = pose

                # 按中心位置匹配目标，防止跟错
                key = f"{cls_id}_{cx//5}_{cy//5}"

                if key not in self.kalman_dict:
                    self.kalman_dict[key] = Kalman()
                kf = self.kalman_dict[key]

                if not kf.inited:
                    kf.init((x, y, z))
                kf.predict()
                kf.update((x, y, z))
                kx, ky, kz = kf.getPosition()

                # 必要输出
                self.get_logger().info(
                    f"第{frame_count}帧，编号={cls_id}，置信度={conf}，中心({cx},{cy})，原始距离{z:.1f}mm，卡尔曼距离{kz:.1f}mm"
                )

                cv2.rectangle(out, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.putText(out, f"ID:{cls_id} Z:{z:.0f} KF:{kz:.0f}",
                            (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

            cv2.imshow("Armor + Kalman", out)

        self.cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = ArmorDetectNode()

if __name__ == "__main__":
    main()

