#!/usr/bin/env python3
"""
aruco_node — détection du cube ArUco via webcam et publication de sa pose.

Réécrit (2026-06-12) pour la webcam du PC, depuis spec-vision-aruco-tracking.
L'original vivait sur le Pi4 (mort) et n'avait jamais été commité.

Publie :
  /cube_pose    geometry_msgs/PoseStamped   pose du CENTRE du cube (repère caméra)
  /image_raw    sensor_msgs/Image           image annotée (marqueurs + axes) pour debug/RViz
  /cube_marker  visualization_msgs/Marker   cube pour RViz
  TF: <camera_frame> -> cube

Détection : OpenCV 4.6 (API transitionnelle), dictionnaire DICT_4X4_50,
marqueurs 10 cm sur un cube 12 cm, 6 faces (IDs 0-5). Robuste à 1 marqueur visible.
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster


def rvec_to_quat(rvec):
    """Vecteur de rotation (Rodrigues) -> quaternion (x,y,z,w)."""
    R, _ = cv2.Rodrigues(rvec)
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


def quat_to_R(q):
    """quaternion (x,y,z,w) -> matrice de rotation 3x3."""
    x, y, z, w = q
    n = np.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])


def R_to_quat_mat(R):
    """matrice de rotation 3x3 -> quaternion (x,y,z,w)."""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25*s; x = (R[2, 1]-R[1, 2])/s; y = (R[0, 2]-R[2, 0])/s; z = (R[1, 0]-R[0, 1])/s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0+R[0, 0]-R[1, 1]-R[2, 2])*2
        w = (R[2, 1]-R[1, 2])/s; x = 0.25*s; y = (R[0, 1]+R[1, 0])/s; z = (R[0, 2]+R[2, 0])/s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0+R[1, 1]-R[0, 0]-R[2, 2])*2
        w = (R[0, 2]-R[2, 0])/s; x = (R[0, 1]+R[1, 0])/s; y = 0.25*s; z = (R[1, 2]+R[2, 1])/s
    else:
        s = np.sqrt(1.0+R[2, 2]-R[0, 0]-R[1, 1])*2
        w = (R[1, 0]-R[0, 1])/s; x = (R[0, 2]+R[2, 0])/s; y = (R[1, 2]+R[2, 1])/s; z = 0.25*s
    return float(x), float(y), float(z), float(w)


def avg_R(Rlist):
    """Moyenne de rotations (projection L2 sur SO(3) via SVD)."""
    M = np.sum(Rlist, axis=0)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        # --- paramètres ---
        self.declare_parameter('video_device', 0)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('marker_size', 0.10)   # m (arête du marqueur)
        self.declare_parameter('cube_size', 0.12)     # m (arête du cube)
        self.declare_parameter('camera_frame', 'camera')
        # Matrice caméra APPROXIMATIVE (webcam non calibrée). fx≈fy≈largeur pour
        # un FOV ~65°. La précision absolue n'est pas critique : le visual_servo
        # utilise un suivi relatif (remap échelle). À calibrer si besoin.
        self.declare_parameter('fx', 1000.0)
        self.declare_parameter('fy', 1000.0)
        self.declare_parameter('cx', 640.0)
        self.declare_parameter('cy', 360.0)
        self.declare_parameter('publish_image', True)
        # Rejet d'aberrations : on ignore les marqueurs dont la profondeur est
        # implausible (lecture d'un marqueur vu de très loin/de biais -> pose folle).
        self.declare_parameter('min_range', 0.10)   # m
        self.declare_parameter('max_range', 2.50)   # m
        # Calibration des faces (orientation cohérente du cube). Vide = orientation
        # brute de la meilleure face (saute par face).
        self.declare_parameter(
            'cube_faces_file',
            '/home/sam/ros2_ws/src/roby_control/config/cube_faces.yaml')
        # L'image pleine résolution coûte cher (cv_bridge+DDS) et plafonne la
        # détection à ~4 Hz. On la publie réduite et 1 tick sur N (pour rqt),
        # ce qui laisse /cube_pose tourner à ~29 Hz.
        self.declare_parameter('image_pub_every', 4)
        self.declare_parameter('image_pub_scale', 0.5)

        g = lambda n: self.get_parameter(n).value
        self.width, self.height = g('width'), g('height')
        self.marker_size = g('marker_size')
        self.half_cube = g('cube_size') / 2.0
        self.camera_frame = g('camera_frame')
        self.publish_image = g('publish_image')
        self.min_range = g('min_range')
        self.max_range = g('max_range')
        self.image_pub_every = max(1, int(g('image_pub_every')))
        self.image_pub_scale = float(g('image_pub_scale'))
        self.frame_count = 0
        self.K = np.array([[g('fx'), 0, g('cx')],
                           [0, g('fy'), g('cy')],
                           [0, 0, 1]], dtype=np.float64)
        self.dist = np.zeros((4, 1))

        # --- ArUco (API transitionnelle OpenCV 4.6) ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # --- calibration des faces (R_marqueur->cube par marqueur) ---
        self.face_R = {}
        path = g('cube_faces_file')
        try:
            import yaml
            with open(path) as f:
                data = yaml.safe_load(f)
            for mid, q in (data.get('faces') or {}).items():
                self.face_R[int(mid)] = quat_to_R(q)
            self.get_logger().info(
                f"Calibration faces chargée ({sorted(self.face_R)}) depuis {path}")
        except Exception as e:
            self.get_logger().warn(
                f"Pas de calibration faces ({e}) -> orientation brute (saute par face).")

        # --- caméra ---
        dev = g('video_device')
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f"Impossible d'ouvrir la caméra {dev}")
            raise RuntimeError('camera open failed')
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.get_logger().info(
            f"Caméra {dev} ouverte {int(self.cap.get(3))}x{int(self.cap.get(4))}")

        # --- ROS I/O ---
        self.pose_pub = self.create_publisher(PoseStamped, '/cube_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/cube_marker', 10)
        if self.publish_image:
            self.image_pub = self.create_publisher(Image, '/image_raw', 5)
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / g('fps'), self.tick)
        self.last_log = self.get_clock().now()

    def tick(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return
        self.frame_count += 1
        do_img = self.publish_image and (self.frame_count % self.image_pub_every == 0)
        frame = frame.copy()  # bug buffer OpenCV (hérité Pi4, inoffensif ici)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)

        stamp = self.get_clock().now().to_msg()
        if ids is not None and len(ids) > 0:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.K, self.dist)

            centers = []   # estimations du CENTRE du cube par chaque face
            areas = []
            kept_ids = []
            for i in range(len(ids)):
                t = tvecs[i].reshape(3)
                # rejet d'aberration : profondeur implausible -> on ignore ce marqueur
                if not (self.min_range < t[2] < self.max_range):
                    continue
                R, _ = cv2.Rodrigues(rvecs[i])
                # le centre du cube est derrière la face de ½ arête, le long de
                # la normale (axe z du marqueur pointe hors de la face).
                center = t - self.half_cube * R[:, 2]
                centers.append(center)
                areas.append(cv2.contourArea(corners[i].reshape(-1, 2)))
                kept_ids.append(i)

            if centers:   # au moins un marqueur plausible retenu
                ids = ids[kept_ids]
                rvecs = rvecs[kept_ids]
                corners = [corners[i] for i in kept_ids]   # garder corners cohérent (bug fix)
                centers = np.array(centers)
                cube_pos = centers.mean(axis=0)         # fusion = moyenne des centres
                best = int(np.argmax(areas))            # marqueur le plus "de face"
                # ORIENTATION cohérente : chaque face calibrée donne la MÊME
                # orientation cube (R_marqueur · R_marqueur->cube) ; on les moyenne.
                Rcubes = [cv2.Rodrigues(rvecs[k])[0] @ self.face_R[int(ids[k])]
                          for k in range(len(ids)) if int(ids[k]) in self.face_R]
                if Rcubes:
                    qx, qy, qz, qw = R_to_quat_mat(avg_R(Rcubes))
                else:
                    qx, qy, qz, qw = rvec_to_quat(rvecs[best])  # fallback non calibré

                self._publish_pose(stamp, cube_pos, (qx, qy, qz, qw))
                self._publish_tf(stamp, cube_pos, (qx, qy, qz, qw))
                self._publish_cube_marker(stamp, cube_pos, (qx, qy, qz, qw))

                if do_img:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    for i in range(len(ids)):
                        cv2.drawFrameAxes(frame, self.K, self.dist,
                                          rvecs[i], tvecs[i], self.marker_size * 0.5)

                now = self.get_clock().now()
                if (now - self.last_log).nanoseconds > 1e9:
                    self.get_logger().info(
                        f"{len(ids)} marqueur(s) {ids.flatten().tolist()} | "
                        f"cube @ x={cube_pos[0]:+.3f} y={cube_pos[1]:+.3f} z={cube_pos[2]:+.3f} m")
                    self.last_log = now

        if do_img:
            if self.image_pub_scale != 1.0:
                frame = cv2.resize(frame, None, fx=self.image_pub_scale,
                                   fy=self.image_pub_scale, interpolation=cv2.INTER_AREA)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = stamp
            msg.header.frame_id = self.camera_frame
            self.image_pub.publish(msg)

    def _publish_pose(self, stamp, pos, quat):
        m = PoseStamped()
        m.header.stamp = stamp
        m.header.frame_id = self.camera_frame
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pos)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = quat
        self.pose_pub.publish(m)

    def _publish_tf(self, stamp, pos, quat):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.camera_frame
        tf.child_frame_id = 'cube'
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, pos)
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = quat
        self.tf_broadcaster.sendTransform(tf)

    def _publish_cube_marker(self, stamp, pos, quat):
        mk = Marker()
        mk.header.stamp = stamp
        mk.header.frame_id = self.camera_frame
        mk.ns = 'cube'
        mk.id = 0
        mk.type = Marker.CUBE
        mk.action = Marker.ADD
        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = map(float, pos)
        mk.pose.orientation.x, mk.pose.orientation.y, mk.pose.orientation.z, mk.pose.orientation.w = quat
        mk.scale.x = mk.scale.y = mk.scale.z = 2.0 * self.half_cube
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.1, 0.6, 1.0, 0.8
        self.marker_pub.publish(mk)

    def destroy_node(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
