#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import json
import numpy as np
import cv2
import easyocr
import rospy
import tf
import threading

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String, Bool, Int32
from scipy.ndimage import distance_transform_edt

class BoxCounterPerception:
    def __init__(self):
        # =========================
        # Parameters
        # =========================
        self.rate = rospy.get_param("~rate", 10)

        self.image_topic = rospy.get_param("~image_topic", "/kinect/rgb/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/kinect/rgb/camera_info")
        self.scan_topic = rospy.get_param("~scan_topic", "/front/scan")
        self.odom_topic = rospy.get_param("~odom_topic", "/final_slam/odom")

        self.camera_frame_override = rospy.get_param("~camera_frame", "")
        self.lidar_frame = rospy.get_param("~lidar_frame", "front_laser")
        self.map_frame = rospy.get_param("~map_frame", "map")

        self.use_gpu = rospy.get_param("~use_gpu", True)

        # OCR thresholds
        self.ocr_conf_thresh = rospy.get_param("~ocr_conf_thresh", 0.95)
        self.min_diag_len = rospy.get_param("~min_diag_len", 45.0)
        self.max_text_len = rospy.get_param("~max_text_len", 1)

        # Recognition stability
        self.required_stable_hits = rospy.get_param("~required_stable_hits", 10)
        self.same_obs_time_window = rospy.get_param("~same_obs_time_window", 1.0)
        self.min_digit_votes = rospy.get_param("~min_digit_votes", 2)

        # Loop-closure / map-jump recovery
        self.loop_recovery_duration = rospy.get_param("~loop_recovery_duration", 2.5)   # sec
        self.loop_recovery_merge_radius = rospy.get_param("~loop_recovery_merge_radius", 0.75)

        # LiDAR matching
        self.front_range_min = rospy.get_param("~front_range_min", 0.15)
        self.front_range_max = rospy.get_param("~front_range_max", 8.0)
        self.search_half_window = rospy.get_param("~search_half_window", 2)
        self.range_offset = rospy.get_param("~range_offset", 0.0)

        # Dedup / track association
        self.pending_match_radius = rospy.get_param("~pending_match_radius",0.5)
        self.track_match_radius = rospy.get_param("~track_match_radius", 0.6)
        self.confirmed_reuse_radius = rospy.get_param("~confirmed_reuse_radius", 0.55)
        self.confirmed_lock_radius = rospy.get_param("~confirmed_lock_radius", 0.5)

        # =========================
        # Two-stage box counting
        # =========================
        self.enable_counting_topic = rospy.get_param("~enable_counting_topic", "/percep/enable_counting")
        self.counting_enabled = rospy.get_param("~counting_enabled_initial", False)

        # lidar-only box-slot extraction
        self.box_slot_cluster_radius = rospy.get_param("~box_slot_cluster_radius", 0.5)
        self.box_slot_merge_radius = rospy.get_param("~box_slot_merge_radius", 0.5)
        self.box_slot_duplicate_radius = rospy.get_param("~box_slot_duplicate_radius", 0.42)
        self.box_slot_confirm_hits = rospy.get_param("~box_slot_confirm_hits", 8)

        # 针对约 0.8m 的箱子，放宽下限以捕获畸变或单面/单边
        # 放宽上限以捕获旋转对角线 (0.8*1.414 ≈ 1.13) + 误差
        self.box_size_min = rospy.get_param("~box_size_min", 0.6)
        self.box_size_max = rospy.get_param("~box_size_max", 1.2)
        self.box_max_arc = rospy.get_param("~box_max_arc", 1.6)

        # when counting is enabled, OCR observation must match existing lidar slot
        self.slot_assign_radius = rospy.get_param("~slot_assign_radius", 0.5)

        # Rotation gating
        # |yaw_rate| <= yaw_no_new_slot_thresh: 正常更新
        # yaw_no_new_slot_thresh < |yaw_rate| <= yaw_freeze_thresh: 不新建slot，不OCR
        # |yaw_rate| > yaw_freeze_thresh: 完全冻结，不更新slot，不OCR
        self.yaw_no_new_slot_thresh = rospy.get_param("~yaw_no_new_slot_thresh", 0.1)   # rad/s
        self.yaw_freeze_thresh = rospy.get_param("~yaw_freeze_thresh", 0.2)             # rad/s

        # Cone trigger
        self.enable_cone_trigger = rospy.get_param("~enable_cone_trigger", True)
        self.cone_trigger_topic = rospy.get_param("~cone_trigger_topic", "/cmd_unblock")
        self.cone_trigger_distance = rospy.get_param("~cone_trigger_distance", 0.8)
        self.cone_trigger_cooldown = rospy.get_param("~cone_trigger_cooldown", 2.0)

        # HSV threshold for orange cone
        self.cone_h_low = rospy.get_param("~cone_h_low", 3)
        self.cone_h_high = rospy.get_param("~cone_h_high", 28)
        self.cone_s_low = rospy.get_param("~cone_s_low", 60)
        self.cone_s_high = rospy.get_param("~cone_s_high", 255)
        self.cone_v_low = rospy.get_param("~cone_v_low", 40)
        self.cone_v_high = rospy.get_param("~cone_v_high", 255)

        self.cone_min_area = rospy.get_param("~cone_min_area", 500)
        self.cone_min_aspect = rospy.get_param("~cone_min_aspect", 0.35)
        self.cone_max_aspect = rospy.get_param("~cone_max_aspect", 3.0)

        self.last_cone_trigger_time = -1e9
        self._unblock_sent_once = False

        # Optional floor1 bounding box filter
        self.use_floor_filter = rospy.get_param("~use_floor_filter", False)
        self.floor_x_min = rospy.get_param("~floor_x_min", -1e9)
        self.floor_x_max = rospy.get_param("~floor_x_max", 1e9)
        self.floor_y_min = rospy.get_param("~floor_y_min", -1e9)
        self.floor_y_max = rospy.get_param("~floor_y_max", 1e9)

        self.current_seen_digit_topic = rospy.get_param("~current_seen_digit_topic", "/percep/current_seen_digit")
        self.current_seen_digit_pub = rospy.Publisher(self.current_seen_digit_topic, Int32, queue_size=1)

        # Occupancy map constraint
        self.use_occupancy_map_constraint = rospy.get_param("~use_occupancy_map_constraint", True)
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.map_occupied_thresh = rospy.get_param("~map_occupied_thresh", 50)   # >=50 视为占据
        self.map_unknown_is_invalid = rospy.get_param("~map_unknown_is_invalid", True)
        self.map_min_free_clearance = rospy.get_param("~map_min_free_clearance", 0.1)  # 米
        self.map_snap_to_free = rospy.get_param("~map_snap_to_free", True)
        self.map_snap_search_radius = rospy.get_param("~map_snap_search_radius", 0.45)   # 米

        # Landmark-style online update
        self.landmark_assoc_radius = rospy.get_param("~landmark_assoc_radius", 0.65)
        self.landmark_confirmed_assoc_radius = rospy.get_param("~landmark_confirmed_assoc_radius", 0.55)
        self.landmark_alpha_unconfirmed = rospy.get_param("~landmark_alpha_unconfirmed", 0.20)
        self.landmark_alpha_confirmed = rospy.get_param("~landmark_alpha_confirmed", 0.05)

        # Final global fusion
        self.final_fuse_radius = rospy.get_param("~final_fuse_radius", 0.65)
        self.final_fuse_recent_sec = rospy.get_param("~final_fuse_recent_sec", 20.0)

        # Optional central ROI to reduce false positives
        self.use_center_roi = rospy.get_param("~use_center_roi", False)
        self.center_roi_w_ratio = rospy.get_param("~center_roi_w_ratio", 0.8)
        self.center_roi_h_ratio = rospy.get_param("~center_roi_h_ratio", 0.8)

        # Visualization
        self.debug_view = rospy.get_param("~debug_view", True)
        self.debug_image_topic = rospy.get_param("~debug_image_topic", "/percep/debug_image")
        self.debug_publish_rate = rospy.get_param("~debug_publish_rate", self.rate)
        self._last_debug_pub_time = 0.0

        # Output
        self.records_topic = rospy.get_param("~records_topic", "/percep/numbers")
        self.target_pose_topic = rospy.get_param("~target_pose_topic", "/percep/pose")

        # =========================
        # Internal State
        # =========================
        self.bridge = CvBridge()
        self.img_curr = None
        self.curr_odom = None
        self.scan_curr = None
        self.scan_msg_curr = None
        self.scan_params_curr = None
        self.map_msg = None
        self.map_data = None              # int8[h, w]
        self.map_free_mask = None         # bool[h, w]
        self.map_dist_m = None            # float[h, w], 到最近障碍/未知的距离(米)
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.map_width = None
        self.map_height = None

        self.num_detect_result = [0] * 10
        self.box_tracks = []
        self.next_track_id = 0
        self.pending_observations = []
        
        self._prev_rot_mode = "normal"
        self.box_slots = []
        self.next_box_slot_id = 0
        self.counts = {i: 0 for i in range(10)}
        self.read_counts = {i: 0 for i in range(10)}
        self.most_read_digit = None
        self.most_read_count = 0
        self.read_event_cooldown = rospy.get_param("~read_event_cooldown", 0.5)
        self._map_update_pending = False
        self._last_map_seq = -1
        self._last_robot_map_x = None
        self._last_robot_map_y = None
        self._pose_jump_thresh = rospy.get_param("~pose_jump_thresh", 0.3)  # 米
        self._loop_recovery_until = 0.0
        self._ocr_result = []
        self._ocr_input_frame = None
        self._ocr_lock = threading.Lock()
        self._ocr_thread = threading.Thread(target=self._ocr_worker, daemon=True)
        self._ocr_thread.start()

        rospy.loginfo("Waiting for camera info on %s ...", self.camera_info_topic)
        camera_info = rospy.wait_for_message(self.camera_info_topic, CameraInfo)
        self.intrinsic = np.array(camera_info.K).reshape(3, 3)
        self.projection = np.array(camera_info.P).reshape(3, 4)
        self.distortion = np.array(camera_info.D)

        if self.camera_frame_override:
            self.img_frame = self.camera_frame_override
        else:
            self.img_frame = camera_info.header.frame_id

        self.ocr_detector = easyocr.Reader(["en"], gpu=self.use_gpu)
        self.tf_listener = tf.TransformListener()

        self.records_pub = rospy.Publisher(self.records_topic, String, queue_size=1)
        self.target_pose_pub = rospy.Publisher(self.target_pose_topic, PoseStamped, queue_size=1)
        self.debug_image_pub = rospy.Publisher(self.debug_image_topic, Image, queue_size=1)
        self.cone_trigger_pub = rospy.Publisher(self.cone_trigger_topic, Bool, queue_size=1)

        self.img_sub = rospy.Subscriber(self.image_topic, Image, self.img_callback, queue_size=1)
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        self.enable_counting_sub = rospy.Subscriber(
            self.enable_counting_topic, Bool, self.enable_counting_callback, queue_size=1
        )
        # 冻结计数 topic
        self.freeze_counts_topic = rospy.get_param("~freeze_counts_topic", "/percep/freeze_counts")
        self._counts_frozen = False
        self.freeze_counts_sub = rospy.Subscriber(
            self.freeze_counts_topic, Bool, self._freeze_counts_cb, queue_size=1
)

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("BoxCounterPerception initialized.")

    def odom_callback(self, msg): self.curr_odom = msg
    def map_callback(self, msg):
        self.map_msg = msg
        self.map_resolution = float(msg.info.resolution)
        self.map_width = int(msg.info.width)
        self.map_height = int(msg.info.height)
        self.map_origin_x = float(msg.info.origin.position.x)
        self.map_origin_y = float(msg.info.origin.position.y)

        arr = np.array(msg.data, dtype=np.int16).reshape((self.map_height, self.map_width))
        self.map_data = arr

        # free: 0
        # occupied: >= map_occupied_thresh
        # unknown: -1
        if self.map_unknown_is_invalid:
            obstacle_mask = (arr >= self.map_occupied_thresh) | (arr < 0)
        else:
            obstacle_mask = (arr >= self.map_occupied_thresh)

        self.map_free_mask = ~obstacle_mask

        # 距离变换：每个free格到最近障碍/unknown的距离（单位：米）
        # 对 obstacle_mask 做 EDT，需要输入“非障碍区域”为True
        self.map_dist_m = distance_transform_edt(self.map_free_mask) * self.map_resolution

        new_seq = msg.header.seq
        if self._last_map_seq >= 0 and new_seq != self._last_map_seq:
            self._map_update_pending = True
        self._last_map_seq = new_seq

    def img_callback(self, msg):
        try: self.img_curr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception: self.img_curr = None
    def scan_callback(self, msg):
        self.scan_curr = msg.ranges
        self.scan_msg_curr = msg
        self.scan_params_curr = {"angle_min": msg.angle_min, "angle_max": msg.angle_max, "angle_increment": msg.angle_increment}
    def enable_counting_callback(self, msg): self.counting_enabled = bool(msg.data)

    def _freeze_counts_cb(self, msg):
        if msg.data and not self._counts_frozen:
            self._counts_frozen = True
            self.pending_observations = []   # 清掉冻结前的短时观测缓存
            rospy.logwarn("[BoxCounter] Counts FROZEN at: %s", str(self.counts))

    def get_current_yaw_rate(self):
        if self.curr_odom is None:
            return 0.0
        try:
            return float(self.curr_odom.twist.twist.angular.z)
        except Exception:
            return 0.0

    def get_rotation_mode(self):
        """
        return:
            "normal"       : 正常运行
            "no_new_slot"  : 不新建slot，不OCR
            "freeze"       : 完全冻结，不更新slot，不OCR
        """
        yaw_rate = abs(self.get_current_yaw_rate())

        if yaw_rate > self.yaw_freeze_thresh:
            return "freeze"
        elif yaw_rate > self.yaw_no_new_slot_thresh:
            return "no_new_slot"
        else:
            return "normal"
        
    def in_loop_recovery(self):
        return rospy.Time.now().to_sec() < self._loop_recovery_until

    def trigger_loop_recovery(self, reason="unknown"):
        now = rospy.Time.now().to_sec()
        self._loop_recovery_until = max(self._loop_recovery_until, now + self.loop_recovery_duration)

        rospy.logwarn("Loop recovery triggered by %s, freeze-new/OCR for %.2fs",
                      reason, self.loop_recovery_duration)

        # 1) 先清掉未确认slot，避免历史残影继续污染
        before = len(self.box_slots)
        self.box_slots = [s for s in self.box_slots if s.get("confirmed", False)]
        removed = before - len(self.box_slots)
        if removed > 0:
            rospy.loginfo("Loop recovery: removed %d unconfirmed slots", removed)

        # 2) 对 confirmed slots 做一次更激进的重合并
        self.aggressive_merge_after_rotation(radius=self.loop_recovery_merge_radius)

        # 3) pending OCR 观测直接清空，避免旧坐标系下的pending继续写入
        self.pending_observations = []

    def get_rotation_mode_with_recovery(self):
        """
        在原有 rotation gating 基础上，再叠加 loop recovery 保护期：
        recovery 期间：
          - 不新建 slot
          - 不做 OCR
          - 但允许已有 slot 被 LiDAR 复用更新
        """
        if self.in_loop_recovery():
            return "no_new_slot"
        return self.get_rotation_mode()

    def best_digit_with_zero_guard(self, votes):
        """
        在 best_digit_from_votes 基础上，对 0 做保守处理：
        - 若 0 领先但优势不明显，则不直接给 0
        - 更倾向返回次优的非0类别
        """
        ranked = sorted([(d, votes.get(d, 0)) for d in range(10)],
                        key=lambda kv: kv[1], reverse=True)

        best_digit, best_votes = ranked[0]
        second_digit, second_votes = ranked[1]

        # # 非0，照常
        # if best_digit != 0:
        #     return best_digit, best_votes

        # # 0 票数太低，先不急着确认
        # if best_votes < (self.min_digit_votes + self.zero_digit_extra_votes):
        #     return second_digit, second_votes

        # # 0 领先优势不够，也不直接信
        # if (best_votes - second_votes) < self.zero_digit_margin:
        #     return second_digit, second_votes

        return best_digit, best_votes

    def clean_unconfirmed_slots_aggressive(self):
        """
        用于大事件后手动清理未确认slot
        """
        before = len(self.box_slots)
        self.box_slots = [s for s in self.box_slots if s.get("confirmed", False)]
        removed = before - len(self.box_slots)
        if removed > 0:
            rospy.loginfo("Aggressive clean: removed %d unconfirmed slots", removed)
        
    def check_pose_jump_and_merge(self):
        """检测 map frame 下机器人位置突变（回环校正信号）"""
        if self.curr_odom is None:
            return

        try:
            (trans, _) = self.tf_listener.lookupTransform(
                self.map_frame, "base_link", rospy.Time(0)
            )
            rx, ry = trans[0], trans[1]
        except Exception:
            return

        if self._last_robot_map_x is not None:
            jump = math.hypot(rx - self._last_robot_map_x, ry - self._last_robot_map_y)
            # if jump > self._pose_jump_thresh:
                # rospy.logwarn("Pose jump detected: %.3fm", jump)
                # self.trigger_loop_recovery(reason="pose_jump")

        self._last_robot_map_x, self._last_robot_map_y = rx, ry

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.img_curr is None or self.scan_curr is None or self.scan_params_curr is None or self.scan_msg_curr is None:
                rate.sleep()
                continue

            frame = self.img_curr.copy()
            vis_frame = frame.copy()

            self.check_pose_jump_and_merge()

            # if getattr(self, '_map_update_pending', False):
            #     self._map_update_pending = False
            #     rospy.loginfo("Map updated (possible loop closure)")
            #     self.trigger_loop_recovery(reason="map_update")

            rot_mode = self.get_rotation_mode_with_recovery()

            # 检测旋转结束边沿
            if self._prev_rot_mode != "normal" and rot_mode == "normal":
                self.aggressive_merge_after_rotation()
            self._prev_rot_mode = rot_mode

            # freeze 后不再更新任何 box slot，保留冻结时的可视化结果
            if (not self._counts_frozen) and rot_mode != "freeze":
                self.update_box_slots_from_lidar(allow_new_slot=(rot_mode == "normal"))
                self.clear_ghost_box_slots()
                self.merge_duplicate_box_slots()

            detections = []
            if self.counting_enabled and rot_mode == "normal":
                with self._ocr_lock:
                    self._ocr_input_frame = frame.copy()
                    detections = list(self._ocr_result)
            else:
                detections = []

            if self.enable_cone_trigger and self.counting_enabled and (not self._counts_frozen):
                cone_detections, cone_mask = self.detect_cones(frame)
                if len(cone_detections) > 0:
                    if self.debug_view:
                        for det in cone_detections:
                            x1, y1, x2, y2 = det["bbox"]
                            cv2.rectangle(vis_frame, (x1, y1), (x2, y2), (0, 140, 255), 2)
                    self.maybe_trigger_cone_open(cone_detections[0], vis_frame)

            for det in detections:
                digit = det["digit"]
                score = det["score"]
                bbox = det["bbox"]

                x1, y1, x2, y2 = bbox
                center_u = 0.5 * (x1 + x2)
                center_v = y2 - 0.1 * (y2 - y1)

                yaw_lidar = self.compute_bearing_in_lidar(center_u, center_v)
                if yaw_lidar is None: continue

                distance, beam_idx = self.get_scan_range_by_yaw(yaw_lidar)
                if distance is None: continue

                distance = distance - self.range_offset
                if distance < self.front_range_min or distance > self.front_range_max: continue

                map_x, map_y = self.project_detection_to_map(distance, yaw_lidar)
                if map_x is None:
                    continue

                if self.use_floor_filter and not self.is_valid_floor_point(map_x, map_y):
                    continue

                map_x, map_y = self.maybe_project_slot_to_valid_free(map_x, map_y)
                if map_x is None:
                    continue
                if not self.counting_enabled: continue

                if not self.counting_enabled:
                    continue

                stable_obs = self.update_pending_observation(digit, map_x, map_y, score)
                if stable_obs is None:
                    continue

                # =========================
                # freeze 后：不再更新 slot / counts
                # 只把“当前稳定看到的数字”发布出去，供 switcher 判断是否进入房间
                # =========================
                if self._counts_frozen:
                    self.current_seen_digit_pub.publish(Int32(data=int(stable_obs["digit"])))
                    continue

                # =========================
                # freeze 前：正常计数流程
                # =========================
                slot = self.assign_digit_to_box_slot(stable_obs)

                if slot is not None and slot["assigned_digit"] is not None:
                    self.current_seen_digit_pub.publish(Int32(data=int(slot["assigned_digit"])))

                    goal_p = PoseStamped()
                    goal_p.header.frame_id = self.map_frame
                    goal_p.header.stamp = rospy.Time.now()
                    goal_p.pose.position.x = slot["x"]
                    goal_p.pose.position.y = slot["y"]
                    goal_p.pose.position.z = 0.0
                    if self.curr_odom is not None:
                        goal_p.pose.orientation = self.curr_odom.pose.pose.orientation
                    else:
                        goal_p.pose.orientation.w = 1.0
                    self.target_pose_pub.publish(goal_p)

            if not self._counts_frozen:
                self.recompute_counts()
            self.publish_summary()

            if self.debug_view:
                self.draw_tracks(vis_frame)
                self.publish_debug_image(vis_frame)

            rate.sleep()

    def aggressive_merge_after_rotation(self, radius=None):
        if radius is None:
            radius = self.confirmed_reuse_radius  # 旋转后：0.55m

        # radius=0.70m 用于回环校正
        # 0.70m < 箱子直径(0.8m)，不会把相邻箱子合并掉

        merged = []
        slots = sorted(self.box_slots,
                    key=lambda s: (s["confirmed"], s["assigned_votes"], s["hits"]),
                    reverse=True)

        for slot in slots:
            duplicate = False
            for kept in merged:
                if math.hypot(slot["x"] - kept["x"], slot["y"] - kept["y"]) < radius:
                    for d in range(10):
                        kept["digit_votes"][d] += slot["digit_votes"].get(d, 0)
                    # 只合并几何与生命周期信息，不盲目累加 digit_votes
                    kept["hits"] = max(kept["hits"], slot["hits"])
                    kept["confirmed"] = kept["hits"] >= self.box_slot_confirm_hits
                    kept["counted_once"] = kept["counted_once"] or slot.get("counted_once", False)

                    # 选择数字证据更可信的那个 slot
                    kept_conf = (kept.get("assigned_votes", 0), kept.get("hits", 0), int(kept.get("confirmed", False)))
                    slot_conf = (slot.get("assigned_votes", 0), slot.get("hits", 0), int(slot.get("confirmed", False)))

                    if slot_conf > kept_conf:
                        kept["digit_votes"] = dict(slot["digit_votes"])
                        kept["assigned_digit"] = slot.get("assigned_digit", None)
                        kept["assigned_votes"] = slot.get("assigned_votes", 0)
                    # 否则保留 kept 原有 digit_votes，不叠加弱证据
                    duplicate = True
                    break
            if not duplicate:
                merged.append(slot)

        removed = len(self.box_slots) - len(merged)
        if removed > 0:
            rospy.loginfo("Merge (r=%.2fm): removed %d duplicate slots", radius, removed)
        self.box_slots = merged

    def finalize_landmarks(self, radius=None, recent_sec=None):
        """
        绕图结束后做一次全局 landmark 融合。
        目标：
        - 合并空间上接近的 confirmed landmarks
        - 位置做加权平均，不再简单保留其一
        - digit_votes 不盲目求和，优先保留更可信的语义证据
        """
        if radius is None:
            radius = self.final_fuse_radius
        if recent_sec is None:
            recent_sec = self.final_fuse_recent_sec

        now = rospy.Time.now().to_sec()

        # 只保留活着的 slot；优先融合 confirmed，最近看见过的优先
        candidates = []
        for s in self.box_slots:
            if s.get("hits", 0) < 0:
                continue
            age = now - s.get("t_last_seen", now)
            if s.get("confirmed", False) or age < recent_sec:
                candidates.append(s)

        # 排序：confirmed > assigned_votes > hits
        candidates = sorted(
            candidates,
            key=lambda s: (
                int(s.get("confirmed", False)),
                s.get("assigned_votes", 0),
                s.get("hits", 0)
            ),
            reverse=True
        )

        fused = []

        def _semantic_strength(slot):
            return (
                int(slot.get("confirmed", False)),
                slot.get("assigned_votes", 0),
                slot.get("hits", 0)
            )

        for slot in candidates:
            matched = False

            for kept in fused:
                d = math.hypot(slot["x"] - kept["x"], slot["y"] - kept["y"])
                if d >= radius:
                    continue

                # 如果两个 slot 都有数字，且数字冲突明显，则保守些，不强行融合
                kd = kept.get("assigned_digit", None)
                sd = slot.get("assigned_digit", None)
                kv = kept.get("assigned_votes", 0)
                sv = slot.get("assigned_votes", 0)

                strong_conflict = (
                    kd is not None and sd is not None and
                    kd != sd and kv >= self.min_digit_votes and sv >= self.min_digit_votes
                )
                if strong_conflict:
                    continue

                # 位置融合：按 hits + assigned_votes 加权平均
                wk = max(1.0, kept.get("hits", 1) + 0.5 * kept.get("assigned_votes", 0))
                ws = max(1.0, slot.get("hits", 1) + 0.5 * slot.get("assigned_votes", 0))

                new_x = (wk * kept["x"] + ws * slot["x"]) / (wk + ws)
                new_y = (wk * kept["y"] + ws * slot["y"]) / (wk + ws)
                new_x, new_y = self.maybe_project_slot_to_valid_free(new_x, new_y)

                if new_x is not None:
                    kept["x"] = float(new_x)
                    kept["y"] = float(new_y)

                # 生命周期信息融合
                kept["hits"] = max(kept.get("hits", 0), slot.get("hits", 0))
                kept["confirmed"] = kept["hits"] >= self.box_slot_confirm_hits
                kept["t_last_seen"] = max(
                    kept.get("t_last_seen", 0.0),
                    slot.get("t_last_seen", 0.0)
                )
                kept["counted_once"] = kept.get("counted_once", False) or slot.get("counted_once", False)

                # 语义融合：不盲加 digit_votes，优先保留更强证据
                if _semantic_strength(slot) > _semantic_strength(kept):
                    kept["digit_votes"] = dict(slot.get("digit_votes", self.empty_votes()))
                    kept["assigned_digit"] = slot.get("assigned_digit", None)
                    kept["assigned_votes"] = slot.get("assigned_votes", 0)
                else:
                    # 若 kept 更强，仅对 kept 当前主类做非常弱的补强，而不全量叠加
                    kd2 = kept.get("assigned_digit", None)
                    if kd2 is not None:
                        kept["digit_votes"][kd2] = kept["digit_votes"].get(kd2, 0) + min(
                            1, slot.get("digit_votes", {}).get(kd2, 0)
                        )
                        kept["assigned_digit"], kept["assigned_votes"] = \
                            self.best_digit_from_votes(kept["digit_votes"])

                matched = True
                break

            if not matched:
                fused.append({
                    "id": slot["id"],
                    "x": float(slot["x"]),
                    "y": float(slot["y"]),
                    "hits": int(slot.get("hits", 0)),
                    "confirmed": bool(slot.get("confirmed", False)),
                    "t_last_seen": float(slot.get("t_last_seen", now)),
                    "digit_votes": dict(slot.get("digit_votes", self.empty_votes())),
                    "assigned_digit": slot.get("assigned_digit", None),
                    "assigned_votes": int(slot.get("assigned_votes", 0)),
                    "counted_once": bool(slot.get("counted_once", False)),
                })

        removed = len(self.box_slots) - len(fused)
        self.box_slots = fused
        self.recompute_counts()

        rospy.loginfo("Finalize landmarks (r=%.2fm): removed %d duplicate landmarks",
                      radius, removed)

    def detect_digits(self, image):
        draw = image.copy()
        if self.use_center_roi:
            h, w = image.shape[:2]
            roi_w = int(w * self.center_roi_w_ratio)
            roi_h = int(h * self.center_roi_h_ratio)
            x0 = (w - roi_w) // 2
            y0 = (h - roi_h) // 2
            crop = image[y0:y0 + roi_h, x0:x0 + roi_w]
            offset_x, offset_y = x0, y0
            cv2.rectangle(draw, (x0, y0), (x0 + roi_w, y0 + roi_h), (255, 0, 0), 2)
        else:
            crop = image
            offset_x, offset_y = 0, 0

        results = self.ocr_detector.readtext(crop, batch_size=2, allowlist="0123456789")
        detections = []

        for detection in results:
            pts, text, conf = detection[0], detection[1], float(detection[2])
            if len(text) > self.max_text_len or len(text) != 1 or not text.isdigit() or conf < self.ocr_conf_thresh:
                continue

            diag_vec = np.array(pts[2]) - np.array(pts[0])
            if np.linalg.norm(diag_vec) < self.min_diag_len: continue

            x1, y1 = int(pts[0][0]) + offset_x, int(pts[0][1]) + offset_y
            x2, y2 = int(pts[2][0]) + offset_x, int(pts[2][1]) + offset_y
            detections.append({"digit": int(text), "score": conf, "bbox": [x1, y1, x2, y2]})

        return detections, draw
    
    def _ocr_worker(self):
        while not rospy.is_shutdown():
            frame = None
            with self._ocr_lock:
                if self._ocr_input_frame is not None:
                    frame = self._ocr_input_frame.copy()
                    self._ocr_input_frame = None
            if frame is None:
                rospy.sleep(0.02)
                continue
            detections, _ = self.detect_digits(frame)
            with self._ocr_lock:
                self._ocr_result = detections

    def compute_bearing_in_lidar(self, u, v):
        direction = np.array([[u], [v], [1.0]], dtype=np.float64)
        try: direction = np.dot(np.linalg.inv(self.intrinsic), direction)
        except np.linalg.LinAlgError: return None

        p_in_cam = PoseStamped()
        p_in_cam.header.frame_id = self.img_frame
        p_in_cam.pose.position.x = float(direction[0].item())
        p_in_cam.pose.position.y = float(direction[1].item())
        p_in_cam.pose.position.z = float(direction[2].item())
        p_in_cam.pose.orientation.w = 1.0

        try:
            # self.tf_listener.waitForTransform(self.lidar_frame, self.img_frame, rospy.Time(0), rospy.Duration(0.05))
            transformed = self.tf_listener.transformPose(self.lidar_frame, p_in_cam)
            return math.atan2(transformed.pose.position.y, transformed.pose.position.x)
        except Exception: return None

    def get_scan_range_by_yaw(self, yaw):
        angle_min = self.scan_params_curr["angle_min"]
        angle_max = self.scan_params_curr["angle_max"]
        angle_inc = self.scan_params_curr["angle_increment"]

        if yaw < angle_min or yaw > angle_max: return None, None
        idx_center = int(round((yaw - angle_min) / angle_inc))
        if idx_center < 0 or idx_center >= len(self.scan_curr): return None, None

        best_r, best_idx = None, None
        start_idx = max(0, idx_center - self.search_half_window)
        end_idx = min(len(self.scan_curr) - 1, idx_center + self.search_half_window)

        for idx in range(start_idx, end_idx + 1):
            r = self.scan_curr[idx]
            if np.isfinite(r) and self.front_range_min <= r <= self.front_range_max:
                if best_r is None or r < best_r:
                    best_r, best_idx = r, idx

        return (float(best_r), int(best_idx)) if best_r else (None, None)

    def project_detection_to_map(self, distance, yaw):
        p_in_lidar = PoseStamped()
        p_in_lidar.header.frame_id = self.lidar_frame
        p_in_lidar.pose.position.x = distance * math.cos(yaw)
        p_in_lidar.pose.position.y = distance * math.sin(yaw)
        p_in_lidar.pose.orientation.w = 1.0

        try:
            # self.tf_listener.waitForTransform(self.map_frame, self.lidar_frame, rospy.Time(0), rospy.Duration(0.05))
            p_in_map = self.tf_listener.transformPose(self.map_frame, p_in_lidar)
            x, y = p_in_map.pose.position.x, p_in_map.pose.position.y
            return (float(x), float(y)) if np.isfinite(x) and np.isfinite(y) else (None, None)
        except Exception: return None, None

    def project_lidar_point_to_map(self, x_l, y_l):
        p_in_lidar = PointStamped()
        p_in_lidar.header.frame_id = self.lidar_frame
        p_in_lidar.point.x, p_in_lidar.point.y = x_l, y_l

        try:
            # self.tf_listener.waitForTransform(self.map_frame, self.lidar_frame, rospy.Time(0), rospy.Duration(0.05))
            p_in_map = self.tf_listener.transformPoint(self.map_frame, p_in_lidar)
            x, y = p_in_map.point.x, p_in_map.point.y
            return (float(x), float(y)) if np.isfinite(x) and np.isfinite(y) else (None, None)
        except Exception: return None, None

    def transform_map_point_to_lidar(self, map_x, map_y):
        p_in_map = PointStamped()
        p_in_map.header.frame_id = self.map_frame
        p_in_map.header.stamp = rospy.Time(0)
        p_in_map.point.x = map_x
        p_in_map.point.y = map_y
        p_in_map.point.z = 0.0

        try:
            self.tf_listener.waitForTransform(
                self.lidar_frame,
                self.map_frame,
                rospy.Time(0),
                rospy.Duration(0.1) # 逆变换要快，不阻塞
            )
            p_in_lidar = self.tf_listener.transformPoint(self.lidar_frame, p_in_map)
            return p_in_lidar.point.x, p_in_lidar.point.y
        except Exception:
            return None, None

    def detect_cones(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower = np.array([self.cone_h_low, self.cone_s_low, self.cone_v_low], dtype=np.uint8)
        upper = np.array([self.cone_h_high, self.cone_s_high, self.cone_v_high], dtype=np.uint8)

        mask = cv2.morphologyEx(cv2.inRange(hsv, lower, upper), cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (9, 31)))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.cone_min_area: continue
            x, y, w, h = cv2.boundingRect(cnt)
            if h <= 0 or y + h < 0.45 * image.shape[0]: continue
            aspect = float(w) / float(h)
            if self.cone_min_aspect <= aspect <= self.cone_max_aspect:
                detections.append({"bbox": [x, y, x + w, y + h], "area": float(area), "bottom": float(y + h)})

        detections.sort(key=lambda d: (d["bottom"], d["bbox"], d["area"]), reverse=True)
        return detections, mask

    def update_box_slots_from_lidar(self, allow_new_slot=True):
        msg = self.scan_msg_curr
        if msg is None: return

        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if np.isfinite(r) and self.front_range_min <= r <= self.front_range_max:
                points.append((r * math.cos(angle), r * math.sin(angle)))
            angle += msg.angle_increment

        if len(points) < 3: return

        clusters = []
        curr = [points[0]]
        for i in range(1, len(points)):
            px, py = points[i - 1]
            qx, qy = points[i]
            if math.hypot(qx - px, qy - py) < self.box_slot_cluster_radius:
                curr.append((qx, qy))
            else:
                if len(curr) >= 3: clusters.append(curr)
                curr = [(qx, qy)]
        if len(curr) >= 3: clusters.append(curr)

        # 改良的形状校验逻辑：摒弃极度依赖角度的 AABB 长宽比，转为使用真实物理跨度与轮廓长度
        for cluster in clusters:
            xs = [p[0] for p in cluster]
            ys = [p[1] for p in cluster]
            
            # 计算最大跨度 Span (首尾直线距离)
            span = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
            
            # 计算轮廓长度 Arc Length (沿聚类表面走线)
            arc_length = 0.0
            for i in range(1, len(cluster)):
                arc_length += math.hypot(xs[i] - xs[i-1], ys[i] - ys[i-1])

            cx = float(np.mean(xs))
            cy = float(np.mean(ys))

            # 惩罚删除机制：如果在 Box 附近扫到了极其宽广的障碍物 (比如大墙面/大车)，则销毁该候选框
            if span > self.box_size_max or arc_length > self.box_max_arc:
                mx, my = self.project_lidar_point_to_map(cx, cy)
                if mx is not None:
                    self.penalize_box_slot(mx, my)
                continue

            # 太小则属于噪点
            if arc_length < self.box_size_min:
                continue

            mx, my = self.project_lidar_point_to_map(cx, cy)
            if mx is None:
                continue

            if self.use_floor_filter and not self.is_valid_floor_point(mx, my):
                continue

            mx2, my2 = self.maybe_project_slot_to_valid_free(mx, my)
            if mx2 is None:
                continue

            self.insert_or_update_box_slot(mx2, my2, allow_new_slot=allow_new_slot)

    def clear_ghost_box_slots(self):
        """
        利用当前雷达扫描作为“负面证据”清除鬼影。
        如果雷达视野中本该有 Box 的地方其实是空的（激光打到了更远处），则大幅扣减其 hits。
        """
        if self.scan_curr is None or self.scan_params_curr is None:
            return

        angle_min = self.scan_params_curr["angle_min"]
        angle_max = self.scan_params_curr["angle_max"]
        angle_inc = self.scan_params_curr["angle_increment"]

        for slot in self.box_slots:
            if slot["hits"] < 0:
                continue # 已死亡，跳过

            # 1. 将 Box 转换到当前雷达坐标系下
            lx, ly = self.transform_map_point_to_lidar(slot["x"], slot["y"])
            if lx is None:
                continue
            
            # 2. 计算 Box 距离当前雷达的距离和角度
            dist_to_slot = math.hypot(lx, ly)
            angle_to_slot = math.atan2(ly, lx)

            # 如果 Box 不在雷达视野范围内，或者距离太远（比如大于4米），不作评判
            if angle_to_slot < angle_min or angle_to_slot > angle_max:
                continue
            if dist_to_slot > 4.0 or dist_to_slot < self.front_range_min:
                continue

            # 3. 找到对应角度的那根雷达光束
            idx = int(round((angle_to_slot - angle_min) / angle_inc))
            if 0 <= idx < len(self.scan_curr):
                actual_laser_dist = self.scan_curr[idx]
                
                if not np.isfinite(actual_laser_dist):
                    continue

                # 4. 核心逻辑：如果实际激光测距 减去 Box理论距离 > 容差(例如 0.5m)
                # 意味着激光“穿透”了本该有 Box 的地方，打到了后面的墙上 -> 绝对是鬼影！
                if actual_laser_dist > dist_to_slot + 0.5:
                    # 发现鬼影，重拳出击（一次扣除多次 hits，加速死亡）
                    slot["hits"] -= 3
                    
                    # 取消确认状态，如果扣到负数，会在 merge 阶段被彻底清理
                    if slot["hits"] < self.box_slot_confirm_hits:
                        slot["confirmed"] = False

    def is_valid_floor_point(self, x, y):
        return self.floor_x_min <= x <= self.floor_x_max and self.floor_y_min <= y <= self.floor_y_max

    def has_valid_occupancy_map(self):
        return (
            self.map_data is not None and
            self.map_free_mask is not None and
            self.map_dist_m is not None and
            self.map_resolution is not None
        )

    def world_to_map_rc(self, x, y):
        """
        world (x, y) -> map array index (row, col)
        """
        if not self.has_valid_occupancy_map():
            return None, None

        col = int((x - self.map_origin_x) / self.map_resolution)
        row = int((y - self.map_origin_y) / self.map_resolution)

        if row < 0 or row >= self.map_height or col < 0 or col >= self.map_width:
            return None, None
        return row, col

    def map_rc_to_world(self, row, col):
        """
        map array index (row, col) -> world (x, y)
        返回该栅格中心点坐标
        """
        if not self.has_valid_occupancy_map():
            return None, None

        x = self.map_origin_x + (col + 0.5) * self.map_resolution
        y = self.map_origin_y + (row + 0.5) * self.map_resolution
        return float(x), float(y)

    def is_valid_map_slot(self, x, y):
        """
        判断某点是否位于合法free区域，并且离障碍/unknown有最小安全距离
        """
        if not self.use_occupancy_map_constraint:
            return True

        if not self.has_valid_occupancy_map():
            return True

        row, col = self.world_to_map_rc(x, y)
        if row is None:
            return False

        # 必须在free区域
        if not self.map_free_mask[row, col]:
            return False

        # 与障碍保持最小距离
        if self.map_dist_m[row, col] < self.map_min_free_clearance:
            return False

        return True

    def snap_to_nearest_free(self, x, y, max_radius_m=None):
        """
        若点落在非法位置，尝试吸附到附近最近合法free格。
        返回 (new_x, new_y)；若失败返回 (None, None)
        """
        if max_radius_m is None:
            max_radius_m = self.map_snap_search_radius

        if not self.use_occupancy_map_constraint:
            return x, y

        if not self.has_valid_occupancy_map():
            return x, y

        row0, col0 = self.world_to_map_rc(x, y)
        if row0 is None:
            return None, None

        max_r = int(max_radius_m / self.map_resolution)
        best = None
        best_d2 = 1e18

        rmin = max(0, row0 - max_r)
        rmax = min(self.map_height - 1, row0 + max_r)
        cmin = max(0, col0 - max_r)
        cmax = min(self.map_width - 1, col0 + max_r)

        for r in range(rmin, rmax + 1):
            for c in range(cmin, cmax + 1):
                if not self.map_free_mask[r, c]:
                    continue
                if self.map_dist_m[r, c] < self.map_min_free_clearance:
                    continue

                d2 = (r - row0) ** 2 + (c - col0) ** 2
                if d2 < best_d2:
                    best_d2 = d2
                    best = (r, c)

        if best is None:
            return None, None

        return self.map_rc_to_world(best[0], best[1])

    def maybe_project_slot_to_valid_free(self, x, y):
        """
        若点已合法，直接返回。
        若非法且允许snap，则吸附到最近free。
        若失败，返回 (None, None)
        """
        if self.is_valid_map_slot(x, y):
            return x, y

        if self.map_snap_to_free:
            return self.snap_to_nearest_free(x, y, self.map_snap_search_radius)

        return None, None
    
    def penalize_box_slot(self, x, y):
        """对不符合箱子特征的庞大障碍物周边的旧槽位进行扣分"""
        for slot in self.box_slots:
            dist = math.hypot(slot["x"] - x, slot["y"] - y)
            if dist < self.box_slot_merge_radius:
                slot["hits"] -= 2  # 重度扣分
                if slot["hits"] < 0:
                    slot["hits"] = -1  # 标记为死亡，待合并阶段清理
                slot["confirmed"] = slot["hits"] >= self.box_slot_confirm_hits

    def insert_or_update_box_slot(self, x, y, allow_new_slot=True):
        """
        将 box slot 视为全局静态 landmark，使用小步长加权更新，而不是硬锁死。
        逻辑：
        1. 新观测先过 occupancy 合法性约束
        2. 先找 confirmed landmark（更可信）
        3. 再找普通 landmark
        4. 若找到，则做小步长位置更新
        5. 若找不到，且允许新建，则创建新 landmark
        """
        x, y = self.maybe_project_slot_to_valid_free(x, y)
        if x is None:
            return None

        now = rospy.Time.now().to_sec()

        def _slot_score(slot):
            # 分数越小越好；confirmed 优先、hits 高优先、距离近优先
            d = math.hypot(slot["x"] - x, slot["y"] - y)
            conf_bonus = 0.15 if slot.get("confirmed", False) else 0.0
            hit_bonus = min(slot.get("hits", 0), 20) * 0.005
            return d - conf_bonus - hit_bonus

        best_slot = None
        best_score = 1e9

        # 1) 优先关联 confirmed landmark
        for slot in self.box_slots:
            if slot.get("hits", 0) < 0:
                continue
            if not slot.get("confirmed", False):
                continue
            d = math.hypot(slot["x"] - x, slot["y"] - y)
            if d < self.landmark_confirmed_assoc_radius:
                s = _slot_score(slot)
                if s < best_score:
                    best_score = s
                    best_slot = slot

        # 2) 若没有 confirmed 命中，再找普通 slot
        if best_slot is None:
            for slot in self.box_slots:
                if slot.get("hits", 0) < 0:
                    continue
                d = math.hypot(slot["x"] - x, slot["y"] - y)
                if d < self.landmark_assoc_radius:
                    s = _slot_score(slot)
                    if s < best_score:
                        best_score = s
                        best_slot = slot

        # 3) 找不到则新建
        if best_slot is None:
            if not allow_new_slot:
                return None

            self.box_slots.append({
                "id": self.next_box_slot_id,
                "x": float(x),
                "y": float(y),
                "hits": 1,
                "confirmed": False,
                "t_last_seen": now,
                "digit_votes": self.empty_votes(),
                "assigned_digit": None,
                "assigned_votes": 0,
                "counted_once": False,
            })
            self.next_box_slot_id += 1
            return self.box_slots[-1]

        # 4) 命中已有 landmark：做小步长更新，而不是完全锁死
        best_slot["hits"] += 1
        best_slot["t_last_seen"] = now
        best_slot["confirmed"] = best_slot["hits"] >= self.box_slot_confirm_hits

        old_x, old_y = best_slot["x"], best_slot["y"]

        if best_slot["confirmed"]:
            alpha = self.landmark_alpha_confirmed
        else:
            alpha = self.landmark_alpha_unconfirmed

        new_x = (1.0 - alpha) * best_slot["x"] + alpha * x
        new_y = (1.0 - alpha) * best_slot["y"] + alpha * y

        new_x, new_y = self.maybe_project_slot_to_valid_free(new_x, new_y)
        if new_x is not None:
            best_slot["x"] = new_x
            best_slot["y"] = new_y
        else:
            best_slot["x"], best_slot["y"] = old_x, old_y

        return best_slot
    
    def merge_duplicate_box_slots(self):
        # 先清理 hits 降为负数的死亡槽位 (即后续被判定为大墙面等非 Box 形状的)
        now = rospy.Time.now().to_sec()
        unconfirmed_ttl = rospy.get_param("~unconfirmed_slot_ttl", 3.0)
        alive_slots = [s for s in self.box_slots if s["hits"] >= 0 and (
            s["confirmed"] or
            (now - s.get("t_last_seen", now)) < unconfirmed_ttl
        )]
        
        merged = []
        slots = sorted(alive_slots, key=lambda s: s["hits"], reverse=True)

        for slot in slots:
            duplicate = False
            for kept in merged:
                if math.hypot(slot["x"] - kept["x"], slot["y"] - kept["y"]) < self.box_slot_duplicate_radius:
                    duplicate = True
                    # 融合两者的 hits（可选）
                    kept["hits"] = max(kept["hits"], slot["hits"])
                    kept["confirmed"] = kept["hits"] >= self.box_slot_confirm_hits
                    break
            if not duplicate:
                merged.append(slot)

        self.box_slots = merged

    def empty_votes(self): return {i: 0 for i in range(10)}

    def best_digit_from_votes(self, votes):
        best_digit, best_votes = 0, -1
        for d in range(10):
            if votes[d] > best_votes: best_digit, best_votes = d, votes[d]
        return best_digit, best_votes

    def update_pending_observation(self, digit, x, y, score):
        now = rospy.Time.now().to_sec()
        self.pending_observations = [i for i in self.pending_observations if now - i["t_last"] <= self.same_obs_time_window]

        best_item, best_dist = None, 1e9
        for item in self.pending_observations:
            dist = math.hypot(item["x"] - x, item["y"] - y)
            if dist < self.pending_match_radius and dist < best_dist:
                best_item, best_dist = item, dist

        if best_item is None:
            item = {"x": float(x), "y": float(y), "score": float(score), "hits": 1, "t_last": now, "votes": self.empty_votes()}
            item["votes"][digit] += 1
            self.pending_observations.append(item)
            return None

        best_item["x"] = 0.8 * best_item["x"] + 0.2 * x
        best_item["y"] = 0.8 * best_item["y"] + 0.2 * y
        best_item["score"] = max(best_item["score"], score)
        best_item["hits"] += 1
        best_item["t_last"] = now
        best_item["votes"][digit] += 1

        assigned_digit, assigned_votes = self.best_digit_from_votes(best_item["votes"])
        if best_item["hits"] >= self.required_stable_hits and assigned_votes >= self.min_digit_votes:
            return {
                "x": float(best_item["x"]), "y": float(best_item["y"]), "score": float(best_item["score"]),
                "digit": int(assigned_digit), "votes": dict(best_item["votes"]), "hits": int(best_item["hits"])
            }
        return None

    def maybe_trigger_cone_open(self, det, vis_frame=None):
        # 只允许全局第一次 unblock 生效
        if self._unblock_sent_once:
            return False

        now = rospy.Time.now().to_sec()
        if now - self.last_cone_trigger_time < self.cone_trigger_cooldown:
            return False

        yaw_lidar = self.compute_bearing_in_lidar(
            0.5 * (det["bbox"][0] + det["bbox"][2]),
            det["bbox"][3] - 0.1 * (det["bbox"][3] - det["bbox"][1])
        )
        if yaw_lidar is None:
            return False

        distance, _ = self.get_scan_range_by_yaw(yaw_lidar)
        if distance is None or distance > self.cone_trigger_distance:
            return False

        self.cone_trigger_pub.publish(Bool(data=True))
        self.last_cone_trigger_time = now
        self._unblock_sent_once = True
        rospy.logwarn("[BoxCounter] First unblock sent. Future cone triggers will be ignored.")
        return True

    def update_most_read_digit(self):
        max_count = max(self.read_counts.values()) if self.read_counts else 0
        if max_count <= 0:
            self.most_read_digit, self.most_read_count = None, 0
            return
        self.most_read_digit = min([d for d, c in self.read_counts.items() if c == max_count])
        self.most_read_count = max_count

    def assign_digit_to_box_slot(self, obs):
        best_slot, best_dist = None, 1e9
        for slot in self.box_slots:
            if not slot["confirmed"]:
                continue
            if not self.is_valid_map_slot(slot["x"], slot["y"]):
                continue

            dist = math.hypot(slot["x"] - obs["x"], slot["y"] - obs["y"])
            if dist < self.slot_assign_radius and dist < best_dist:
                best_slot, best_dist = slot, dist

        if best_slot is None: return None

        for d in range(10): best_slot["digit_votes"][d] += obs["votes"].get(d, 0)
        assigned_digit, assigned_votes = self.best_digit_from_votes(best_slot["digit_votes"])
        best_slot["assigned_digit"] = int(assigned_digit)
        best_slot["assigned_votes"] = int(assigned_votes)

        if not best_slot["counted_once"] and assigned_votes >= self.min_digit_votes:
            self.read_counts[assigned_digit] += 1
            best_slot["counted_once"] = True
            self.update_most_read_digit()

        return best_slot

    def recompute_counts(self):
        self.counts = {i: 0 for i in range(10)}
        for slot in self.box_slots:
            if slot["confirmed"] and slot["assigned_digit"] is not None:
                if 0 <= int(slot["assigned_digit"]) <= 9:
                    self.counts[int(slot["assigned_digit"])] += 1
        self.num_detect_result = [1 if self.counts[i] > 0 else 0 for i in range(10)]

    def publish_summary(self):
        msg = {
            "counts": self.counts, "read_counts": self.read_counts,
            "most_read_digit": self.most_read_digit, "most_read_count": self.most_read_count,
            "box_slots": self.box_slots,
        }
        self.records_pub.publish(String(data=json.dumps(msg, ensure_ascii=False)))

    def publish_debug_image(self, image_bgr):
        if image_bgr is None:
            return

        now = rospy.Time.now().to_sec()
        if self.debug_publish_rate > 0.0:
            min_interval = 1.0 / float(self.debug_publish_rate)
            if (now - self._last_debug_pub_time) < min_interval:
                return

        try:
            msg = self.bridge.cv2_to_imgmsg(image_bgr, encoding="bgr8")
            msg.header.stamp = rospy.Time.now()
            self.debug_image_pub.publish(msg)
            self._last_debug_pub_time = now
        except Exception:
            pass

    def draw_tracks(self, image):
        overlay = image.copy()
        cv2.rectangle(overlay, (10, 10), (220, 310), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, image, 0.4, 0, image)
        y = 35
        cv2.putText(image, "Map Digit Counts:", (20, y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
        y += 25
        for i in range(10):
            count = self.counts.get(i, 0)
            # 出现的数字高亮显示为绿色，未出现的显示为暗灰色
            color = (0, 255, 0) if count > 0 else (100, 100, 100)
            thickness = 2 if count > 0 else 1
            cv2.putText(image, f"Digit {i}: {count} boxes", (30, y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, thickness, cv2.LINE_AA)
            y += 22
        y_slot = 30
        img_w = image.shape[1]
        for slot in self.box_slots[:15]:
            color = (0, 255, 0) if slot["confirmed"] else (0, 165, 255)
            text = f"slot={slot['id']} d={slot['assigned_digit']} v={slot['assigned_votes']}"
            cv2.putText(image, text, (img_w - 240, y_slot), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
            y_slot += 20

    def on_shutdown(self):
        self.recompute_counts()

if __name__ == "__main__":
    rospy.init_node("box_counter_perception")
    node = BoxCounterPerception()
    node.run()