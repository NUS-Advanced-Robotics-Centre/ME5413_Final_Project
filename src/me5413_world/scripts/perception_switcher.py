#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
perception_switcher.py

监听 cone trigger topic（默认 /cmd_unblock）：
  - 只接受第一次有效的 True 信号，之后忽略所有后续信号
  - 触发时：
      1. 快照锁定当前 /percep/numbers 的 counts
      2. 关闭 box_counter_perception 节点
      3. 由本节点接管 enter_room 逻辑（用锁定的 counts 判断最小值)
"""

import json
import threading
import subprocess
import rospy
from std_msgs.msg import Bool, String, Int32


class PerceptionSwitcher:
    def __init__(self):
        # ===== 参数 =====
        self.cone_trigger_topic      = rospy.get_param("~cone_trigger_topic",      "/cmd_unblock")
        self.box_counter_node_name   = rospy.get_param("~box_counter_node_name",   "/box_counter_perception")
        self.records_topic           = rospy.get_param("~records_topic",           "/percep/numbers")
        self.current_seen_digit_topic= rospy.get_param("~current_seen_digit_topic","/percep/current_seen_digit")
        self.enter_room_topic        = rospy.get_param("~enter_room_topic",        "/second_floor/enter_room")
        self.enter_trigger_topic     = rospy.get_param("~enter_trigger_topic",     "/second_floor/enter_room_trigger")

        # 不允许并列时只取最小集合里的最小数字
        self.allow_tie               = rospy.get_param("~allow_tie",               True)
        # 同一数字重复触发冷却（秒）
        self.same_digit_cooldown     = rospy.get_param("~same_digit_cooldown",     2.0)
        # 是否忽略 count=0 的数字（True=0不参与最小值竞争）
        self.ignore_zero_count       = rospy.get_param("~ignore_zero_count",       True)
        # 数字 → 房间号映射
        self.room_map                = rospy.get_param("~room_map", {
            "0": 0, "1": 1, "2": 2, "3": 3, "4": 4,
            "5": 5, "6": 6, "7": 7, "8": 8, "9": 9
        })

        # ===== 状态 =====
        self._lock          = threading.Lock()
        self._switched      = False          # 是否已完成切换
        self._live_counts   = {i: 0 for i in range(10)}   # box_counter 实时 counts
        self._locked_counts = None           # 切换时快照，之后只用这个
        self._last_trigger_time = {i: -1e9 for i in range(10)}

        # ===== 发布 =====
        self._enter_room_pub    = rospy.Publisher(self.enter_room_topic,    Int32, queue_size=1)
        self._enter_trigger_pub = rospy.Publisher(self.enter_trigger_topic, Bool,  queue_size=1)
        self.freeze_counts_topic = rospy.get_param("~freeze_counts_topic", "/percep/freeze_counts")
        self._freeze_pub = rospy.Publisher(self.freeze_counts_topic, Bool, queue_size=1, latch=True)

        # ===== 订阅 =====
        # 始终订阅 counts，保证切换时能拿到最新值
        self._records_sub = rospy.Subscriber(
            self.records_topic, String, self._records_cb, queue_size=1
        )
        # unblock trigger（只有第一次 True 有效）
        self._trigger_sub = rospy.Subscriber(
            self.cone_trigger_topic, Bool, self._trigger_cb, queue_size=1
        )
        # 当前看到的数字（切换前不处理）
        self._digit_sub = rospy.Subscriber(
            self.current_seen_digit_topic, Int32, self._digit_cb, queue_size=1
        )

        rospy.loginfo(
            "[PerceptionSwitcher] Ready. Listening on '%s'.",
            self.cone_trigger_topic
        )

    # ------------------------------------------------------------------
    # counts 实时更新（切换前）
    # ------------------------------------------------------------------
    def _records_cb(self, msg):
        if self._switched:
            return  # 切换后不再更新 live counts，已用锁定值
        try:
            data = json.loads(msg.data)
            counts_raw = data.get("counts", {})
            new_counts = {i: 0 for i in range(10)}
            for k, v in counts_raw.items():
                try:
                    d = int(k)
                    if 0 <= d <= 9:
                        new_counts[d] = int(v)
                except Exception:
                    pass
            with self._lock:
                self._live_counts = new_counts
        except Exception as e:
            rospy.logwarn_throttle(2.0, "[PerceptionSwitcher] Failed to parse counts: %s", str(e))

    # ------------------------------------------------------------------
    # unblock trigger 回调：只接受第一次
    # ------------------------------------------------------------------
    def _trigger_cb(self, msg):
        if hasattr(msg, 'data') and not msg.data:
            return

        with self._lock:
            if self._switched:
                rospy.logwarn_throttle(
                    5.0,
                    "[PerceptionSwitcher] Ignoring subsequent unblock signal (already switched)."
                )
                return
            self._switched = True
            # ★ 锁定当前 counts 快照
            self._locked_counts = dict(self._live_counts)

        rospy.logwarn(
            "[PerceptionSwitcher] Unblock received! Locked counts: %s",
            str(self._locked_counts)
        )

        rospy.sleep(0.2)
        self._freeze_pub.publish(Bool(data=True))
        rospy.logwarn("[PerceptionSwitcher] Sent freeze signal on %s.", self.freeze_counts_topic)

    # def _freeze_box_counter(self):
    #     freeze_topic = rospy.get_param("~freeze_counts_topic", "/percep/freeze_counts")
    #     pub = rospy.Publisher(freeze_topic, Bool, queue_size=1, latch=True)
    #     rospy.sleep(0.3)  # 等待 publisher 建立连接
    #     pub.publish(Bool(data=True))
    #     rospy.logwarn("[PerceptionSwitcher] Sent freeze signal to box_counter.")

    # ------------------------------------------------------------------
    # 当前看到的数字 → 用锁定 counts 判断是否是最小值
    # ------------------------------------------------------------------
    def _digit_cb(self, msg):
        # 切换完成前不处理
        if not self._switched or self._locked_counts is None:
            return

        digit = int(msg.data)
        if digit < 0 or digit > 9:
            return

        min_digits = self._get_min_digits()
        if not min_digits:
            return

        if not self.allow_tie:
            min_digits = [min(min_digits)]

        if digit not in min_digits:
            rospy.loginfo_throttle(
                1.0,
                "[PerceptionSwitcher] Seen digit=%d not in min_digits=%s (locked counts=%s)",
                digit, str(min_digits), str(self._locked_counts)
            )
            return

        now = rospy.Time.now().to_sec()
        if now - self._last_trigger_time[digit] < self.same_digit_cooldown:
            return

        self._last_trigger_time[digit] = now
        room_id = int(self.room_map.get(str(digit), digit))

        self._enter_room_pub.publish(Int32(data=room_id))
        self._enter_trigger_pub.publish(Bool(data=True))

        rospy.logwarn(
            "[PerceptionSwitcher] Enter room: digit=%d → room_id=%d | locked counts=%s | min_digits=%s",
            digit, room_id, str(self._locked_counts), str(min_digits)
        )

    def _get_min_digits(self):
        """从锁定的 counts 里找最小计数的数字列表。"""
        items = []
        for d in range(10):
            c = self._locked_counts.get(d, 0)
            if self.ignore_zero_count and c == 0:
                continue
            items.append((d, c))
        if not items:
            return []
        min_c = min(c for _, c in items)
        return [d for d, c in items if c == min_c]


if __name__ == "__main__":
    rospy.init_node("perception_switcher")
    node = PerceptionSwitcher()
    rospy.spin()