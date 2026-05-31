#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vlm_limo_image_sampler.py
Run on LIMO.

Purpose:
  - Subscribe to the LIMO compressed camera topic.
  - Re-publish sampled frames to /vlm/measure/image for laptop VLM inference.
  - Use LIMO ROS time in header.stamp so laptop can compute total latency if NTP is synchronized.

This node does NOT save CSV. CSV/images are saved on the laptop side.
"""

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool


class VlmLimoImageSampler:
    def __init__(self):
        rospy.init_node("vlm_limo_image_sampler")

        self.camera_topic = rospy.get_param("~camera_topic", "/camera/color/image_raw/compressed")
        self.output_image_topic = rospy.get_param("~output_image_topic", "/vlm/measure/image")
        self.sample_rate = float(rospy.get_param("~sample_rate", 1.0))
        self.trial_id = rospy.get_param("~trial_id", "trial01_lane_02mps")
        self.use_now_stamp = bool(rospy.get_param("~use_now_stamp", True))
        self.done_topic = rospy.get_param("~done_topic", "/vlm/measure/done")
        self.shutdown_on_done = bool(rospy.get_param("~shutdown_on_done", True))

        self.frame_idx = 0
        self.last_pub_time = 0.0

        self.pub = rospy.Publisher(self.output_image_topic, CompressedImage, queue_size=1)

        rospy.Subscriber(
            self.camera_topic,
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=2**24,
        )
        rospy.Subscriber(self.done_topic, Bool, self.done_callback, queue_size=1)

        rospy.loginfo("[LIMO sampler] started")
        rospy.loginfo("[LIMO sampler] camera_topic       : %s", self.camera_topic)
        rospy.loginfo("[LIMO sampler] output_image_topic : %s", self.output_image_topic)
        rospy.loginfo("[LIMO sampler] sample_rate        : %.3f Hz", self.sample_rate)
        rospy.loginfo("[LIMO sampler] trial_id           : %s", self.trial_id)
        rospy.loginfo("[LIMO sampler] use_now_stamp      : %s", self.use_now_stamp)
        rospy.loginfo("[LIMO sampler] done_topic         : %s", self.done_topic)
        rospy.loginfo("[LIMO sampler] shutdown_on_done   : %s", self.shutdown_on_done)


    def done_callback(self, msg):
        if msg.data and self.shutdown_on_done:
            rospy.loginfo("[LIMO sampler] received done signal. Shutting down sampler.")
            rospy.signal_shutdown("VLM measurement completed")

    def image_callback(self, msg):
        now = rospy.Time.now()
        now_sec = now.to_sec()

        if self.sample_rate > 0:
            min_interval = 1.0 / self.sample_rate
            if now_sec - self.last_pub_time < min_interval:
                return

        self.last_pub_time = now_sec
        self.frame_idx += 1

        out = CompressedImage()
        out.header = msg.header
        out.header.frame_id = f"{self.trial_id}_frame_{self.frame_idx:06d}"

        # For total latency calculation, stamp should represent the LIMO-side sampling/publish time.
        if self.use_now_stamp or out.header.stamp.to_sec() <= 0:
            out.header.stamp = now

        out.format = msg.format
        out.data = msg.data

        self.pub.publish(out)

        rospy.loginfo_throttle(
            2.0,
            "[LIMO sampler] published frame=%06d stamp=%.6f",
            self.frame_idx,
            out.header.stamp.to_sec(),
        )


def main():
    VlmLimoImageSampler()
    rospy.spin()


if __name__ == "__main__":
    main()
