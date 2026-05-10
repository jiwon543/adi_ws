#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vlm_laptop_infer_logger.py
Run on laptop.

Purpose:
  - Subscribe to sampled LIMO images from /vlm/measure/image.
  - Run Moondream VLM inference.
  - Save one CSV row per inference on the laptop.
  - Save input images, plus low/high latency image subsets for visual analysis.

Timing definitions:
  - total_latency_ms = laptop_vlm_end_time - limo_image_stamp
      Requires LIMO and laptop clocks to be synchronized with NTP/chrony.
  - vlm_inference_ms = time spent in model.encode_image + model.query on the laptop.
  - residual_ms = total_latency_ms - vlm_inference_ms
      This is NOT pure Wi-Fi. It includes ROS transport/queueing + laptop decode/resize overhead.
"""

import os
import csv
import cv2
import time
import json
import torch
import rospy
import numpy as np
from datetime import datetime
from PIL import Image as PILImage
from transformers import AutoModelForCausalLM

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class VlmLaptopInferLogger:
    def __init__(self):
        rospy.init_node("vlm_laptop_infer_logger")

        self.image_topic = rospy.get_param("~image_topic", "/vlm/measure/image")
        self.result_topic = rospy.get_param("~result_topic", "/vlm/measure/result_json")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        self.model_name = rospy.get_param("~model", "vikhyatk/moondream2")
        self.revision = rospy.get_param("~revision", "2025-01-09")
        self.prompt = rospy.get_param(
            "~prompt",
            "Describe the road scene ahead in one short sentence.",
        )
        self.max_tokens = int(rospy.get_param("~max_tokens", 20))
        self.input_size = int(rospy.get_param("~input_size", 384))
        self.sync_cuda = bool(rospy.get_param("~sync_cuda", False))
        self.warmup_count = int(rospy.get_param("~warmup", 3))
        self.max_inferences = int(rospy.get_param("~max_inferences", 100))
        self.shutdown_on_complete = bool(rospy.get_param("~shutdown_on_complete", True))
        self.done_topic = rospy.get_param("~done_topic", "/vlm/measure/done")

        self.output_dir = rospy.get_param(
            "~output_dir",
            os.path.expanduser("~/vlm_latency_results"),
        )
        self.trial_id = rospy.get_param(
            "~trial_id",
            datetime.now().strftime("trial_%Y%m%d_%H%M%S"),
        )
        self.save_images = bool(rospy.get_param("~save_images", True))
        self.save_all_images = bool(rospy.get_param("~save_all_images", True))
        self.high_latency_threshold_ms = float(rospy.get_param("~high_latency_threshold_ms", 620.0))
        self.low_latency_threshold_ms = float(rospy.get_param("~low_latency_threshold_ms", 540.0))

        self.result_dir = os.path.join(self.output_dir, self.trial_id)
        self.image_dir = os.path.join(self.result_dir, "images")
        self.high_dir = os.path.join(self.result_dir, "high_latency_images")
        self.low_dir = os.path.join(self.result_dir, "low_latency_images")
        os.makedirs(self.result_dir, exist_ok=True)
        os.makedirs(self.image_dir, exist_ok=True)
        os.makedirs(self.high_dir, exist_ok=True)
        os.makedirs(self.low_dir, exist_ok=True)

        self.csv_path = os.path.join(self.result_dir, f"vlm_latency_{self.trial_id}.csv")
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8-sig")
        self.csv_writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                "trial_id",
                "frame_id",
                "seq",
                "limo_image_stamp",
                "laptop_receive_time",
                "vlm_start_time",
                "vlm_end_time",
                "total_latency_ms",
                "vlm_inference_ms",
                "residual_ms",
                "decode_resize_ms",
                "image_filename",
                "latency_group",
                "answer",
                "mission_estimate",
                "prompt",
                "max_tokens",
                "input_size",
                "model",
                "revision",
                "sync_cuda",
                "robot_speed_odom_mps",
                "cmd_linear_x",
                "cmd_angular_z",
            ],
        )
        self.csv_writer.writeheader()

        self.latest_odom_speed = 0.0
        self.latest_cmd_linear_x = 0.0
        self.latest_cmd_angular_z = 0.0
        self.busy = False
        self.seq = 0
        self.completed_count = 0

        self.result_pub = rospy.Publisher(self.result_topic, String, queue_size=10)
        self.done_pub = rospy.Publisher(self.done_topic, Bool, queue_size=1, latch=True)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_callback, queue_size=10)

        rospy.loginfo("[Laptop VLM] loading model...")
        self.model = self.load_model()
        self.warmup()

        rospy.Subscriber(
            self.image_topic,
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=2**24,
        )
        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("[Laptop VLM] started")
        rospy.loginfo("[Laptop VLM] image_topic : %s", self.image_topic)
        rospy.loginfo("[Laptop VLM] result_topic: %s", self.result_topic)
        rospy.loginfo("[Laptop VLM] csv_path    : %s", self.csv_path)
        rospy.loginfo("[Laptop VLM] output_dir  : %s", self.result_dir)
        rospy.loginfo("[Laptop VLM] max_inferences: %d", self.max_inferences)
        rospy.loginfo("[Laptop VLM] done_topic  : %s", self.done_topic)

    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.latest_odom_speed = (vx * vx + vy * vy) ** 0.5

    def cmd_callback(self, msg):
        self.latest_cmd_linear_x = msg.linear.x
        self.latest_cmd_angular_z = msg.angular.z

    def sync_if_needed(self):
        if self.sync_cuda and torch.cuda.is_available():
            torch.cuda.synchronize()

    def load_model(self):
        kwargs = {"trust_remote_code": True, "revision": self.revision}
        rospy.loginfo("[Laptop VLM] model    : %s", self.model_name)
        rospy.loginfo("[Laptop VLM] revision : %s", self.revision)
        rospy.loginfo("[Laptop VLM] CUDA     : %s", torch.cuda.is_available())

        if torch.cuda.is_available():
            rospy.loginfo("[Laptop VLM] GPU      : %s", torch.cuda.get_device_name(0))
            kwargs["device_map"] = {"": "cuda"}
            if "4bit" not in self.model_name.lower():
                kwargs["torch_dtype"] = torch.float16
                rospy.loginfo("[Laptop VLM] precision: FP16")
        else:
            rospy.logwarn("[Laptop VLM] CUDA unavailable. Running on CPU.")

        model = AutoModelForCausalLM.from_pretrained(self.model_name, **kwargs)
        model.eval()
        try:
            p = next(model.parameters())
            rospy.loginfo("[Laptop VLM] dtype/device: %s / %s", str(p.dtype), str(p.device))
        except Exception:
            pass
        return model

    def warmup(self):
        if self.warmup_count <= 0:
            return
        dummy = PILImage.new("RGB", (self.input_size, self.input_size), (128, 128, 128))
        rospy.loginfo("[Laptop VLM] warmup: %d", self.warmup_count)
        for i in range(self.warmup_count):
            self.sync_if_needed()
            with torch.inference_mode():
                encoded = self.model.encode_image(dummy)
                _ = self.model.query(encoded, self.prompt, settings={"max_tokens": self.max_tokens})
            self.sync_if_needed()
            rospy.loginfo("[Laptop VLM] warmup %d/%d done", i + 1, self.warmup_count)

    def decode_image(self, msg):
        t0 = time.time()
        arr = np.frombuffer(msg.data, np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            raise RuntimeError("cv2.imdecode failed")
        original_bgr = bgr.copy()
        if self.input_size > 0:
            bgr_model = cv2.resize(bgr, (self.input_size, self.input_size))
        else:
            bgr_model = bgr
        rgb = cv2.cvtColor(bgr_model, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(rgb)
        t1 = time.time()
        return original_bgr, pil_img, (t1 - t0) * 1000.0

    def classify_latency_group(self, infer_ms):
        if infer_ms >= self.high_latency_threshold_ms:
            return "high"
        if infer_ms <= self.low_latency_threshold_ms:
            return "low"
        return "middle"

    def estimate_mission(self, answer):
        text = answer.lower()
        if "cone" in text:
            return "cone_avoidance"
        if "crosswalk" in text or "yellow" in text or "striped" in text:
            return "crosswalk_or_marker"
        if "vehicle" in text or "car" in text or "truck" in text or "obstacle" in text:
            return "obstacle"
        if "road" in text or "track" in text or "lane" in text:
            return "normal_drive"
        return "unknown"

    def image_callback(self, msg):
        if self.busy:
            rospy.logwarn_throttle(2.0, "[Laptop VLM] busy; dropping frame")
            return

        self.busy = True
        laptop_receive_time = time.time()
        try:
            self.seq += 1
            frame_id = msg.header.frame_id or f"{self.trial_id}_frame_{self.seq:06d}"
            limo_image_stamp = msg.header.stamp.to_sec()

            original_bgr, pil_img, decode_resize_ms = self.decode_image(msg)

            self.sync_if_needed()
            vlm_start_time = time.time()
            with torch.inference_mode():
                encoded = self.model.encode_image(pil_img)
                result = self.model.query(encoded, self.prompt, settings={"max_tokens": self.max_tokens})
            self.sync_if_needed()
            vlm_end_time = time.time()

            infer_ms = (vlm_end_time - vlm_start_time) * 1000.0
            total_latency_ms = (vlm_end_time - limo_image_stamp) * 1000.0 if limo_image_stamp > 0 else 0.0
            residual_ms = total_latency_ms - infer_ms
            answer = result["answer"].strip()
            mission = self.estimate_mission(answer)
            latency_group = self.classify_latency_group(infer_ms)

            image_filename = ""
            if self.save_images:
                safe_frame = frame_id.replace("/", "_")
                base = f"{safe_frame}_{latency_group}_{infer_ms:.1f}ms.jpg"
                if self.save_all_images:
                    image_path = os.path.join(self.image_dir, base)
                    cv2.imwrite(image_path, original_bgr)
                    image_filename = image_path
                if latency_group == "high":
                    cv2.imwrite(os.path.join(self.high_dir, base), original_bgr)
                elif latency_group == "low":
                    cv2.imwrite(os.path.join(self.low_dir, base), original_bgr)

            row = {
                "trial_id": self.trial_id,
                "frame_id": frame_id,
                "seq": self.seq,
                "limo_image_stamp": limo_image_stamp,
                "laptop_receive_time": laptop_receive_time,
                "vlm_start_time": vlm_start_time,
                "vlm_end_time": vlm_end_time,
                "total_latency_ms": total_latency_ms,
                "vlm_inference_ms": infer_ms,
                "residual_ms": residual_ms,
                "decode_resize_ms": decode_resize_ms,
                "image_filename": image_filename,
                "latency_group": latency_group,
                "answer": answer,
                "mission_estimate": mission,
                "prompt": self.prompt,
                "max_tokens": self.max_tokens,
                "input_size": self.input_size,
                "model": self.model_name,
                "revision": self.revision,
                "sync_cuda": self.sync_cuda,
                "robot_speed_odom_mps": self.latest_odom_speed,
                "cmd_linear_x": self.latest_cmd_linear_x,
                "cmd_angular_z": self.latest_cmd_angular_z,
            }
            self.csv_writer.writerow(row)
            self.csv_file.flush()

            out = String()
            out.data = json.dumps({
                "trial_id": self.trial_id,
                "frame_id": frame_id,
                "limo_image_stamp": limo_image_stamp,
                "laptop_receive_time": laptop_receive_time,
                "vlm_start_time": vlm_start_time,
                "vlm_end_time": vlm_end_time,
                "total_latency_ms": total_latency_ms,
                "vlm_inference_ms": infer_ms,
                "residual_ms": residual_ms,
                "answer": answer,
                "mission_estimate": mission,
                "latency_group": latency_group,
            })
            self.result_pub.publish(out)

            self.completed_count += 1

            rospy.loginfo(
                "[Laptop VLM] %s | count=%d/%d | total=%.1f ms | vlm=%.1f ms | residual=%.1f ms | group=%s | speed=%.3f | answer=%s",
                frame_id,
                self.completed_count,
                self.max_inferences,
                total_latency_ms,
                infer_ms,
                residual_ms,
                latency_group,
                self.latest_odom_speed,
                answer,
            )

            if self.shutdown_on_complete and self.max_inferences > 0 and self.completed_count >= self.max_inferences:
                rospy.loginfo(
                    "[Laptop VLM] completed %d inferences. Publishing done signal and shutting down.",
                    self.completed_count,
                )
                self.done_pub.publish(Bool(data=True))
                rospy.sleep(0.3)
                rospy.signal_shutdown("Reached max_inferences")
        except Exception as e:
            rospy.logerr("[Laptop VLM] failed: %s", str(e))
        finally:
            self.busy = False

    def on_shutdown(self):
        try:
            self.csv_file.flush()
            self.csv_file.close()
            rospy.loginfo("[Laptop VLM] CSV saved: %s", self.csv_path)
        except Exception:
            pass


def main():
    VlmLaptopInferLogger()
    rospy.spin()


if __name__ == "__main__":
    main()
