#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <shared_mutex>
#include <string>

namespace ida_camera
{

// Per-frame timing for all pipeline stages, used by publishDiagnostics.
struct PipelineTiming
{
  double undistort_ms{0};
  double denoise_ms{0};
  double clahe_ms{0};
  double sharpen_ms{0};
  double gamma_ms{0};
  double letterbox_ms{0};

  double total() const
  {
    return undistort_ms + denoise_ms + clahe_ms + sharpen_ms + gamma_ms + letterbox_ms;
  }
};

/**
 * ROS2 node that processes raw camera frames from a Logitech C920 Pro (or
 * any standard rectilinear camera).
 *
 * Pipeline per frame (each stage is individually enable/disable-able):
 *   raw image
 *     → undistort   (cv::remap, precomputed Brown-Conrady maps)
 *     → denoise     (bilateral filter; preserves edges, removes sensor noise)
 *     → CLAHE       (contrast enhancement on LAB L-channel only)
 *     → sharpen     (unsharp mask; recovers C920 firmware softening)
 *     → gamma       (LUT-based gamma correction; free at runtime)
 *     → publish
 *
 * Calibration source priority:
 *   1. YAML file   (yaml_calibration_path param, loaded at startup)
 *   2. camera_info topic  (used as fallback if YAML is absent or invalid)
 *
 * All processing parameters are dynamically reconfigurable at runtime via
 *   ros2 param set /camera_processor <name> <value>
 * Thread-safe: onParamChange and onImage are protected by a shared_mutex.
 */
class CameraProcessorNode : public rclcpp::Node
{
public:
  explicit CameraProcessorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── ROS2 callbacks ────────────────────────────────────────────────────────
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & params);

  // ── Processing stages ─────────────────────────────────────────────────────
  bool    loadCalibrationFromYaml(const std::string & path);
  void    buildRemapMaps(double alpha);
  void    buildGammaLut();
  cv::Mat applyDenoise(const cv::Mat & bgr) const;
  cv::Mat applyCLAHE(const cv::Mat & bgr) const;
  cv::Mat applySharpen(const cv::Mat & bgr) const;
  cv::Mat applyGamma(const cv::Mat & bgr) const;

  // Letterbox-resize bgr to (ml_width_ × ml_height_) and convert to RGB.
  // Returns the resized RGB mat and fills out_offset/out_scale for the caller
  // to remap bounding-box coordinates back to the original frame if needed.
  cv::Mat applyLetterbox(const cv::Mat & bgr,
                         cv::Point2f & out_offset,
                         float       & out_scale) const;

  // ── Diagnostics ───────────────────────────────────────────────────────────
  void publishDiagnostics(const PipelineTiming & t);

  // ── ROS2 interfaces ───────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr         image_pub_;   // full-res BGR debug
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr         ml_pub_;      // 640×640 RGB for YOLO
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ── Calibration (written once, then read-only) ────────────────────────────
  cv::Mat  camera_matrix_;
  cv::Mat  dist_coeffs_;
  cv::Size image_size_;
  cv::Mat  map1_, map2_;
  bool     calibration_ready_{false};
  bool     maps_built_{false};

  // ── Processing engines (rebuilt on param change, guarded by proc_mutex_) ──
  cv::Ptr<cv::CLAHE> clahe_;
  cv::Mat            gamma_lut_;   // 1×256 CV_8UC1 lookup table

  // ── Parameters (guarded by proc_mutex_ for write, shared read in onImage) ─
  double undistort_alpha_;

  bool   use_denoise_;
  int    denoise_d_;
  double denoise_sigma_color_;
  double denoise_sigma_space_;

  bool   use_clahe_;
  double clahe_clip_limit_;
  int    clahe_tile_w_;
  int    clahe_tile_h_;

  bool   use_sharpen_;
  double sharpen_sigma_;
  double sharpen_strength_;

  bool   use_gamma_;
  double gamma_;

  // ── ML output parameters ───────────────────────────────────────────────────
  std::string ml_topic_;
  int         ml_width_;
  int         ml_height_;
  int         letterbox_fill_;   // gray pad value (114 is YOLO standard)

  // Protects all processing parameters and engines against concurrent
  // reads (onImage) and writes (onParamChange).
  mutable std::shared_mutex proc_mutex_;

  // ── Diagnostics accumulators (only touched from onImage / single thread) ──
  uint64_t   frame_count_{0};
  double     rolling_total_ms_{0.0};
  std::chrono::steady_clock::time_point last_diag_time_;
};

}  // namespace ida_camera
