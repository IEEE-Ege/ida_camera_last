#include "ida_camera/camera_processor_node.hpp"

#include <yaml-cpp/yaml.h>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <chrono>
#include <cmath>

namespace ida_camera
{

using namespace std::chrono;

// ── File-local helpers ────────────────────────────────────────────────────────

static double toMs(steady_clock::duration d)
{
  return duration_cast<nanoseconds>(d).count() / 1e6;
}

static rcl_interfaces::msg::ParameterDescriptor desc(const std::string & text)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = text;
  return d;
}

static diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & val)
{
  diagnostic_msgs::msg::KeyValue p;
  p.key = key; p.value = val;
  return p;
}

// Format a double to N significant characters without scientific notation.
static std::string fmt(double v, int chars = 5)
{
  auto s = std::to_string(v);
  return s.substr(0, std::min(static_cast<int>(s.size()), chars));
}


// ── Constructor ───────────────────────────────────────────────────────────────

CameraProcessorNode::CameraProcessorNode(const rclcpp::NodeOptions & options)
: Node("camera_processor", options),
  last_diag_time_(steady_clock::now())
{
  // ── Topic names ──────────────────────────────────────────────────────────
  declare_parameter("image_topic",       "/camera/image_raw",       desc("Input raw image topic"));
  declare_parameter("camera_info_topic", "/camera/camera_info",     desc("CameraInfo topic (fallback calibration)"));
  declare_parameter("output_topic",      "/camera/image_processed", desc("Processed image output topic"));

  // ── Calibration ──────────────────────────────────────────────────────────
  declare_parameter("yaml_calibration_path", "",
    desc("Path to a ros camera_calibration YAML. Leave empty to use camera_info."));
  declare_parameter("undistort_alpha", 0.0,
    desc("0=crop to valid pixels (no black borders), 1=keep all pixels."));

  // ── Bilateral denoising ──────────────────────────────────────────────────
  // Run BEFORE CLAHE so noise is not amplified by contrast enhancement.
  // bilateralFilter is edge-preserving: it blurs flat regions but keeps sharp
  // transitions (unlike GaussianBlur which blurs everything equally).
  // Trade-off: it is expensive — ~8-15 ms at 1080p. Disable for real-time use
  // unless you have a GPU pipeline downstream that benefits from cleaner input.
  declare_parameter("use_denoise",          false, desc("Enable bilateral denoising (runs before CLAHE)"));
  declare_parameter("denoise_d",            9,     desc("Bilateral filter diameter. Larger=stronger, slower. 5=fast, 9=good quality."));
  declare_parameter("denoise_sigma_color",  75.0,  desc("Color range sigma. Higher=mix more dissimilar colors (stronger, may blur edges)."));
  declare_parameter("denoise_sigma_space",  75.0,  desc("Spatial sigma. Higher=pixels farther apart influence each other."));

  // ── CLAHE ────────────────────────────────────────────────────────────────
  declare_parameter("use_clahe",        true, desc("Enable CLAHE contrast enhancement"));
  declare_parameter("clahe_clip_limit", 1.5,  desc("Max contrast amplification per tile. Higher=more punch."));
  declare_parameter("clahe_tile_width",  8,   desc("CLAHE grid cell width in pixels. Smaller=more local."));
  declare_parameter("clahe_tile_height", 8,   desc("CLAHE grid cell height in pixels."));

  // ── Sharpening (unsharp mask) ─────────────────────────────────────────────
  // The C920 applies mild in-camera softening at 30 fps.
  // Run AFTER CLAHE: sharpen the already-enhanced image, not the noise.
  declare_parameter("use_sharpen",      false, desc("Enable unsharp mask sharpening"));
  declare_parameter("sharpen_sigma",    1.0,   desc("Blur sigma for unsharp mask. 0.8–1.5 suits the C920 at 1080p."));
  declare_parameter("sharpen_strength", 0.8,   desc("Blend weight. 0=none, 0.8=noticeable, >1=aggressive."));

  // ── Gamma correction ─────────────────────────────────────────────────────
  // Applied last, as a global brightness curve via a precomputed LUT.
  // formula: out[i] = 255 * (i/255)^(1/gamma)
  //   gamma < 1.0 → brighter shadows (typical use: lift dark areas)
  //   gamma > 1.0 → darker, more contrast
  //   gamma = 1.0 → identity (no change)
  // The C920 ships with gamma ≈ 2.2 (sRGB). If your downstream algorithm
  // expects linear light, set gamma to 2.2 to linearise.
  declare_parameter("use_gamma", false, desc("Enable gamma correction"));
  declare_parameter("gamma",     1.0,   desc("Gamma exponent. <1=brighter, >1=darker, 1.0=no-op."));

  // ── ML output (YOLOv11 / Jetson Nano) ────────────────────────────────────
  // Publishes a second, separate image: letterboxed to ml_width × ml_height,
  // converted to RGB8. The YOLO ROS2 node subscribes to this topic.
  // Letterboxing preserves the original aspect ratio by padding the shorter
  // axis with a solid fill colour (114 = YOLO standard grey).
  declare_parameter("ml_topic",       "/camera/image_ml",
    desc("Output topic for the YOLO-ready letterboxed RGB image."));
  declare_parameter("ml_width",       640,
    desc("Target width for the ML output image."));
  declare_parameter("ml_height",      640,
    desc("Target height for the ML output image."));
  declare_parameter("letterbox_fill", 114,
    desc("Pixel value used to fill letterbox padding (0-255). YOLO standard is 114."));

  // ── Read all initial values ───────────────────────────────────────────────
  const auto image_topic       = get_parameter("image_topic").as_string();
  const auto camera_info_topic = get_parameter("camera_info_topic").as_string();
  const auto output_topic      = get_parameter("output_topic").as_string();
  const auto yaml_path         = get_parameter("yaml_calibration_path").as_string();

  undistort_alpha_    = get_parameter("undistort_alpha").as_double();
  use_denoise_        = get_parameter("use_denoise").as_bool();
  denoise_d_          = get_parameter("denoise_d").as_int();
  denoise_sigma_color_= get_parameter("denoise_sigma_color").as_double();
  denoise_sigma_space_= get_parameter("denoise_sigma_space").as_double();
  use_clahe_          = get_parameter("use_clahe").as_bool();
  clahe_clip_limit_   = get_parameter("clahe_clip_limit").as_double();
  clahe_tile_w_       = get_parameter("clahe_tile_width").as_int();
  clahe_tile_h_       = get_parameter("clahe_tile_height").as_int();
  use_sharpen_        = get_parameter("use_sharpen").as_bool();
  sharpen_sigma_      = get_parameter("sharpen_sigma").as_double();
  sharpen_strength_   = get_parameter("sharpen_strength").as_double();
  use_gamma_          = get_parameter("use_gamma").as_bool();
  gamma_              = get_parameter("gamma").as_double();
  ml_topic_           = get_parameter("ml_topic").as_string();
  ml_width_           = get_parameter("ml_width").as_int();
  ml_height_          = get_parameter("ml_height").as_int();
  letterbox_fill_     = get_parameter("letterbox_fill").as_int();

  // ── Calibration from YAML ────────────────────────────────────────────────
  if (!yaml_path.empty()) {
    if (loadCalibrationFromYaml(yaml_path)) {
      RCLCPP_INFO(get_logger(), "Calibration loaded from YAML: %s", yaml_path.c_str());
    } else {
      RCLCPP_WARN(get_logger(),
        "YAML calibration '%s' could not be read — waiting for camera_info topic.",
        yaml_path.c_str());
    }
  }

  // ── Build processing engines ──────────────────────────────────────────────
  if (use_clahe_) {
    clahe_ = cv::createCLAHE(clahe_clip_limit_, cv::Size(clahe_tile_w_, clahe_tile_h_));
  }
  buildGammaLut();

  // ── Dynamic reconfiguration ───────────────────────────────────────────────
  param_cb_handle_ = add_on_set_parameters_callback(
    std::bind(&CameraProcessorNode::onParamChange, this, std::placeholders::_1));

  // ── Subscriptions ─────────────────────────────────────────────────────────
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic, rclcpp::SensorDataQoS(),
    std::bind(&CameraProcessorNode::onImage, this, std::placeholders::_1));

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic, rclcpp::QoS(1).reliable(),
    std::bind(&CameraProcessorNode::onCameraInfo, this, std::placeholders::_1));

  // ── Publishers ────────────────────────────────────────────────────────────
  image_pub_ = create_publisher<sensor_msgs::msg::Image>(
    output_topic, rclcpp::SensorDataQoS());

  ml_pub_ = create_publisher<sensor_msgs::msg::Image>(
    ml_topic_, rclcpp::SensorDataQoS());

  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(10).reliable());

  RCLCPP_INFO(get_logger(),
    "Camera processor ready.  in='%s'  out='%s'  ml='%s' (%dx%d RGB)\n"
    "  denoise=%s  clahe=%s  sharpen=%s  gamma=%s",
    image_topic.c_str(), output_topic.c_str(),
    ml_topic_.c_str(), ml_width_, ml_height_,
    use_denoise_ ? "ON" : "OFF",
    use_clahe_   ? "ON" : "OFF",
    use_sharpen_ ? "ON" : "OFF",
    use_gamma_   ? "ON" : "OFF");
}


// ── onCameraInfo ──────────────────────────────────────────────────────────────

void CameraProcessorNode::onCameraInfo(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  if (calibration_ready_) return;

  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      camera_matrix_.at<double>(r, c) = msg->k[r * 3 + c];
    }
  }

  dist_coeffs_ = cv::Mat(static_cast<int>(msg->d.size()), 1, CV_64F);
  for (size_t i = 0; i < msg->d.size(); ++i) {
    dist_coeffs_.at<double>(i) = msg->d[i];
  }

  image_size_        = cv::Size(msg->width, msg->height);
  calibration_ready_ = true;
  maps_built_        = false;

  RCLCPP_INFO(get_logger(),
    "Calibration received from camera_info (%ux%u).", msg->width, msg->height);

  camera_info_sub_.reset();
}


// ── loadCalibrationFromYaml ───────────────────────────────────────────────────

bool CameraProcessorNode::loadCalibrationFromYaml(const std::string & path)
{
  try {
    YAML::Node cfg = YAML::LoadFile(path);

    const auto k = cfg["camera_matrix"]["data"].as<std::vector<double>>();
    if (k.size() != 9) {
      RCLCPP_ERROR(get_logger(),
        "camera_matrix must have 9 elements, found %zu.", k.size());
      return false;
    }
    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
      camera_matrix_.at<double>(i / 3, i % 3) = k[i];
    }

    const auto d = cfg["distortion_coefficients"]["data"].as<std::vector<double>>();
    dist_coeffs_ = cv::Mat(static_cast<int>(d.size()), 1, CV_64F);
    for (size_t i = 0; i < d.size(); ++i) {
      dist_coeffs_.at<double>(i) = d[i];
    }

    image_size_        = cv::Size(cfg["image_width"].as<int>(), cfg["image_height"].as<int>());
    calibration_ready_ = true;
    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "YAML load error: %s", e.what());
    return false;
  }
}


// ── buildRemapMaps ────────────────────────────────────────────────────────────

void CameraProcessorNode::buildRemapMaps(double alpha)
{
  const cv::Mat optimal_K = cv::getOptimalNewCameraMatrix(
    camera_matrix_, dist_coeffs_, image_size_, alpha, image_size_);

  cv::initUndistortRectifyMap(
    camera_matrix_, dist_coeffs_,
    cv::Mat(),   // no rectification rotation (single camera, not stereo)
    optimal_K, image_size_, CV_32FC1,
    map1_, map2_);

  maps_built_ = true;
  RCLCPP_INFO(get_logger(),
    "Remap maps built (alpha=%.2f, %dx%d).", alpha, image_size_.width, image_size_.height);
}


// ── buildGammaLut ─────────────────────────────────────────────────────────────
//
// Precompute a 256-entry lookup table so applyGamma() is just one cv::LUT call
// per frame — effectively free compared to the other stages.
void CameraProcessorNode::buildGammaLut()
{
  gamma_lut_.create(1, 256, CV_8UC1);
  for (int i = 0; i < 256; ++i) {
    // power curve: out = 255 * (in/255)^(1/gamma)
    // gamma < 1 → convex curve → brightens shadows
    // gamma > 1 → concave curve → darkens, more contrast
    const double normalised = std::pow(i / 255.0, 1.0 / gamma_);
    gamma_lut_.at<uint8_t>(i) = cv::saturate_cast<uint8_t>(normalised * 255.0);
  }
}


// ── applyDenoise ──────────────────────────────────────────────────────────────
//
// Bilateral filter: for each output pixel, takes a weighted average of nearby
// pixels where the weight drops off with both spatial distance AND color
// distance. Flat regions get blurred; edges are preserved because pixels across
// an edge have very different colors and therefore low weight.
cv::Mat CameraProcessorNode::applyDenoise(const cv::Mat & bgr) const
{
  cv::Mat result;
  // Note: bilateralFilter cannot operate in-place (src != dst required).
  cv::bilateralFilter(bgr, result, denoise_d_, denoise_sigma_color_, denoise_sigma_space_);
  return result;
}


// ── applyCLAHE ────────────────────────────────────────────────────────────────
//
// We work in LAB colorspace so CLAHE only modifies Lightness (L channel).
// Running it on BGR directly would shift hue and saturation.
cv::Mat CameraProcessorNode::applyCLAHE(const cv::Mat & bgr) const
{
  cv::Mat lab;
  cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);

  std::vector<cv::Mat> ch(3);
  cv::split(lab, ch);
  clahe_->apply(ch[0], ch[0]);
  cv::merge(ch, lab);

  cv::Mat result;
  cv::cvtColor(lab, result, cv::COLOR_Lab2BGR);
  return result;
}


// ── applySharpen ──────────────────────────────────────────────────────────────
//
// Unsharp mask: high_frequency_detail = original − gaussian_blur(original)
//               output = original + strength × high_frequency_detail
//
// Equivalent to: cv::addWeighted(original, 1+k, blur, -k, 0, output)
cv::Mat CameraProcessorNode::applySharpen(const cv::Mat & bgr) const
{
  cv::Mat blurred;
  cv::GaussianBlur(bgr, blurred, cv::Size(0, 0), sharpen_sigma_);

  cv::Mat result;
  cv::addWeighted(bgr, 1.0 + sharpen_strength_, blurred, -sharpen_strength_, 0, result);
  return result;
}


// ── applyGamma ────────────────────────────────────────────────────────────────

cv::Mat CameraProcessorNode::applyGamma(const cv::Mat & bgr) const
{
  cv::Mat result;
  cv::LUT(bgr, gamma_lut_, result);
  return result;
}


// ── applyLetterbox ────────────────────────────────────────────────────────────
//
// Scales the image to fit inside (ml_width_ × ml_height_) while preserving
// the original aspect ratio, then pads the remaining area symmetrically.
//
// Why this matters for YOLO:
//   A naive cv::resize to 640×640 squashes a 1920×1080 frame horizontally
//   (ratio 1.78 → 1.0). YOLO was trained on letter-boxed inputs, so
//   deformed aspect ratios hurt bounding-box regression and classification.
//
// out_scale  — multiply a coordinate in the resized image by this to get the
//              corresponding pixel in the original undistorted frame.
// out_offset — (x,y) top-left of the active image within the 640×640 canvas.
//              Subtract this before applying out_scale to undo the padding.
//
// To convert a YOLO detection box back to original frame coordinates:
//   orig_x = (box_x - out_offset.x) / out_scale
//   orig_y = (box_y - out_offset.y) / out_scale
cv::Mat CameraProcessorNode::applyLetterbox(const cv::Mat & bgr,
                                             cv::Point2f   & out_offset,
                                             float         & out_scale) const
{
  const float scale_x = static_cast<float>(ml_width_)  / bgr.cols;
  const float scale_y = static_cast<float>(ml_height_) / bgr.rows;
  out_scale = std::min(scale_x, scale_y);  // uniform scale, limited by tighter axis

  const int scaled_w = static_cast<int>(std::round(bgr.cols * out_scale));
  const int scaled_h = static_cast<int>(std::round(bgr.rows * out_scale));

  // Padding is split evenly on both sides (YOLO-standard centering)
  const float pad_x = (ml_width_  - scaled_w) / 2.0f;
  const float pad_y = (ml_height_ - scaled_h) / 2.0f;
  out_offset = cv::Point2f(pad_x, pad_y);

  // 1. Resize the BGR image to the scaled dimensions
  cv::Mat scaled;
  cv::resize(bgr, scaled, cv::Size(scaled_w, scaled_h), 0, 0, cv::INTER_LINEAR);

  // 2. Create the canvas filled with the letterbox colour
  cv::Mat canvas(ml_height_, ml_width_, CV_8UC3,
    cv::Scalar(letterbox_fill_, letterbox_fill_, letterbox_fill_));

  // 3. Copy the scaled image into the centred ROI
  scaled.copyTo(canvas(cv::Rect(
    static_cast<int>(pad_x), static_cast<int>(pad_y), scaled_w, scaled_h)));

  // 4. Convert BGR → RGB (YOLO/PyTorch convention)
  cv::Mat rgb;
  cv::cvtColor(canvas, rgb, cv::COLOR_BGR2RGB);
  return rgb;
}


// ── onParamChange ─────────────────────────────────────────────────────────────
//
// Called synchronously before the new value is committed. We take an exclusive
// lock so onImage (which holds a shared lock) cannot run concurrently while we
// rebuild CLAHE or the gamma LUT.
rcl_interfaces::msg::SetParametersResult
CameraProcessorNode::onParamChange(const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool rebuild_clahe = false;
  bool rebuild_gamma = false;
  bool rebuild_maps  = false;

  // Collect changes first, then take the lock to apply them atomically.
  // This avoids holding the lock during the O(n) parameter loop.
  struct Snapshot {
    bool   use_denoise, use_clahe, use_sharpen, use_gamma;
    int    denoise_d, tile_w, tile_h;
    double denoise_sc, denoise_ss;
    double clip, sigma, strength, gamma_v, alpha;
  } s{use_denoise_, use_clahe_, use_sharpen_, use_gamma_,
      denoise_d_, clahe_tile_w_, clahe_tile_h_,
      denoise_sigma_color_, denoise_sigma_space_,
      clahe_clip_limit_, sharpen_sigma_, sharpen_strength_, gamma_, undistort_alpha_};

  for (const auto & p : params) {
    const auto & n = p.get_name();
    if      (n == "use_denoise")          s.use_denoise = p.as_bool();
    else if (n == "denoise_d")          { s.denoise_d   = p.as_int(); }
    else if (n == "denoise_sigma_color"){ s.denoise_sc  = p.as_double(); }
    else if (n == "denoise_sigma_space"){ s.denoise_ss  = p.as_double(); }
    else if (n == "use_clahe")            s.use_clahe   = p.as_bool();
    else if (n == "clahe_clip_limit")   { s.clip        = p.as_double(); rebuild_clahe = true; }
    else if (n == "clahe_tile_width")   { s.tile_w      = p.as_int();    rebuild_clahe = true; }
    else if (n == "clahe_tile_height")  { s.tile_h      = p.as_int();    rebuild_clahe = true; }
    else if (n == "use_sharpen")          s.use_sharpen = p.as_bool();
    else if (n == "sharpen_sigma")        s.sigma       = p.as_double();
    else if (n == "sharpen_strength")     s.strength    = p.as_double();
    else if (n == "use_gamma")            s.use_gamma   = p.as_bool();
    else if (n == "gamma")              { s.gamma_v     = p.as_double(); rebuild_gamma = true; }
    else if (n == "undistort_alpha")    { s.alpha       = p.as_double(); rebuild_maps  = true; }
  }

  // Now apply under exclusive lock
  {
    std::unique_lock lock(proc_mutex_);

    use_denoise_         = s.use_denoise;
    denoise_d_           = s.denoise_d;
    denoise_sigma_color_ = s.denoise_sc;
    denoise_sigma_space_ = s.denoise_ss;
    use_clahe_           = s.use_clahe;
    clahe_clip_limit_    = s.clip;
    clahe_tile_w_        = s.tile_w;
    clahe_tile_h_        = s.tile_h;
    use_sharpen_         = s.use_sharpen;
    sharpen_sigma_       = s.sigma;
    sharpen_strength_    = s.strength;
    use_gamma_           = s.use_gamma;
    gamma_               = s.gamma_v;
    undistort_alpha_     = s.alpha;

    // Rebuild if clip/tile changed, OR if use_clahe was just enabled and the
    // engine was never constructed (started with use_clahe: false).
    if (rebuild_clahe || (s.use_clahe && !clahe_)) {
      clahe_ = cv::createCLAHE(clahe_clip_limit_, cv::Size(clahe_tile_w_, clahe_tile_h_));
      RCLCPP_INFO(get_logger(), "CLAHE rebuilt: clip=%.1f tile=%dx%d",
        clahe_clip_limit_, clahe_tile_w_, clahe_tile_h_);
    }
    if (rebuild_gamma) {
      buildGammaLut();  // reads gamma_, so must be called inside lock
      RCLCPP_INFO(get_logger(), "Gamma LUT rebuilt: gamma=%.2f", gamma_);
    }
    if (rebuild_maps && maps_built_) {
      maps_built_ = false;  // onImage will rebuild with the new alpha on next frame
      RCLCPP_INFO(get_logger(), "Remap maps flagged for rebuild (alpha=%.2f).", s.alpha);
    }
  }

  return result;
}


// ── publishDiagnostics ────────────────────────────────────────────────────────

void CameraProcessorNode::publishDiagnostics(const PipelineTiming & t)
{
  ++frame_count_;
  rolling_total_ms_ += t.total();

  const auto now = steady_clock::now();
  if (toMs(now - last_diag_time_) < 1000.0) return;

  const double avg_ms = rolling_total_ms_ / static_cast<double>(frame_count_);

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name        = "camera_processor";
  status.hardware_id = "logitech_c920";
  status.level       = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message     = "Running";

  const double elapsed_s = toMs(now - last_diag_time_) / 1000.0;
  status.values = {
    kv("frames_per_sec",   fmt(frame_count_ / elapsed_s)),
    kv("avg_pipeline_ms",  fmt(avg_ms)),
    kv("undistort_ms",     fmt(t.undistort_ms)),
    kv("denoise_ms",       fmt(t.denoise_ms)),
    kv("clahe_ms",         fmt(t.clahe_ms)),
    kv("sharpen_ms",       fmt(t.sharpen_ms)),
    kv("gamma_ms",         fmt(t.gamma_ms)),
    kv("letterbox_ms",     fmt(t.letterbox_ms)),
    kv("ml_resolution",    std::to_string(ml_width_) + "x" + std::to_string(ml_height_)),
    kv("denoise_enabled",  use_denoise_ ? "true" : "false"),
    kv("clahe_enabled",    use_clahe_   ? "true" : "false"),
    kv("sharpen_enabled",  use_sharpen_ ? "true" : "false"),
    kv("gamma_enabled",    use_gamma_   ? "true" : "false"),
  };

  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = get_clock()->now();
  arr.status.push_back(status);
  diag_pub_->publish(arr);

  last_diag_time_   = now;
  frame_count_      = 0;
  rolling_total_ms_ = 0.0;
}


// ── onImage ───────────────────────────────────────────────────────────────────

void CameraProcessorNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (!calibration_ready_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "No calibration yet — publishing raw frame unchanged.");
    image_pub_->publish(*msg);
    return;
  }

  // ── Decode to BGR8 ───────────────────────────────────────────────────────
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  const cv::Mat & frame = cv_ptr->image;

  // Remap maps are built once. Rebuild if resolution changed (e.g. driver restart).
  // Snapshot undistort_alpha_ under a brief shared lock so buildRemapMaps()
  // always sees a consistent value, even if onParamChange runs concurrently.
  if (!maps_built_ || frame.size() != image_size_) {
    if (maps_built_) {
      RCLCPP_WARN(get_logger(),
        "Frame size changed to %dx%d — rebuilding remap maps.", frame.cols, frame.rows);
    }
    image_size_ = frame.size();
    double alpha_snapshot;
    {
      std::shared_lock lock(proc_mutex_);
      alpha_snapshot = undistort_alpha_;
    }
    buildRemapMaps(alpha_snapshot);
  }

  PipelineTiming timing;

  // ── Stage 1: Undistort ───────────────────────────────────────────────────
  auto t = steady_clock::now();
  cv::Mat current;
  cv::remap(frame, current, map1_, map2_, cv::INTER_LINEAR);
  timing.undistort_ms = toMs(steady_clock::now() - t);

  // Acquire shared lock for all processing stages — allows concurrent reads
  // but blocks if onParamChange is rebuilding an engine.
  std::shared_lock lock(proc_mutex_);

  // ── Stage 2: Denoise ─────────────────────────────────────────────────────
  t = steady_clock::now();
  if (use_denoise_) {
    current = applyDenoise(current);
  }
  timing.denoise_ms = toMs(steady_clock::now() - t);

  // ── Stage 3: CLAHE ───────────────────────────────────────────────────────
  t = steady_clock::now();
  if (use_clahe_ && clahe_) {
    current = applyCLAHE(current);
  }
  timing.clahe_ms = toMs(steady_clock::now() - t);

  // ── Stage 4: Sharpen ─────────────────────────────────────────────────────
  t = steady_clock::now();
  if (use_sharpen_) {
    current = applySharpen(current);
  }
  timing.sharpen_ms = toMs(steady_clock::now() - t);

  // ── Stage 5: Gamma ───────────────────────────────────────────────────────
  t = steady_clock::now();
  if (use_gamma_) {
    current = applyGamma(current);
  }
  timing.gamma_ms = toMs(steady_clock::now() - t);

  // ── Stage 6: Letterbox + BGR→RGB for YOLO ───────────────────────────────
  auto t_lb = steady_clock::now();

  cv::Point2f lb_offset;
  float       lb_scale{1.0f};
  cv::Mat     ml_rgb = applyLetterbox(current, lb_offset, lb_scale);

  timing.letterbox_ms = toMs(steady_clock::now() - t_lb);

  lock.unlock();

  // ── Publish: full-res BGR debug image ────────────────────────────────────
  auto out_msg = cv_bridge::CvImage(
    msg->header,
    sensor_msgs::image_encodings::BGR8,
    current).toImageMsg();
  image_pub_->publish(*out_msg);

  // ── Publish: 640×640 RGB for the YOLO node ───────────────────────────────
  // We embed scale/offset as custom fields in the header's frame_id so the
  // detector node can project bounding boxes back to the original frame
  // without needing a separate metadata message.
  // Format: "<original_frame_id>|s=<scale>|ox=<pad_x>|oy=<pad_y>"
  std_msgs::msg::Header ml_header = msg->header;
  ml_header.frame_id =
    msg->header.frame_id +
    "|s="  + std::to_string(lb_scale) +
    "|ox=" + std::to_string(lb_offset.x) +
    "|oy=" + std::to_string(lb_offset.y);

  auto ml_msg = cv_bridge::CvImage(
    ml_header,
    sensor_msgs::image_encodings::RGB8,
    ml_rgb).toImageMsg();
  ml_pub_->publish(*ml_msg);

  publishDiagnostics(timing);
}

}  // namespace ida_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ida_camera::CameraProcessorNode)
