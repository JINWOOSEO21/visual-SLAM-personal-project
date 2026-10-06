// ORB-SLAM3 단안 래퍼.
//
// 입력   image                    sensor_msgs/Image (헤더 stamp 를 그대로 타임스탬프로 쓴다)
// 출력   /orb_slam3/camera_pose   geometry_msgs/PoseStamped, frame "orb_world"
//                                 T_world_cam, 스케일 없음. tracking 이 OK 일 때만 발행.
//        /orb_slam3/state         std_msgs/Int32MultiArray
//                                 [0] tracking 상태 (ORB_SLAM3::Tracking::eTrackingState)
//                                 [1] 맵 epoch — 새 맵이 초기화될 때마다 1 씩 증가
//                                 [2] 큰 맵 변경 횟수 — loop closure / 맵 병합 / GBA 누적
//        /orb_slam3/map_points    sensor_msgs/PointCloud2, frame "orb_world", 스케일 없음
// 서비스 ~/save_map               std_srvs/Trigger — ORB-SLAM3 를 종료하면서 Atlas 저장
//
// 스케일과 map 프레임 정렬은 scale_aligner 가 맡는다. 이 노드는 ORB-SLAM3 가 내놓는
// 값을 그대로 옮기기만 한다.

#include <unistd.h>

#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "System.h"

namespace fs = std::filesystem;

namespace
{

// launch 에서 넘어온 경로의 "~" 를 풀고 절대경로로 만든다.
std::string expand(const std::string & path)
{
  if (!path.empty() && path[0] == '~') {
    const char * home = std::getenv("HOME");
    return fs::absolute(std::string(home ? home : "") + path.substr(1)).string();
  }
  return fs::absolute(path).string();
}

constexpr int kNotInitialized = ORB_SLAM3::Tracking::NOT_INITIALIZED;
constexpr int kOk = ORB_SLAM3::Tracking::OK;

// 15 Hz 에서 프레임 3 장 이상이 빠지면 누락으로 본다.
constexpr double kFrameGapWarn = 0.2;

const char * state_name(int state)
{
  switch (state) {
    case ORB_SLAM3::Tracking::SYSTEM_NOT_READY: return "SYSTEM_NOT_READY";
    case ORB_SLAM3::Tracking::NO_IMAGES_YET: return "NO_IMAGES_YET";
    case ORB_SLAM3::Tracking::NOT_INITIALIZED: return "NOT_INITIALIZED";
    case ORB_SLAM3::Tracking::OK: return "OK";
    case ORB_SLAM3::Tracking::RECENTLY_LOST: return "RECENTLY_LOST";
    case ORB_SLAM3::Tracking::LOST: return "LOST";
    case ORB_SLAM3::Tracking::OK_KLT: return "OK_KLT";
    default: return "UNKNOWN";
  }
}

}  // namespace

class MonoNode : public rclcpp::Node
{
public:
  MonoNode()
  : Node("orb_slam3")
  {
    const auto vocabulary = expand(declare_parameter<std::string>("vocabulary", ""));
    const auto settings = expand(declare_parameter<std::string>("settings_path", ""));
    const auto atlas = expand(declare_parameter<std::string>("atlas_path", "~/.ros/orbslam3/atlas"));
    mode_ = declare_parameter<std::string>("mode", "slam");
    const bool use_viewer = declare_parameter<bool>("use_viewer", false);
    trajectory_path_ = declare_parameter<std::string>("trajectory_path", "");
    if (!trajectory_path_.empty()) {
      trajectory_path_ = expand(trajectory_path_);
    }
    const double cloud_rate = declare_parameter<double>("map_points_rate", 1.0);

    if (mode_ != "slam" && mode_ != "localization") {
      throw std::runtime_error("mode must be 'slam' or 'localization', got '" + mode_ + "'");
    }

    // ── Atlas 경로 처리 ─────────────────────────────────────────────
    // ORB-SLAM3 는 저장/불러오기 경로를 "./" + 이름 + ".osa" 로 만든다 (System.cc:1411,
    // 1450). 절대경로를 주면 ".//home/..." 이 되어 현재 디렉터리 밑에 엉뚱하게 쓰인다.
    // 그래서 Atlas 디렉터리로 chdir 하고 파일 이름만 넘긴다. 이 노드는 그 밖에 상대
    // 경로를 쓰지 않으므로(위에서 전부 절대경로로 바꿨다) chdir 의 부작용은 없다.
    const fs::path atlas_path(atlas);
    fs::create_directories(atlas_path.parent_path());
    if (chdir(atlas_path.parent_path().c_str()) != 0) {
      throw std::runtime_error("cannot chdir to " + atlas_path.parent_path().string());
    }
    const std::string atlas_name = atlas_path.filename().string();
    if (mode_ == "localization" && !fs::exists(atlas + ".osa")) {
      throw std::runtime_error("mode=localization needs an existing atlas: " + atlas + ".osa");
    }

    // ── 실행용 설정 파일 ─────────────────────────────────────────────
    // Atlas 저장/불러오기는 설정 yaml 의 System.* 키로만 지정할 수 있다
    // (System::SaveAtlas 는 private 이고 Shutdown() 안에서만 불린다). 원본 설정은
    // 건드리지 않고, 복사본에 mode 에 맞는 키를 덧붙여 넘긴다.
    runtime_settings_ = (atlas_path.parent_path() / "runtime_settings.yaml").string();
    {
      std::ifstream in(settings);
      if (!in) {
        throw std::runtime_error("cannot read settings_path: " + settings);
      }
      std::stringstream text;
      text << in.rdbuf();
      // 주석(#) 이 아닌 줄에서 키로 시작할 때만 걸러낸다. 설정 파일 주석에 키 이름을
      // 언급하는 것까지 막으면 안 된다.
      std::istringstream lines(text.str());
      for (std::string line; std::getline(lines, line); ) {
        const auto first = line.find_first_not_of(" \t");
        if (first == std::string::npos || line[first] == '#') {
          continue;
        }
        if (line.compare(first, 22, "System.SaveAtlasToFile") == 0 ||
          line.compare(first, 24, "System.LoadAtlasFromFile") == 0)
        {
          throw std::runtime_error(
                  "settings_path must not set System.SaveAtlasToFile/LoadAtlasFromFile; "
                  "use the atlas_path and mode parameters instead");
        }
      }
      std::ofstream out(runtime_settings_);
      out << text.str() << "\n";
      if (mode_ == "slam") {
        out << "System.SaveAtlasToFile: \"" << atlas_name << "\"\n";
      } else {
        out << "System.LoadAtlasFromFile: \"" << atlas_name << "\"\n";
      }
    }

    RCLCPP_INFO(
      get_logger(), "ORB-SLAM3 시작: mode=%s, atlas=%s.osa, viewer=%s",
      mode_.c_str(), atlas.c_str(), use_viewer ? "on" : "off");
    slam_ = std::make_unique<ORB_SLAM3::System>(
      vocabulary, runtime_settings_, ORB_SLAM3::System::MONOCULAR, use_viewer);
    if (mode_ == "localization") {
      slam_->ActivateLocalizationMode();
    }

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/orb_slam3/camera_pose", 10);
    state_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("/orb_slam3/state", 10);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/orb_slam3/map_points", rclcpp::QoS(1).transient_local());

    // 프레임 간 추적이 전제라 가능한 한 프레임을 버리지 않는다. BEST_EFFORT 구독은
    // RELIABLE 발행자(republish)와도 연결된다.
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image", rclcpp::SensorDataQoS().keep_last(5),
      std::bind(&MonoNode::on_image, this, std::placeholders::_1));

    save_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/save_map",
      std::bind(&MonoNode::on_save, this, std::placeholders::_1, std::placeholders::_2));

    if (cloud_rate > 0.0) {
      cloud_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / cloud_rate), std::bind(&MonoNode::publish_cloud, this));
    }
  }

  ~MonoNode() override {shutdown_and_save();}

private:
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (shut_down_) {
      return;
    }
    cv_bridge::CvImageConstPtr gray;
    try {
      gray = cv_bridge::toCvShare(msg, "mono8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
      return;
    }
    const double stamp = rclcpp::Time(msg->header.stamp).seconds();

    std::lock_guard<std::mutex> lock(slam_mutex_);
    // 직전 프레임과의 간격. 뒤로 가면 ORB-SLAM3 가 맵을 버리고 새로 만들고,
    // 크게 벌어지면 (Wi-Fi 누락) 프레임 사이 움직임이 커져 추적을 잃기 쉽다.
    const double gap = last_stamp_ > 0.0 ? stamp - last_stamp_ : 0.0;
    last_stamp_ = stamp;
    if (gap < 0.0) {
      RCLCPP_WARN(get_logger(), "타임스탬프가 %.0f ms 뒤로 갔다 — ORB-SLAM3 가 새 맵을 만든다", -gap * 1000.0);
    } else if (gap > kFrameGapWarn) {
      ++frame_gaps_;
      RCLCPP_WARN(
        get_logger(), "프레임 간격 %.0f ms (누락 의심, 누적 %d 회)", gap * 1000.0, frame_gaps_);
    }

    const Sophus::SE3f Tcw = slam_->TrackMonocular(gray->image, stamp);
    const int state = slam_->GetTrackingState();

    // 추적 상태가 바뀔 때마다 남긴다. 맵이 자주 새로 만들어질 때 그 직전에 추적 점이
    // 줄어들었는지(텍스처 부족, 급회전), 프레임이 빠졌는지를 로그만으로 가릴 수 있게 한다.
    if (state != prev_state_) {
      RCLCPP_INFO(
        get_logger(), "추적 상태 %s -> %s (직전 OK 프레임 추적 점 %d 개, 프레임 간격 %.0f ms)",
        state_name(prev_state_), state_name(state), last_tracked_points_, gap * 1000.0);
    }
    if (state == kOk) {
      int tracked = 0;
      for (const auto * mp : slam_->GetTrackedMapPoints()) {
        tracked += mp != nullptr;
      }
      last_tracked_points_ = tracked;
    }

    // 새 맵 초기화 = NOT_INITIALIZED -> OK. 추적을 잃고 새 맵을 만들 때마다 일어나며,
    // 그때마다 월드 프레임과 스케일이 바뀐다. scale_aligner 가 이 번호로 재정렬한다.
    if (prev_state_ == kNotInitialized && state == kOk) {
      ++epoch_;
      cloud_.clear();
      RCLCPP_INFO(get_logger(), "새 맵 초기화 (epoch %d)", epoch_);
    }
    if (slam_->MapChanged()) {
      ++big_changes_;
      cloud_.clear();  // loop closure 로 점들이 옮겨졌으니 다시 모은다
      RCLCPP_INFO(get_logger(), "맵 큰 변경 (loop closure / 병합) — 누적 %d 회", big_changes_);
    }
    prev_state_ = state;

    std_msgs::msg::Int32MultiArray st;
    st.data = {state, epoch_, big_changes_};
    state_pub_->publish(st);

    if (state != kOk) {
      return;
    }

    const Sophus::SE3f Twc = Tcw.inverse();
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = msg->header.stamp;
    pose.header.frame_id = "orb_world";
    const Eigen::Vector3f t = Twc.translation();
    const Eigen::Quaternionf q = Twc.unit_quaternion();
    pose.pose.position.x = t.x();
    pose.pose.position.y = t.y();
    pose.pose.position.z = t.z();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    pose_pub_->publish(pose);

    // 전체 맵을 꺼내는 공개 API 가 없어 매 프레임 추적 중인 점을 id 별로 누적한다.
    // 포인터는 보관하지 않고 좌표만 복사한다 (맵 리셋 때 MapPoint 가 해제된다).
    for (ORB_SLAM3::MapPoint * mp : slam_->GetTrackedMapPoints()) {
      if (mp && !mp->isBad() && cloud_.size() < kMaxCloud) {
        cloud_[mp->mnId] = mp->GetWorldPos();
      }
    }
  }

  void publish_cloud()
  {
    sensor_msgs::msg::PointCloud2 cloud;
    {
      std::lock_guard<std::mutex> lock(slam_mutex_);
      sensor_msgs::PointCloud2Modifier mod(cloud);
      mod.setPointCloud2FieldsByString(1, "xyz");
      mod.resize(cloud_.size());
      sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
      for (const auto & kv : cloud_) {
        *x = kv.second.x();
        *y = kv.second.y();
        *z = kv.second.z();
        ++x; ++y; ++z;
      }
    }
    cloud.header.stamp = now();
    cloud.header.frame_id = "orb_world";
    cloud_pub_->publish(cloud);
  }

  void on_save(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (mode_ != "slam") {
      res->success = false;
      res->message = "localization mode does not save the atlas";
      return;
    }
    // SaveAtlas 는 private 이라 Shutdown() 을 거쳐야만 저장된다. 그래서 저장하면
    // 이 노드의 SLAM 은 끝난다. 저장 후 계속 돌리려면 노드를 다시 띄운다.
    shutdown_and_save();
    res->success = true;
    res->message = "ORB-SLAM3 stopped and atlas saved; restart the node to continue";
  }

  void shutdown_and_save()
  {
    if (shut_down_.exchange(true)) {
      return;
    }
    std::lock_guard<std::mutex> lock(slam_mutex_);
    if (!slam_) {
      return;
    }
    RCLCPP_INFO(get_logger(), "ORB-SLAM3 종료 중 (slam 모드면 Atlas 저장)...");
    slam_->Shutdown();
    if (!trajectory_path_.empty()) {
      // 단안은 키프레임 궤적만 저장할 수 있다 (SaveTrajectoryTUM 은 스테레오/RGB-D 전용).
      slam_->SaveKeyFrameTrajectoryTUM(trajectory_path_);
      RCLCPP_INFO(get_logger(), "키프레임 궤적 저장: %s", trajectory_path_.c_str());
    }
  }

  static constexpr size_t kMaxCloud = 300000;

  std::unique_ptr<ORB_SLAM3::System> slam_;
  std::mutex slam_mutex_;
  std::atomic<bool> shut_down_{false};
  std::string mode_;
  std::string runtime_settings_;
  std::string trajectory_path_;

  int prev_state_{ORB_SLAM3::Tracking::NO_IMAGES_YET};
  int epoch_{0};
  int big_changes_{0};
  double last_stamp_{0.0};
  int frame_gaps_{0};
  int last_tracked_points_{0};
  std::unordered_map<unsigned long, Eigen::Vector3f> cloud_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::TimerBase::SharedPtr cloud_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<MonoNode>();
    rclcpp::spin(node);
  }  // 노드 소멸자에서 ORB-SLAM3 를 종료하고 Atlas 를 저장한다
  rclcpp::shutdown();
  return 0;
}
