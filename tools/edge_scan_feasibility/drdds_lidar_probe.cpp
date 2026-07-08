#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "drdds/core/common_type.h"

using Clock = std::chrono::steady_clock;

struct Stats {
  std::atomic<int> samples{0};
  std::atomic<int> points{0};
  std::atomic<int> bytes{0};
  std::string frame;
  Clock::time_point first{};
  Clock::time_point last{};
};

int main(int argc, char **argv) {
  std::string topic = argc > 1 ? argv[1] : "/LIDAR/POINTS";
  int domain = argc > 2 ? std::stoi(argv[2]) : 0;
  bool use_shm = argc > 3 ? std::stoi(argv[3]) != 0 : false;
  std::string prefix = argc > 4 ? argv[4] : "rt";
  int duration_s = argc > 5 ? std::stoi(argv[5]) : 8;

  Stats stats;
  auto cb = [&stats](const sensor_msgs::msg::PointCloud2 *msg) {
    auto now = Clock::now();
    int prev = stats.samples.fetch_add(1);
    if (prev == 0) {
      stats.first = now;
    }
    stats.last = now;

    uint32_t height = msg->height();
    if (height == 0) {
      height = 1;
    }
    stats.points.store(static_cast<int>(msg->width() * height));
    stats.bytes.store(static_cast<int>(msg->data().size()));
    stats.frame = msg->header().frame_id();

    if (prev < 3) {
      std::cout << "sample " << (prev + 1)
                << " frame=" << stats.frame
                << " width=" << msg->width()
                << " height=" << msg->height()
                << " point_step=" << msg->point_step()
                << " bytes=" << msg->data().size()
                << std::endl;
    }
  };

  DrDDSManager::Init(domain, "");
  ChannelLidarPointCloud channel(cb, topic, domain, use_shm, prefix);
  std::cout << "probe_start topic=" << topic
            << " domain=" << domain
            << " use_shm=" << use_shm
            << " prefix=" << prefix
            << std::endl;

  auto end = Clock::now() + std::chrono::seconds(duration_s);
  while (Clock::now() < end) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  int sample_count = stats.samples.load();
  double rate_hz = 0.0;
  if (sample_count > 1 && stats.last > stats.first) {
    rate_hz = static_cast<double>(sample_count - 1) /
              std::chrono::duration<double>(stats.last - stats.first).count();
  }

  std::cout << "samples=" << sample_count
            << " rate_hz=" << rate_hz
            << " points=" << stats.points.load()
            << " bytes=" << stats.bytes.load()
            << " frame=" << stats.frame
            << std::endl;

  DrDDSManager::Delete();
  return sample_count > 0 ? 0 : 1;
}

