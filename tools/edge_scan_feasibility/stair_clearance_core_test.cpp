#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "stair_clearance_core.hpp"

using m20pro::StairClearanceConfig;
using m20pro::StairPoint;

static std::vector<StairPoint> staircase(float extra_height = 0.0f) {
  std::vector<StairPoint> points;
  for (int xi = 0; xi < 19; ++xi) {
    const float x = 0.32f + static_cast<float>(xi) * 0.10f;
    const float tread = std::floor((x - 0.30f) / 0.30f) * 0.18f;
    for (int yi = -5; yi <= 5; ++yi) {
      const float y = static_cast<float>(yi) * 0.09f;
      for (int repeat = 0; repeat < 2; ++repeat) {
        points.push_back({x, y, tread + static_cast<float>(repeat) * 0.005f});
      }
    }
  }
  if (extra_height > 0.0f) {
    for (int xi = 8; xi <= 10; ++xi) {
      const float x = 0.32f + static_cast<float>(xi) * 0.10f;
      const float tread = std::floor((x - 0.30f) / 0.30f) * 0.18f;
      for (int sample = 0; sample < 18; ++sample) {
        const float y = -0.12f + static_cast<float>(sample % 6) * 0.045f;
        const float z = tread + 0.30f + static_cast<float>(sample / 6) * extra_height * 0.3f;
        points.push_back({x, y, z});
      }
    }
  }
  return points;
}

static void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

int main() {
  StairClearanceConfig cfg;
  const auto clear = m20pro::classify_stair_clearance(staircase(), cfg);
  require(clear.state == "clear", "regular 0.18m staircase must remain traversable");

  const auto blocked = m20pro::classify_stair_clearance(staircase(0.9f), cfg);
  require(blocked.state == "blocked", "object above the tread envelope must block");
  require(blocked.nearest_obstacle_m < 1.5f, "blocked distance must be bounded");
  require(!blocked.filtered_obstacles.empty(), "blocked scan must retain only residual points");

  std::vector<StairPoint> sparse{{0.5f, 0.0f, 0.0f}};
  const auto unknown = m20pro::classify_stair_clearance(sparse, cfg);
  require(unknown.state == "unknown", "sparse data must fail closed");

  auto discontinuity = staircase();
  for (auto &point : discontinuity) {
    if (point.x > 1.2f) {
      point.z += 0.40f;
    }
  }
  const auto wall = m20pro::classify_stair_clearance(discontinuity, cfg);
  require(wall.state == "blocked", "height discontinuity above one stair must block");

  std::cout << "stair clearance core tests passed" << std::endl;
  return 0;
}
