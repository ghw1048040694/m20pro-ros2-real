#ifndef M20PRO_STAIR_CLEARANCE_CORE_HPP_
#define M20PRO_STAIR_CLEARANCE_CORE_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace m20pro {

struct StairPoint {
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
};

struct StairClearanceConfig {
  float forward_min{0.30f};
  float forward_max{2.20f};
  float half_width{0.55f};
  float height_min{-1.20f};
  float height_max{1.80f};
  float longitudinal_bin{0.10f};
  float base_quantile{0.15f};
  float obstacle_height{0.26f};
  float max_step_height{0.24f};
  int min_corridor_points{80};
  int min_profile_bins{5};
  int min_points_per_bin{5};
  int min_obstacle_points{10};
};

struct StairClearanceResult {
  std::string state{"unknown"};
  std::string reason{"unknown_no_data"};
  int corridor_points{0};
  int profile_bins{0};
  int obstacle_points{0};
  int obstacle_bins{0};
  float nearest_obstacle_m{std::numeric_limits<float>::infinity()};
  std::vector<StairPoint> filtered_obstacles;
};

inline StairClearanceResult classify_stair_clearance(
    const std::vector<StairPoint> &points,
    const StairClearanceConfig &cfg) {
  StairClearanceResult result;
  if (!(cfg.forward_max > cfg.forward_min) || cfg.longitudinal_bin <= 0.0f) {
    result.reason = "unknown_invalid_config";
    return result;
  }

  const int bin_count = std::max(
      1,
      static_cast<int>(std::ceil(
          (cfg.forward_max - cfg.forward_min) / cfg.longitudinal_bin)));
  std::vector<std::vector<StairPoint>> bins(static_cast<size_t>(bin_count));
  for (const auto &point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    if (point.x < cfg.forward_min || point.x > cfg.forward_max ||
        std::abs(point.y) > cfg.half_width ||
        point.z < cfg.height_min || point.z > cfg.height_max) {
      continue;
    }
    int index = static_cast<int>(
        std::floor((point.x - cfg.forward_min) / cfg.longitudinal_bin));
    index = std::max(0, std::min(bin_count - 1, index));
    bins[static_cast<size_t>(index)].push_back(point);
    result.corridor_points += 1;
  }

  std::vector<float> bases(static_cast<size_t>(bin_count), std::numeric_limits<float>::quiet_NaN());
  std::vector<bool> blocked_bins(static_cast<size_t>(bin_count), false);
  for (int index = 0; index < bin_count; ++index) {
    auto &bin = bins[static_cast<size_t>(index)];
    if (static_cast<int>(bin.size()) < cfg.min_points_per_bin) {
      continue;
    }
    std::vector<float> heights;
    heights.reserve(bin.size());
    for (const auto &point : bin) {
      heights.push_back(point.z);
    }
    std::sort(heights.begin(), heights.end());
    const float quantile = std::max(0.0f, std::min(0.49f, cfg.base_quantile));
    const size_t base_index = std::min(
        heights.size() - 1,
        static_cast<size_t>(std::floor(quantile * static_cast<float>(heights.size() - 1))));
    const float base = heights[base_index];
    bases[static_cast<size_t>(index)] = base;
    result.profile_bins += 1;

    std::vector<StairPoint> elevated;
    for (const auto &point : bin) {
      if (point.z - base >= cfg.obstacle_height) {
        elevated.push_back(point);
      }
    }
    if (static_cast<int>(elevated.size()) >= cfg.min_obstacle_points) {
      blocked_bins[static_cast<size_t>(index)] = true;
      result.filtered_obstacles.insert(
          result.filtered_obstacles.end(), elevated.begin(), elevated.end());
    }
  }

  for (int index = 1; index < bin_count; ++index) {
    const float previous = bases[static_cast<size_t>(index - 1)];
    const float current = bases[static_cast<size_t>(index)];
    if (!std::isfinite(previous) || !std::isfinite(current)) {
      continue;
    }
    if (std::abs(current - previous) <= cfg.max_step_height) {
      continue;
    }
    blocked_bins[static_cast<size_t>(index)] = true;
    const auto &bin = bins[static_cast<size_t>(index)];
    result.filtered_obstacles.insert(
        result.filtered_obstacles.end(), bin.begin(), bin.end());
  }

  for (int index = 0; index < bin_count; ++index) {
    if (!blocked_bins[static_cast<size_t>(index)]) {
      continue;
    }
    result.obstacle_bins += 1;
    result.nearest_obstacle_m = std::min(
        result.nearest_obstacle_m,
        cfg.forward_min + static_cast<float>(index) * cfg.longitudinal_bin);
  }
  result.obstacle_points = static_cast<int>(result.filtered_obstacles.size());

  if (result.corridor_points < cfg.min_corridor_points ||
      result.profile_bins < cfg.min_profile_bins) {
    result.state = "unknown";
    result.reason = "unknown_sparse_profile";
    result.filtered_obstacles.clear();
    result.obstacle_points = 0;
    result.obstacle_bins = 0;
    result.nearest_obstacle_m = std::numeric_limits<float>::infinity();
    return result;
  }
  if (result.obstacle_bins > 0) {
    result.state = "blocked";
    result.reason = "blocked_profile_residual";
    return result;
  }
  result.state = "clear";
  result.reason = "clear_stair_profile";
  return result;
}

}  // namespace m20pro

#endif  // M20PRO_STAIR_CLEARANCE_CORE_HPP_
