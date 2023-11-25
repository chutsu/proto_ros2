#pragma once

#include <iostream>
#include <sys/stat.h>
#include <mutex>
#include <time.h>

#include <Eigen/Core>
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag36h11.h>
// NOTE: Order matters with the AprilTags lib by Michael Kaess. The detector first.

#define APRILGRID_IMPLEMENTATION
#include <aprilgrid.h>

/**
 * Timestamp now
 */
int64_t time_now() {
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);

  const time_t sec = spec.tv_sec;
  const long int ns = spec.tv_nsec;
  const uint64_t BILLION = 1000000000L;

  return (int64_t) sec * BILLION + (int64_t) ns;
}

/* Linspace */
static std::vector<double> linspace(double start, double end, int numPoints) {
  std::vector<double> result;

  if (numPoints <= 1) {
    result.push_back(start);
    return result;
  }

  double step = (end - start) / (numPoints - 1);
  for (int i = 0; i < numPoints; ++i) {
    result.push_back(start + i * step);
  }

  return result;
}

/* Check if path exists */
static int path_exists(const std::string &path) {
  struct stat buffer;
  return stat(path.c_str(), &buffer) == 0;
}

/* Create directory */
static int dir_create(const std::string &path) {
  if (path_exists(path)) {
    return 0;
  }

  if (mkdir(path.c_str(), 0777) != 0) {
    return -1;
  }

  return 0;
}

/* Compare AprilTag */
static bool apriltag_cmp(const AprilTags::TagDetection &a,
                         const AprilTags::TagDetection &b) {
  return (a.id < b.id);
}

/* Detecto AprilGrid */
static int detect_aprilgrid(const AprilTags::AprilGridDetector &detector,
                            const cv::Mat &image,
                            aprilgrid_t *grid,
                            const int min_tags_threshold = 4,
                            const double min_border_dist = 5.0,
                            const double max_subpix_disp = sqrt(1.5)) {
  // Use AprilTags by Michael Kaess
  std::vector<AprilTags::TagDetection> tags = detector.extractTags(image);

  // -- Check if too few measurements
  if (tags.size() < (size_t) min_tags_threshold) {
    return -1;
  }

  // -- Sort Tags by ID
  std::sort(tags.begin(), tags.end(), apriltag_cmp);

  // -- Get measurement data
  const int img_cols = image.cols;
  const int img_rows = image.rows;
  const cv::Size win_size(2, 2);
  const cv::Size zero_zone(-1, -1);
  const int type = cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER;
  const cv::TermCriteria criteria(type, 30, 0.1);

  std::vector<int> tag_ids;
  std::vector<int> corner_indices;
  std::vector<std::pair<double, double>> keypoints;

  for (auto &tag : tags) {
    // Check if tag detection is good
    if (tag.good == false) {
      continue;
    }

    // Check if tag id is out of range for this grid
    if (tag.id >= (grid->num_rows * grid->num_cols)) {
      continue;
    }

    // Check if too close to image bounds
    bool bad_tag = false;
    for (int i = 0; i < 4; i++) {
      bad_tag |= tag.p[i].first < min_border_dist;
      bad_tag |= tag.p[i].first > img_cols - min_border_dist;
      bad_tag |= tag.p[i].second < min_border_dist;
      bad_tag |= tag.p[i].second > img_rows - min_border_dist;
      if (bad_tag) {
        break;
      }
    }
    if (bad_tag) {
      continue;
    }

    // Corner subpixel refinement
    std::vector<cv::Point2f> corners_before;
    std::vector<cv::Point2f> corners_after;
    corners_before.emplace_back(tag.p[0].first, tag.p[0].second);
    corners_before.emplace_back(tag.p[1].first, tag.p[1].second);
    corners_before.emplace_back(tag.p[2].first, tag.p[2].second);
    corners_before.emplace_back(tag.p[3].first, tag.p[3].second);
    corners_after.emplace_back(tag.p[0].first, tag.p[0].second);
    corners_after.emplace_back(tag.p[1].first, tag.p[1].second);
    corners_after.emplace_back(tag.p[2].first, tag.p[2].second);
    corners_after.emplace_back(tag.p[3].first, tag.p[3].second);
    cv::cornerSubPix(image, corners_after, win_size, zero_zone, criteria);

    // -- Check euclidean distance
    for (size_t i = 0; i < 4; i++) {
      // Subpixel distance
      const auto &p_before = corners_before[i];
      const auto &p_after = corners_after[i];
      const auto dx = p_before.x - p_after.x;
      const auto dy = p_before.y - p_after.y;
      const auto dist = sqrt(dx * dx + dy * dy);
      if (dist > max_subpix_disp) {
        // bad_tag = true;
        continue;
      }
      tag.p[i].first = p_after.x;
      tag.p[i].second = p_after.y;

      // Add to results
      tag_ids.push_back(tag.id);
      corner_indices.push_back(i);
      keypoints.push_back({tag.p[i].first, tag.p[i].second});
    }
  }

  // -- Check if too few measurements
  if (tag_ids.size() < (size_t) (4 * min_tags_threshold)) {
    return -1;
  }

  // -- Add filtered tags to grid
  for (size_t i = 0; i < tag_ids.size(); i++) {
    const int tag_id = tag_ids[i];
    const int corner_idx = corner_indices[i];
    const double kp[2] = {keypoints[i].first, keypoints[i].second};
    aprilgrid_add_corner(grid, tag_id, corner_idx, kp);
  }

  return 0;
}

/**
 * Draw aprilgrid
 */
cv::Mat aprilgrid_draw(const aprilgrid_t *grid,
                       const cv::Mat &image,
                       const int marker_size = 2,
                       const cv::Scalar &color = cv::Scalar{0, 0, 255}) {
  const cv::Scalar text_color(0, 255, 0);
  const int font = cv::FONT_HERSHEY_PLAIN;
  const real_t font_scale = 1.0;
  const int thickness = 2;
  cv::Mat image_rgb;
  cv::cvtColor(image, image_rgb, cv::COLOR_GRAY2BGR);

  const int num_corners = grid->corners_detected;
  int *tag_ids = MALLOC(int, num_corners);
  int *corner_indices = MALLOC(int, num_corners);
  real_t *kps = MALLOC(real_t, num_corners * 2);
  real_t *pts = MALLOC(real_t, num_corners * 3);
  aprilgrid_measurements(grid, tag_ids, corner_indices, kps, pts);

  for (size_t i = 0; i < num_corners; i++) {
    // Setup
    const int tag_id = tag_ids[i];
    const real_t *kp = &kps[i * 2];

    // Draw corners
    cv::Point2f p(kp[0], kp[1]);
    cv::circle(image_rgb, p, marker_size, color, -1);

    // Label corner
    cv::Point2f cxy(kp[0], kp[1]);
    std::string text = std::to_string(tag_id);
    cv::putText(image_rgb, text, cxy, font, font_scale, text_color, thickness);
  }

  free(tag_ids);
  free(corner_indices);
  free(kps);
  free(pts);

  return image_rgb;
}
