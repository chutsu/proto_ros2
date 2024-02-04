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

#ifndef FATAL
#define FATAL(...)                                                             \
  printf(__VA_ARGS__);                                                         \
  exit(-1);
#endif

#ifndef UNUSED
#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0);
#endif

/**
 * Convert Degrees to Radians.
 */
double deg2rad(const double d) {
  return d * (M_PI / 180.0);
}

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

  const std::string cmd = "mkdir -p " + path;
  if (system(cmd.c_str()) != 0) {
    return -1;
  }

  return 0;
}

/**
 * Print vector `v` with a `name`.
 */
void print_vector(const std::string &name, const Eigen::VectorXd &v) {
  printf("%s: ", name.c_str());
  for (long i = 0; i < v.size(); i++) {
    printf("%f", v(i));
    if ((i + 1) != v.size()) {
      printf(", ");
    }
  }
  printf("\n");
}

/**
 * Print matrix `m` with a `name`.
 */
void print_matrix(const std::string &name,
                  const Eigen::MatrixXd &m,
                  const std::string &indent) {
  printf("%s:\n", name.c_str());
  for (long i = 0; i < m.rows(); i++) {
    printf("%s", indent.c_str());
    for (long j = 0; j < m.cols(); j++) {
      printf("%f", m(i, j));
      if ((j + 1) != m.cols()) {
        printf(", ");
      }
    }
    printf("\n");
  }
  printf("\n");
}

/**
   * String copy from `src` to `dst`.
   */
size_t string_copy(char *dst, const char *src) {
  dst[0] = '\0';
  memcpy(dst, src, strlen(src));
  dst[strlen(src)] = '\0'; // Null terminate
  return strlen(dst);
}

/**
   * Strip whitespace from string `s`.
   */
char *string_strip(char *s) {
  char *end;

  // Trim leading space
  while (*s == ' ') {
    s++;
  }

  if (*s == 0) { // All spaces?
    return s;
  }

  // Trim trailing space
  end = s + strlen(s) - 1;
  while (end > s && (*end == ' ' || *end == '\n')) {
    end--;
  }

  // Write new null terminator character
  end[1] = '\0';

  return s;
}

/**
   * Strip specific character `c` from string `s`.
   */
char *string_strip_char(char *s, const char c) {
  char *end;

  // Trim leading space
  while (*s == c) {
    s++;
  }

  if (*s == 0) { // All spaces?
    return s;
  }

  // Trim trailing space
  end = s + strlen(s) - 1;
  while (end > s && *end == c) {
    end--;
  }

  // Write new null terminator character
  end[1] = '\0';

  return s;
}

/**
   * Split string `s` by delimiter `d`
   */
char **string_split(char *a_str, const char a_delim, size_t *n) {
  char **result = 0;
  char *tmp = a_str;
  char *last_comma = 0;
  char delim[2];
  delim[0] = a_delim;
  delim[1] = 0;

  /* Count how many elements will be extracted. */
  while (*tmp) {
    if (a_delim == *tmp) {
      (*n)++;
      last_comma = tmp;
    }
    tmp++;
  }

  /* Add space for trailing token. */
  *n += last_comma < (a_str + strlen(a_str) - 1);

  /* Add space for terminating null string so caller
     knows where the list of returned strings ends. */
  (*n)++;
  result = (char **) malloc(sizeof(char *) * *n);

  if (result) {
    size_t idx = 0;
    char *token = strtok(a_str, delim);

    while (token) {
      assert(idx < *n);
      *(result + idx++) = strdup(token);
      token = strtok(0, delim);
    }
    assert(idx == *n - 1);
    *(result + idx) = 0;
  }

  // Return results
  (*n)--;

  return result;
}

/**
   * Skip line
   */
static void parse_skip_line(FILE *fp) {
  assert(fp != NULL);
  const size_t buf_len = 9046;
  char buf[9046] = {0};
  const char *read = fgets(buf, buf_len, fp);
  UNUSED(read);
}

/**
   * Parse integer vector from string line.
   * @returns `0` for success or `-1` for failure
   */
static int parse_vector_line(char *line, const char *type, void *data, int n) {
  assert(line != NULL);
  assert(data != NULL);
  char entry[1024] = {0};
  int index = 0;

  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == '[' || c == ' ') {
      continue;
    }

    if (c == ',' || c == ']' || c == '\n') {
      if (strcmp(type, "int") == 0) {
        ((int *) data)[index] = strtod(entry, NULL);
      } else if (strcmp(type, "double") == 0) {
        ((double *) data)[index] = strtod(entry, NULL);
      } else {
        FATAL("Invalid type [%s]\n", type);
      }
      index++;
      memset(entry, '\0', sizeof(char) * 100);
    } else {
      entry[strlen(entry)] = c;
    }
  }

  if (index != n) {
    return -1;
  }

  return 0;
}

/**
   * Parse key-value pair from string line
   **/
void parse_key_value(FILE *fp,
                     const char *key,
                     const char *value_type,
                     void *value) {
  assert(fp != NULL);
  assert(key != NULL);
  assert(value_type != NULL);
  assert(value != NULL);

  // Parse line
  const size_t buf_len = 1024;
  char buf[1024] = {0};
  if (fgets(buf, buf_len, fp) == NULL) {
    FATAL("Failed to parse [%s]\n", key);
  }

  // Split key-value
  char delim[2] = ":";
  char *key_str = strtok(buf, delim);
  char *value_str = strtok(NULL, delim);
  if (key_str == NULL || value_str == NULL) {
    FATAL("Failed to parse [%s]\n", key);
  }
  key_str = string_strip(key_str);
  value_str = string_strip(value_str);

  // Check key matches
  if (strcmp(key_str, key) != 0) {
    FATAL("Failed to parse [%s]\n", key);
  }

  // Typecase value
  if (value_type == NULL) {
    FATAL("Value type not set!\n");
  }

  // Parse value
  if (strcmp(value_type, "int") == 0) {
    *(int *) value = atoi(value_str);
  } else if (strcmp(value_type, "double") == 0) {
    *(double *) value = atof(value_str);
  } else if (strcmp(value_type, "int64_t") == 0) {
    *(int64_t *) value = atol(value_str);
  } else if (strcmp(value_type, "uint64_t") == 0) {
    *(uint64_t *) value = atol(value_str);
  } else if (strcmp(value_type, "string") == 0) {
    value_str = string_strip_char(value_str, '"');
    string_copy((char *) value, value_str);
  } else if (strcmp(value_type, "vec2i") == 0) {
    parse_vector_line(value_str, "int", value, 2);
  } else if (strcmp(value_type, "vec3i") == 0) {
    parse_vector_line(value_str, "int", value, 3);
  } else if (strcmp(value_type, "vec2d") == 0) {
    parse_vector_line(value_str, "double", value, 2);
  } else if (strcmp(value_type, "vec3d") == 0) {
    parse_vector_line(value_str, "double", value, 3);
  } else if (strcmp(value_type, "vec4d") == 0) {
    parse_vector_line(value_str, "double", value, 4);
  } else if (strcmp(value_type, "vec7d") == 0) {
    parse_vector_line(value_str, "double", value, 7);
  } else if (strcmp(value_type, "pose") == 0) {
    parse_vector_line(value_str, "double", value, 7);
  } else {
    FATAL("Invalid value type [%s]\n", value_type);
  }
}

/* Compare AprilTag */
static bool apriltag_cmp(const AprilTags::TagDetection &a,
                         const AprilTags::TagDetection &b) {
  return (a.id < b.id);
}

/* Detecto AprilGrid */
static int detect_aprilgrid(const AprilTags::AprilGridDetector &detector,
                            const int64_t timestamp,
                            const cv::Mat &image,
                            aprilgrid_t *grid,
                            const int min_tags_threshold = 4,
                            const double min_border_dist = 5.0,
                            const double max_subpix_disp = sqrt(1.5)) {
  // Set aprilgrid timestamp
  grid->timestamp = timestamp;

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
                       const cv::Scalar &color = cv::Scalar{0, 0, 255},
                       const bool label_corner = false) {
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
    if (label_corner) {
      cv::Point2f cxy(kp[0], kp[1]);
      std::string text = std::to_string(tag_id);
      cv::putText(image_rgb,
                  text,
                  cxy,
                  font,
                  font_scale,
                  text_color,
                  thickness);
    }
  }

  free(tag_ids);
  free(corner_indices);
  free(kps);
  free(pts);

  return image_rgb;
}

/**
 * Form rotation matrix around z axis
 */
template <typename T>
Eigen::Matrix<T, 3, 3> ROTZ(const T theta) {
  Eigen::Matrix<T, 3, 3> C;

  C(0, 0) = cos(theta);
  C(0, 1) = -sin(theta);
  C(0, 2) = T(0.0);

  C(1, 0) = sin(theta);
  C(1, 1) = cos(theta);
  C(1, 2) = T(0.0);

  C(2, 0) = T(0.0);
  C(2, 1) = T(0.0);
  C(2, 2) = T(1.0);

  return C;
}

/**
 * Quaternion to rotation matrix.
 */
template <typename T>
Eigen::Matrix<T, 3, 3> quat2rot(const Eigen::Quaternion<T> &q) {
  const T qw = q.w();
  const T qx = q.x();
  const T qy = q.y();
  const T qz = q.z();

  const T qx2 = qx * qx;
  const T qy2 = qy * qy;
  const T qz2 = qz * qz;
  const T qw2 = qw * qw;

  // Homogeneous form
  Eigen::Matrix<T, 3, 3> C;
  // -- 1st row
  C(0, 0) = qw2 + qx2 - qy2 - qz2;
  C(0, 1) = T(2.0) * (qx * qy - qw * qz);
  C(0, 2) = T(2.0) * (qx * qz + qw * qy);
  // -- 2nd row
  C(1, 0) = T(2.0) * (qx * qy + qw * qz);
  C(1, 1) = qw2 - qx2 + qy2 - qz2;
  C(1, 2) = T(2.0) * (qy * qz - qw * qx);
  // -- 3rd row
  C(2, 0) = T(2.0) * (qx * qz - qw * qy);
  C(2, 1) = T(2.0) * (qy * qz + qw * qx);
  C(2, 2) = qw2 - qx2 - qy2 + qz2;

  return C;
}

/**
 * Form transformation matrix.
 */
template <typename T>
Eigen::Matrix<T, 4, 4> transform(const T *params) {
  const Eigen::Quaternion<T> q{params[3], params[4], params[5], params[6]};
  const Eigen::Vector<T, 3> r{params[0], params[1], params[2]};

  Eigen::Matrix<T, 4, 4> transform;
  transform.setIdentity();
  transform.block(0, 0, 3, 3) = quat2rot(q);
  transform.block(0, 3, 3, 1) = r;

  return transform;
}

/**
 * Form transformation matrix.
 */
Eigen::VectorXd transform_vector(const Eigen::Matrix4d &T) {
  const Eigen::Vector3d r = T.block<3, 1>(0, 3);
  const Eigen::Quaterniond q{T.block<3, 3>(0, 0)};
  Eigen::VectorXd vec;
  vec.resize(7);
  vec << r.x(), r.y(), r.z(), q.w(), q.x(), q.y(), q.z();
  return vec;
}

/**
 * Gimbal joint angle to transformation matrix.
 */
template <typename T>
Eigen::Matrix<T, 4, 4> gimbal_joint_transform(const T angle) {
  Eigen::Matrix<T, 4, 4> transform;
  transform.setIdentity();
  transform.block(0, 0, 3, 3) = ROTZ(angle);
  return transform;
}

/**
 * Project point
 */
Eigen::Vector2d project_point(const Eigen::Vector3d &p_C) {
  return Eigen::Vector2d{p_C.x() / p_C.z(), p_C.y() / p_C.z()};
}

/**
 * Radtan4 Distort
 */
Eigen::Vector2d radtan4_distort(const Eigen::Vector4d &dist_params,
                                const Eigen::Vector2d &p) {
  const real_t x = p.x();
  const real_t y = p.y();

  const real_t k1 = dist_params(0);
  const real_t k2 = dist_params(1);
  const real_t p1 = dist_params(2);
  const real_t p2 = dist_params(3);

  // Apply radial distortion
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;
  const real_t radial_factor = 1.0 + (k1 * r2) + (k2 * r4);
  const real_t x_dash = x * radial_factor;
  const real_t y_dash = y * radial_factor;

  // Apply tangential distortion
  const real_t xy = x * y;
  const real_t x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2));
  const real_t y_ddash = y_dash + (2.0 * p2 * xy + p1 * (r2 + 2.0 * y2));

  return Eigen::Vector2d{x_ddash, y_ddash};
}

/**
 * Radtan4 Point Jacobian
 */
Eigen::Matrix2d radtan4_point_jacobian(const Eigen::Vector4d &dist_params,
                                       const Eigen::Vector2d &p) {
  const real_t x = p(0);
  const real_t y = p(1);

  const real_t k1 = dist_params(0);
  const real_t k2 = dist_params(1);
  const real_t p1 = dist_params(2);
  const real_t p2 = dist_params(3);

  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;

  // Let p = [x; y] normalized point
  // Let p' be the distorted p
  // The jacobian of p' w.r.t. p (or dp'/dp) is:
  Eigen::Matrix2d J_point;
  J_point(0, 0) = 1.0 + k1 * r2 + k2 * r4;
  J_point(0, 0) += 2.0 * p1 * y + 6.0 * p2 * x;
  J_point(0, 0) += x * (2.0 * k1 * x + 4.0 * k2 * x * r2);
  J_point(1, 0) = 2.0 * p1 * x + 2.0 * p2 * y;
  J_point(1, 0) += y * (2.0 * k1 * x + 4.0 * k2 * x * r2);
  J_point(0, 1) = J_point(1, 0);
  J_point(1, 1) = 1.0 + k1 * r2 + k2 * r4;
  J_point(1, 1) += 6.0 * p1 * y + 2.0 * p2 * x;
  J_point(1, 1) += y * (2.0 * k1 * y + 4.0 * k2 * y * r2);

  return J_point;
}

/**
 * Radtan4 Undistort
 */
Eigen::Vector2d radtan4_undistort(const Eigen::Vector4d &dist_params,
                                  const Eigen::Vector2d &p0) {
  int max_iter = 5;
  Eigen::Vector2d p = p0;

  for (int i = 0; i < max_iter; i++) {
    // Error
    const Eigen::Vector2d p_distorted = radtan4_distort(dist_params, p);
    const Eigen::Vector2d err = (p0 - p_distorted);

    // Jacobian
    const Eigen::Matrix2d J = radtan4_point_jacobian(dist_params, p);
    const Eigen::Vector2d dp =
        (J.transpose() * J).inverse() * J.transpose() * err;
    p = p + dp;

    if ((err.transpose() * err) < 1.0e-15) {
      break;
    }
  }

  return p;
}

/**
 * Pinhole-Radtan4 Undistort
 */
Eigen::Vector2d pinhole_radtan4_undistort(const Eigen::VectorXd &params,
                                          const Eigen::Vector2d &z) {
  // Back-project and undistort
  const real_t fx = params(0);
  const real_t fy = params(1);
  const real_t cx = params(2);
  const real_t cy = params(3);
  const real_t px = (z.x() - cx) / fx;
  const real_t py = (z.y() - cy) / fy;
  const Eigen::Vector2d p{px, py};
  const Eigen::Vector2d p_undist = radtan4_undistort(params.tail(4), p);

  // Project undistorted point to image plane
  const real_t x = p_undist.x() * fx + cx;
  const real_t y = p_undist.y() * fy + cy;
  const Eigen::Vector2d z_undist = {x, y};

  return z_undist;
}

/**
 * Pinhole-Radtan4 Project
 */
int pinhole_radtan4_project(const int res[2],
                            const Eigen::VectorXd &params,
                            const Eigen::Vector3d &p_C,
                            Eigen::Vector2d &z_hat) {
  // Setup
  const Eigen::Vector4d proj_params = params.head(4);
  const Eigen::Vector4d dist_params = params.tail(4);

  // Project, distort and then scale and center
  const real_t fx = proj_params(0);
  const real_t fy = proj_params(1);
  const real_t cx = proj_params(2);
  const real_t cy = proj_params(3);
  const Eigen::Vector2d p = project_point(p_C);
  const Eigen::Vector2d p_d = radtan4_distort(dist_params, p);
  z_hat.x() = fx * p_d.x() + cx;
  z_hat.y() = fy * p_d.y() + cy;

  // Check projection is within image frame
  const bool x_ok = (z_hat.x() >= 0 && z_hat.x() < res[0]);
  const bool y_ok = (z_hat.y() >= 0 && z_hat.y() < res[1]);
  const bool z_ok = (p_C.z() > 0.0);
  const bool valid = (x_ok && y_ok && z_ok) ? true : false;

  return (valid) ? 0 : -1;
}

/**
 * Convert cv::Mat to Eigen::Matrix
 */
void convert(const cv::Mat &x, Eigen::MatrixXd &y) {
  y.resize(x.rows, x.cols);

  for (int i = 0; i < x.rows; i++) {
    for (int j = 0; j < x.cols; j++) {
      y(i, j) = x.at<real_t>(i, j);
    }
  }
}

/**
 * Convert cv::Mat to Eigen::Matrix
 */
Eigen::MatrixXd convert(const cv::Mat &x) {
  Eigen::MatrixXd y;
  convert(x, y);
  return y;
}

/**
 * SolvePnP
 */
int solvepnp(const int cam_res[2],
             const Eigen::VectorXd &cam_params,
             const std::vector<Eigen::Vector2d> &keypoints,
             const std::vector<Eigen::Vector3d> &object_points,
             Eigen::Matrix4d &T_CF) {
  UNUSED(cam_res);
  assert(keypoints.size() == object_points.size());

  // Create object points (counter-clockwise, from bottom left)
  // Note: SolvPnP assumes radtan which may not be true, therefore we
  // have to manually undistort the keypoints ourselves
  size_t nb_points = keypoints.size();
  std::vector<cv::Point2f> img_pts;
  std::vector<cv::Point3f> obj_pts;
  for (size_t i = 0; i < nb_points; i++) {
    // Check keypoint is valid
    const Eigen::Vector2d z = keypoints[i];
    const bool x_ok = (z.x() >= 0 && z.x() <= cam_res[0]);
    const bool y_ok = (z.y() >= 0 && z.y() <= cam_res[1]);
    const bool valid = (x_ok && y_ok) ? true : false;
    if (valid == false) {
      printf("INVALID!\n");
      continue;
    }

    // Keypoint
    const Eigen::Vector2d &kp = pinhole_radtan4_undistort(cam_params, z);
    img_pts.emplace_back(kp.x(), kp.y());

    // Object point
    const Eigen::Vector3d &pt = object_points[i];
    obj_pts.emplace_back(pt.x(), pt.y(), pt.z());
  }

  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = cam_params(0);
  K.at<double>(1, 1) = cam_params(1);
  K.at<double>(0, 2) = cam_params(2);
  K.at<double>(1, 2) = cam_params(3);

  cv::Mat D = cv::Mat::zeros(4, 1, CV_64F);
  cv::Mat rvec(3, 1, CV_64F);
  cv::Mat tvec(3, 1, CV_64F);
  cv::solvePnP(obj_pts, img_pts, K, D, rvec, tvec);

  // Form relative tag pose as a 4x4 tfation matrix
  // -- Convert Rodrigues rotation vector to rotation matrix
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  // -- Form full transformation matrix
  T_CF.setIdentity();
  T_CF.block<3, 3>(0, 0) = convert(R);
  T_CF.block<3, 1>(0, 3) = convert(tvec);

  return 0;
}
