#pragma once
#include <math.h>
#include <assert.h>

/**
 * Degrees to radians.
 * @returns Radians
 */
static double deg2rad(const double d) {
  return d * (M_PI / 180.0);
}

/**
 * Radians to degrees.
 * @returns Degrees
 */
static double rad2deg(const double r) {
  return r * (180.0 / M_PI);
}

/**
 * Clip vector `x` to be between `val_min` and `val_max`.
 */
void clip(double *x, const size_t n, const double vmin, const double vmax) {
  for (size_t i = 0; i < n; i++) {
    x[i] = (x[i] > vmax) ? vmax : x[i];
    x[i] = (x[i] < vmin) ? vmin : x[i];
  }
}

/**
 * Dot product of two matrices or vectors `A` and `B` of size `A_m x A_n` and
 * `B_m x B_n`. Results are written to `C`.
 */
void dot(const double *A,
         const size_t A_m,
         const size_t A_n,
         const double *B,
         const size_t B_m,
         const size_t B_n,
         double *C) {
  assert(A != NULL && B != NULL && A != C && B != C);
  assert(A_m > 0 && A_n > 0 && B_m > 0 && B_n > 0);
  assert(A_n == B_m);

  const size_t m = A_m;
  const size_t n = B_n;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      for (size_t k = 0; k < A_n; k++) {
        C[(i * n) + j] += A[(i * A_n) + k] * B[(k * B_n) + j];
      }
    }
  }
}

/**
 * Convert 3x3 rotation matrix `C` to Quaternion `q`.
 */
void rot2quat(const double C[3 * 3], double q[4]) {
  assert(C != NULL);
  assert(q != NULL);

  const double C00 = C[0];
  const double C01 = C[1];
  const double C02 = C[2];
  const double C10 = C[3];
  const double C11 = C[4];
  const double C12 = C[5];
  const double C20 = C[6];
  const double C21 = C[7];
  const double C22 = C[8];

  const double tr = C00 + C11 + C22;
  double S = 0.0f;
  double qw = 0.0f;
  double qx = 0.0f;
  double qy = 0.0f;
  double qz = 0.0f;

  if (tr > 0) {
    S = sqrt(tr + 1.0) * 2; // S=4*qw
    qw = 0.25 * S;
    qx = (C21 - C12) / S;
    qy = (C02 - C20) / S;
    qz = (C10 - C01) / S;
  } else if ((C00 > C11) && (C[0] > C22)) {
    S = sqrt(1.0 + C[0] - C11 - C22) * 2; // S=4*qx
    qw = (C21 - C12) / S;
    qx = 0.25 * S;
    qy = (C01 + C10) / S;
    qz = (C02 + C20) / S;
  } else if (C11 > C22) {
    S = sqrt(1.0 + C11 - C[0] - C22) * 2; // S=4*qy
    qw = (C02 - C20) / S;
    qx = (C01 + C10) / S;
    qy = 0.25 * S;
    qz = (C12 + C21) / S;
  } else {
    S = sqrt(1.0 + C22 - C[0] - C11) * 2; // S=4*qz
    qw = (C10 - C01) / S;
    qx = (C02 + C20) / S;
    qy = (C12 + C21) / S;
    qz = 0.25 * S;
  }

  q[0] = qw;
  q[1] = qx;
  q[2] = qy;
  q[3] = qz;
}

/**
 * Convert gazebo pose vector to homogeneous transformation matrix.
 */
void gzpose2tf(const double *pose, double T[4 * 4]) {
  // Position vector
  const double r[3] = {pose[0], pose[1], pose[2]};

  // Euler to rotation matrix
  const double psi = pose[5];
  const double theta = pose[4];
  const double phi = pose[3];
  const double cpsi = cos(psi);
  const double spsi = sin(psi);
  const double ctheta = cos(theta);
  const double stheta = sin(theta);
  const double cphi = cos(phi);
  const double sphi = sin(phi);
  // -- Form rotation matrix
  double C[3 * 3] = {0};
  // ---- 1st row
  C[0] = cpsi * ctheta;
  C[1] = cpsi * stheta * sphi - spsi * cphi;
  C[2] = cpsi * stheta * cphi + spsi * sphi;
  // ---- 2nd row
  C[3] = spsi * ctheta;
  C[4] = spsi * stheta * sphi + cpsi * cphi;
  C[5] = spsi * stheta * cphi - cpsi * sphi;
  // ---- 3rd row
  C[6] = -stheta;
  C[7] = ctheta * sphi;
  C[8] = ctheta * cphi;

  // Form transformation matrix
  T[0] = C[0];
  T[1] = C[1];
  T[2] = C[2];
  T[3] = r[0];

  T[4] = C[3];
  T[5] = C[4];
  T[6] = C[5];
  T[7] = r[1];

  T[8] = C[6];
  T[9] = C[7];
  T[10] = C[8];
  T[11] = r[2];

  T[12] = 0.0;
  T[13] = 0.0;
  T[14] = 0.0;
  T[15] = 1.0;
}

/**
 * Get the rotation matrix `C` from the 4x4 transformation matrix `T`.
 */
void tf_rot_get(const double T[4 * 4], double C[3 * 3]) {
  assert(T != NULL);
  assert(C != NULL);
  assert(T != C);

  C[0] = T[0];
  C[1] = T[1];
  C[2] = T[2];

  C[3] = T[4];
  C[4] = T[5];
  C[5] = T[6];

  C[6] = T[8];
  C[7] = T[9];
  C[8] = T[10];
}

/**
 * Get the translational vector `r` from the 4x4 transformation matrix `T`.
 */
void tf_trans_get(const double T[4 * 4], double r[3]) {
  assert(T != NULL);
  assert(r != NULL);
  assert(T != r);

  r[0] = T[3];
  r[1] = T[7];
  r[2] = T[11];
}

/**
 * Form 7x1 pose parameter vector `params` from 4x4 homogeneous transformation
 * matrix `T`.
 */
void tf_vector(const double T[4 * 4], double params[7]) {
  assert(T != NULL);
  assert(params != NULL);

  double C[3 * 3] = {0};
  tf_rot_get(T, C);

  double r[3] = {0};
  tf_trans_get(T, r);

  double q[4] = {0};
  rot2quat(C, q);

  params[0] = r[0];
  params[1] = r[1];
  params[2] = r[2];

  params[3] = q[0];
  params[4] = q[1];
  params[5] = q[2];
  params[6] = q[3];
}

/**
 * Pose vector to string.
 */
std::string pose2str(const double pose[7]) {
  std::string str = "";
  str += std::to_string(pose[0]) + ", ";
  str += std::to_string(pose[1]) + ", ";
  str += std::to_string(pose[2]) + ", ";
  str += std::to_string(pose[3]) + ", ";
  str += std::to_string(pose[4]) + ", ";
  str += std::to_string(pose[5]) + ", ";
  str += std::to_string(pose[6]);
  return str;
}
