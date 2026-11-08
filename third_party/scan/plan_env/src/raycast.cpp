#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <limits>
#include <plan_env/raycast.h>

int signum(int x) {
  return x == 0 ? 0 : x < 0 ? -1 : 1;
}

double mod(double value, double modulus) {
  return fmod(fmod(value, modulus) + modulus, modulus);
}

double intbound(double s, double ds) {
  if (ds == 0.0) return std::numeric_limits<double>::infinity();
  return ds > 0.0 ? (std::floor(s)+1.0-s)/ds : (s-std::floor(s))/-ds;
}

void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output) {
  //    std::cout << start << ' ' << end << std::endl;
  // From "A Fast Voxel Traversal Algorithm for Ray Tracing"
  // by John Amanatides and Andrew Woo, 1987
  // <http://www.cse.yorku.ca/~amana/research/grid.pdf>
  // <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>
  // Extensions to the described algorithm:
  //   • Imposed a distance limit.
  //   • The face passed through to reach the current cube is provided to
  //     the callback.

  // The foundation of this algorithm is a parameterized representation of
  // the provided ray,
  //                    origin + t * direction,
  // except that t is not actually stored; rather, at any given point in the
  // traversal, we keep track of the *greater* t values which we would have
  // if we took a step sufficient to cross a cube boundary along that axis
  // (i.e. change the integer part of the coordinate) in the variables
  // tMaxX, tMaxY, and tMaxZ.

  // Cube containing origin point.
  int x = (int)std::floor(start.x());
  int y = (int)std::floor(start.y());
  int z = (int)std::floor(start.z());
  int endX = (int)std::floor(end.x());
  int endY = (int)std::floor(end.y());
  int endZ = (int)std::floor(end.z());
  Eigen::Vector3d direction = (end - start);
  double maxDist = direction.squaredNorm();

  // Break out direction vector.
  double dx = endX - x;
  double dy = endY - y;
  double dz = endZ - z;

  // Direction to increment x,y,z when stepping.
  int stepX = (int)signum((int)dx);
  int stepY = (int)signum((int)dy);
  int stepZ = (int)signum((int)dz);

  // See description above. The initial values depend on the fractional
  // part of the origin.
  double tMaxX = intbound(start.x(), dx);
  double tMaxY = intbound(start.y(), dy);
  double tMaxZ = intbound(start.z(), dz);

  // The change in t when taking a step (always positive).
  double tDeltaX = ((double)stepX) / dx;
  double tDeltaY = ((double)stepY) / dy;
  double tDeltaZ = ((double)stepZ) / dz;

  // Avoids an infinite loop.
  if (stepX == 0 && stepY == 0 && stepZ == 0) return;

  double dist = 0;
  while (true) {
    if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() && z >= min.z() && z < max.z()) {
      output[output_points_cnt](0) = x;
      output[output_points_cnt](1) = y;
      output[output_points_cnt](2) = z;

      output_points_cnt++;
      dist = sqrt((x - start(0)) * (x - start(0)) + (y - start(1)) * (y - start(1)) +
                  (z - start(2)) * (z - start(2)));

      if (dist > maxDist) return;

      /*            if (output_points_cnt > 1500) {
                      std::cerr << "Error, too many racyast voxels." <<
         std::endl;
                      throw std::out_of_range("Too many raycast voxels");
                  }*/
    }

    if (x == endX && y == endY && z == endZ) break;

    // tMaxX stores the t-value at which we cross a cube boundary along the
    // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
    // chooses the closest cube boundary. Only the first case of the four
    // has been commented in detail.
    if (tMaxX < tMaxY) {
      if (tMaxX < tMaxZ) {
        // Update which cube we are now in.
        x += stepX;
        // Adjust tMaxX to the next X-oriented boundary crossing.
        tMaxX += tDeltaX;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    } else {
      if (tMaxY < tMaxZ) {
        y += stepY;
        tMaxY += tDeltaY;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
  }
}

void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output) {
  //    std::cout << start << ' ' << end << std::endl;
  // From "A Fast Voxel Traversal Algorithm for Ray Tracing"
  // by John Amanatides and Andrew Woo, 1987
  // <http://www.cse.yorku.ca/~amana/research/grid.pdf>
  // <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>
  // Extensions to the described algorithm:
  //   • Imposed a distance limit.
  //   • The face passed through to reach the current cube is provided to
  //     the callback.

  // The foundation of this algorithm is a parameterized representation of
  // the provided ray,
  //                    origin + t * direction,
  // except that t is not actually stored; rather, at any given point in the
  // traversal, we keep track of the *greater* t values which we would have
  // if we took a step sufficient to cross a cube boundary along that axis
  // (i.e. change the integer part of the coordinate) in the variables
  // tMaxX, tMaxY, and tMaxZ.

  // Cube containing origin point.
  int x = (int)std::floor(start.x());
  int y = (int)std::floor(start.y());
  int z = (int)std::floor(start.z());
  int endX = (int)std::floor(end.x());
  int endY = (int)std::floor(end.y());
  int endZ = (int)std::floor(end.z());
  Eigen::Vector3d direction = (end - start);
  double maxDist = direction.squaredNorm();

  // Break out direction vector.
  double dx = endX - x;
  double dy = endY - y;
  double dz = endZ - z;

  // Direction to increment x,y,z when stepping.
  int stepX = (int)signum((int)dx);
  int stepY = (int)signum((int)dy);
  int stepZ = (int)signum((int)dz);

  // See description above. The initial values depend on the fractional
  // part of the origin.
  double tMaxX = intbound(start.x(), dx);
  double tMaxY = intbound(start.y(), dy);
  double tMaxZ = intbound(start.z(), dz);

  // The change in t when taking a step (always positive).
  double tDeltaX = ((double)stepX) / dx;
  double tDeltaY = ((double)stepY) / dy;
  double tDeltaZ = ((double)stepZ) / dz;

  output->clear();

  // Avoids an infinite loop.
  if (stepX == 0 && stepY == 0 && stepZ == 0) return;

  double dist = 0;
  while (true) {
    if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() && z >= min.z() && z < max.z()) {
      output->push_back(Eigen::Vector3d(x, y, z));

      dist = (Eigen::Vector3d(x, y, z) - start).squaredNorm();

      if (dist > maxDist) return;

      if (output->size() > 1500) {
        std::cerr << "Error, too many racyast voxels." << std::endl;
        throw std::out_of_range("Too many raycast voxels");
      }
    }

    if (x == endX && y == endY && z == endZ) break;

    // tMaxX stores the t-value at which we cross a cube boundary along the
    // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
    // chooses the closest cube boundary. Only the first case of the four
    // has been commented in detail.
    if (tMaxX < tMaxY) {
      if (tMaxX < tMaxZ) {
        // Update which cube we are now in.
        x += stepX;
        // Adjust tMaxX to the next X-oriented boundary crossing.
        tMaxX += tDeltaX;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    } else {
      if (tMaxY < tMaxZ) {
        y += stepY;
        tMaxY += tDeltaY;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
  }
}

bool RayCaster::setInput(const Eigen::Vector3d& start,
                         const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                         const Eigen::Vector3d& max */) {
  start_ = start;
  end_ = end;
  // max_ = max;
  // min_ = min;

  x_ = (int)std::floor(start_.x());
  y_ = (int)std::floor(start_.y());
  z_ = (int)std::floor(start_.z());
  endX_ = (int)std::floor(end_.x());
  endY_ = (int)std::floor(end_.y());
  endZ_ = (int)std::floor(end_.z());
  direction_ = (end_ - start_);
  maxDist_ = direction_.squaredNorm();

  // Break out direction vector.
  dx_ = direction_.x();
  dy_ = direction_.y();
  dz_ = direction_.z();

  // Direction to increment x,y,z when stepping.
  stepX_ = (dx_ > 0) - (dx_ < 0);
  stepY_ = (dy_ > 0) - (dy_ < 0);
  stepZ_ = (dz_ > 0) - (dz_ < 0);

  // See description above. The initial values depend on the fractional
  // part of the origin.
  tMaxX_ = intbound(start_.x(), dx_);
  tMaxY_ = intbound(start_.y(), dy_);
  tMaxZ_ = intbound(start_.z(), dz_);

  // The change in t when taking a step (always positive).
  tDeltaX_ = stepX_ == 0 ? std::numeric_limits<double>::infinity() : 1.0/std::abs(dx_);
  tDeltaY_ = stepY_ == 0 ? std::numeric_limits<double>::infinity() : 1.0/std::abs(dy_);
  tDeltaZ_ = stepZ_ == 0 ? std::numeric_limits<double>::infinity() : 1.0/std::abs(dz_);

  dist_ = 0;

  step_num_ = 0;
  finished_ = false;

  // Avoids an infinite loop.
  if (stepX_ == 0 && stepY_ == 0 && stepZ_ == 0)
    return false;
  else
    return true;
}

bool RayCaster::step(Eigen::Vector3d& ray_pt) {
  if (finished_) return false;
  // if (x_ >= min_.x() && x_ < max_.x() && y_ >= min_.y() && y_ < max_.y() &&
  // z_ >= min_.z() && z_ <
  // max_.z())
  ray_pt = Eigen::Vector3d(x_, y_, z_);

  // step_num_++;

  // dist_ = (Eigen::Vector3d(x_, y_, z_) - start_).squaredNorm();

  if (x_ == endX_ && y_ == endY_ && z_ == endZ_) {
    return false;
  }

  // if (dist_ > maxDist_)
  // {
  //   return false;
  // }

  // tMaxX stores the t-value at which we cross a cube boundary along the
  // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
  // chooses the closest cube boundary. Only the first case of the four
  // has been commented in detail.
  const double next = std::min(tMaxX_, std::min(tMaxY_, tMaxZ_));
  // A terminal grid-plane tie with mixed direction signs can advance one axis
  // into the endpoint cell while advancing another out of it. End-cell equality
  // then never happens. Stop at the actual segment parameter, after emitting
  // this last positive-length voxel, and never extrapolate beyond the endpoint.
  if (next >= 1.0) { finished_ = true; return true; }
  // A corner/edge crossing advances every tied axis; touching a voxel only at
  // a boundary must not invent observed free volume in the adjacent voxel.
  if (tMaxX_ <= next+1e-12) { x_ += stepX_; tMaxX_ += tDeltaX_; }
  if (tMaxY_ <= next+1e-12) { y_ += stepY_; tMaxY_ += tDeltaY_; }
  if (tMaxZ_ <= next+1e-12) { z_ += stepZ_; tMaxZ_ += tDeltaZ_; }

  return true;
}
