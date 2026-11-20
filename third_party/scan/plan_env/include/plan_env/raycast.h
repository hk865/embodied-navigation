#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <vector>

// Helpers for the Amanatides-Woo voxel traversal (units are voxel indices, the
// caller pre-divides world coordinates by the map resolution).
double signum(double x);

double mod(double value, double modulus);

// Smallest positive t such that s + t*ds is an integer (used for the first
// boundary crossing along an axis).
double intbound(double s, double ds);

// Traverses the integer cells on the segment start->end, keeping only cells
// inside the half-open box [min, max); output_points_cnt is in/out (it must be
// pre-zeroed by the caller).
void Raycast(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const Eigen::Vector3d &min,
             const Eigen::Vector3d &max, int &output_points_cnt, Eigen::Vector3d *output);

void Raycast(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const Eigen::Vector3d &min,
             const Eigen::Vector3d &max, std::vector<Eigen::Vector3d> *output);

class RayCaster {
  private:
    /* data: traversal state, all in voxel-index coordinates except maxDist_ */
    Eigen::Vector3d start_;
    Eigen::Vector3d end_;
    Eigen::Vector3d direction_;
    Eigen::Vector3d min_;
    Eigen::Vector3d max_;
    int x_;
    int y_;
    int z_;
    int endX_;
    int endY_;
    int endZ_;
    double maxDist_;
    double dx_;
    double dy_;
    double dz_;
    int stepX_;
    int stepY_;
    int stepZ_;
    double tMaxX_;
    double tMaxY_;
    double tMaxZ_;
    double tDeltaX_;
    double tDeltaY_;
    double tDeltaZ_;
    double dist_;

    int step_num_;
    bool finished_{false};

  public:
    RayCaster(/* args */) {}
    ~RayCaster() {}

    bool setInput(const Eigen::Vector3d& start,
                const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                const Eigen::Vector3d& max */);

    // Advances to the next voxel crossed by the ray and writes its integer voxel
    // index (the same index space as setInput) into ray_pt; returns false after
    // emitting the end voxel or once the traversal has finished. Callers convert
    // indices to continuous coordinates themselves (e.g. +0.5 then * resolution).
    bool step(Eigen::Vector3d &ray_pt);
};

#endif // RAYCAST_H_
