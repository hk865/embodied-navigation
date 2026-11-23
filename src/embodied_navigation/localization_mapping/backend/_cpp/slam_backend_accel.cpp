// Native C++17 / Eigen3 hot-path accelerator for SLAM backend:
//   1. cpp_voxel_downsample   : exact origin-anchored centroid voxel filter
//   2. cpp_icp_point_to_plane : exact 10-NN PCA point-to-plane ICP + MAP gravity prior + Fisher H output
//   3. cpp_optimize_pose_graph: exact SE(3) sparse LM solver with SimplicialLDLT + SparseLU fallback
//
// Pure C ABI for zero-dependency ctypes loading; mathematically identical to the Python reference.

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <vector>

namespace {

using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;

constexpr int kMinPoints = 50;
constexpr int kNormalNeighbors = 10;

inline Mat3 hat3(const Vec3& v) {
    Mat3 K;
    K << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;
    return K;
}

inline Mat3 so3_exp(const Vec3& phi) {
    const double theta = phi.norm();
    const Mat3 K = hat3(phi);
    if (theta < 1e-6) {
        const double t2 = theta * theta;
        return Mat3::Identity() + (1.0 - t2 / 6.0) * K + (0.5 - t2 / 24.0) * (K * K);
    }
    const double s = std::sin(theta) / theta;
    const double c = (1.0 - std::cos(theta)) / (theta * theta);
    return Mat3::Identity() + s * K + c * (K * K);
}

inline Vec3 so3_log(const Mat3& R) {
    // Match scipy Rotation.from_matrix(R).as_rotvec() cleanly across all angles [0, pi].
    const double tr = R.trace();
    const double cos_theta = std::clamp(0.5 * (tr - 1.0), -1.0, 1.0);
    const double theta = std::acos(cos_theta);
    if (theta < 1e-6) {
        const double scale = 0.5 + (theta * theta) / 12.0;
        return Vec3(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1)) * scale;
    }
    if (M_PI - theta < 1e-4) {
        // Near pi: use Eigen::AngleAxisd which handles the pi singularity stably.
        Eigen::AngleAxisd aa(R);
        return aa.angle() * aa.axis();
    }
    const double scale = theta / (2.0 * std::sin(theta));
    return Vec3(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1)) * scale;
}

inline Mat3 left_jacobian(const Vec3& phi) {
    const double theta = phi.norm();
    const Mat3 K = hat3(phi);
    const Mat3 K2 = K * K;
    const double t2 = theta * theta;
    if (theta < 1e-3) {
        return Mat3::Identity() + (0.5 - t2 / 24.0) * K + (1.0 / 6.0 - t2 / 120.0) * K2;
    }
    return Mat3::Identity()
         + ((1.0 - std::cos(theta)) / t2) * K
         + ((theta - std::sin(theta)) / (t2 * theta)) * K2;
}

inline Mat3 so3_left_jacobian_inv(const Vec3& phi) {
    const double theta = phi.norm();
    const Mat3 K = hat3(phi);
    const Mat3 K2 = K * K;
    const double t2 = theta * theta;
    if (theta < 1e-3) {
        return Mat3::Identity() - 0.5 * K + (1.0 / 12.0 + t2 / 720.0) * K2;
    }
    const double coef = (1.0 / t2) - (1.0 + std::cos(theta)) / (2.0 * theta * std::sin(theta));
    return Mat3::Identity() - 0.5 * K + coef * K2;
}

inline Mat3 se3_Q_left(const Vec3& rho, const Vec3& phi) {
    const double theta = phi.norm();
    const Mat3 P = hat3(rho);
    const Mat3 K = hat3(phi);
    const Mat3 K2 = K * K;
    const double t2 = theta * theta;
    double c1, c2, c3;
    if (theta < 1e-2) {
        const double t4 = t2 * t2;
        c1 = 1.0 / 6.0 - t2 / 120.0 + t4 / 5040.0;
        c2 = 1.0 / 24.0 - t2 / 720.0 + t4 / 40320.0;
        c3 = 1.0 / 120.0 - t2 / 2520.0 + t4 / 120960.0;
    } else {
        const double t3 = t2 * theta;
        const double t4 = t2 * t2;
        const double t5 = t4 * theta;
        const double s = std::sin(theta);
        const double c = std::cos(theta);
        c1 = (theta - s) / t3;
        c2 = (t2 + 2.0 * c - 2.0) / (2.0 * t4);
        c3 = (2.0 * theta - 3.0 * s + theta * c) / (2.0 * t5);
    }
    return 0.5 * P
         + c1 * (K * P + P * K + K * P * K)
         + c2 * (K2 * P + P * K2 - 3.0 * K * P * K)
         + c3 * (K * P * K2 + K2 * P * K);
}

inline Mat6 se3_right_jacobian_inv(const Vec6& xi) {
    const Vec3 rho = xi.head<3>();
    const Vec3 phi = xi.tail<3>();
    const Mat3 Jr_phi_inv = so3_left_jacobian_inv(-phi);
    const Mat3 Qr = se3_Q_left(-rho, -phi);
    Mat6 J = Mat6::Zero();
    J.block<3, 3>(0, 0) = Jr_phi_inv;
    J.block<3, 3>(3, 3) = Jr_phi_inv;
    J.block<3, 3>(0, 3) = -(Jr_phi_inv * Qr * Jr_phi_inv);
    return J;
}

inline Mat4 se3_exp(const Vec6& xi) {
    Mat4 T = Mat4::Identity();
    const Vec3 rho = xi.head<3>();
    const Vec3 phi = xi.tail<3>();
    T.block<3, 3>(0, 0) = so3_exp(phi);
    T.block<3, 1>(0, 3) = left_jacobian(phi) * rho;
    return T;
}

inline Vec6 se3_log(const Mat4& T) {
    const Vec3 phi = so3_log(T.block<3, 3>(0, 0));
    const Vec3 rho = left_jacobian(phi).lu().solve(T.block<3, 1>(0, 3));
    Vec6 xi;
    xi.head<3>() = rho;
    xi.tail<3>() = phi;
    return xi;
}

inline Mat6 adjoint_se3(const Mat4& T) {
    const Mat3 R = T.block<3, 3>(0, 0);
    const Vec3 t = T.block<3, 1>(0, 3);
    Mat6 A = Mat6::Zero();
    A.block<3, 3>(0, 0) = R;
    A.block<3, 3>(3, 3) = R;
    A.block<3, 3>(0, 3) = hat3(t) * R;
    return A;
}

inline Mat4 invert_se3(const Mat4& T) {
    Mat4 inv = Mat4::Identity();
    const Mat3 Rt = T.block<3, 3>(0, 0).transpose();
    inv.block<3, 3>(0, 0) = Rt;
    inv.block<3, 1>(0, 3) = -Rt * T.block<3, 1>(0, 3);
    return inv;
}

inline Mat4 load_mat4_rm(const double* ptr) {
    return Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(ptr);
}

inline void store_mat4_rm(const Mat4& M, double* ptr) {
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> out(ptr);
    out = M;
}

inline Mat6 load_mat6_rm(const double* ptr) {
    return Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(ptr);
}

// ---------------------------------------------------------------------------
// Fast array-based 3D KD-Tree (leaf bucket size = 16, zero per-node heap alloc)
// ---------------------------------------------------------------------------
class FlatKDTree3D {
public:
    struct Node {
        int left = -1;
        int right = -1;
        int start = 0;
        int end = 0;
        int axis = 0;
        double split = 0.0;
    };

    explicit FlatKDTree3D(const std::vector<Vec3>& pts) {
        const int n = static_cast<int>(pts.size());
        idx_.resize(n);
        std::iota(idx_.begin(), idx_.end(), 0);
        if (n > 0) {
            nodes_.reserve(std::max(1, n / 4));
            build(pts, 0, n);
            ordered_pts_.resize(n);
            for (int i = 0; i < n; ++i) ordered_pts_[i] = pts[idx_[i]];
        }
    }

    int nearest(const Vec3& q, double max_dist_sq, double& out_dist_sq) const {
        if (nodes_.empty()) return -1;
        double best_d2 = max_dist_sq;
        int best_pos = -1;
        int stack[64];
        int top = 0;
        stack[top++] = 0;
        while (top > 0) {
            const Node& node = nodes_[stack[--top]];
            if (node.left == -1) {
                for (int i = node.start; i < node.end; ++i) {
                    const double d2 = (ordered_pts_[i] - q).squaredNorm();
                    if (d2 < best_d2) {
                        best_d2 = d2;
                        best_pos = i;
                    }
                }
            } else {
                const double diff = q[node.axis] - node.split;
                const int near_child = (diff <= 0.0) ? node.left : node.right;
                const int far_child = (diff <= 0.0) ? node.right : node.left;
                if (diff * diff < best_d2) {
                    stack[top++] = far_child;
                }
                stack[top++] = near_child;
            }
        }
        out_dist_sq = best_d2;
        return (best_pos >= 0) ? idx_[best_pos] : -1;
    }

    void knn(const Vec3& q, int k, std::vector<int>& out_ids) const {
        out_ids.clear();
        if (nodes_.empty() || k <= 0) return;
        std::pair<double, int> heap[32];
        const int cap = std::min(k, 32);
        int count = 0;
        double worst_d2 = std::numeric_limits<double>::infinity();

        int stack[64];
        int top = 0;
        stack[top++] = 0;
        while (top > 0) {
            const Node& node = nodes_[stack[--top]];
            if (node.left == -1) {
                for (int i = node.start; i < node.end; ++i) {
                    const double d2 = (ordered_pts_[i] - q).squaredNorm();
                    if (count < cap) {
                        heap[count++] = {d2, i};
                        if (count == cap) {
                            std::make_heap(heap, heap + cap);
                            worst_d2 = heap[0].first;
                        }
                    } else if (d2 < worst_d2) {
                        std::pop_heap(heap, heap + cap);
                        heap[cap - 1] = {d2, i};
                        std::push_heap(heap, heap + cap);
                        worst_d2 = heap[0].first;
                    }
                }
            } else {
                const double diff = q[node.axis] - node.split;
                const int near_child = (diff <= 0.0) ? node.left : node.right;
                const int far_child = (diff <= 0.0) ? node.right : node.left;
                if (count < cap || diff * diff < worst_d2) {
                    stack[top++] = far_child;
                }
                stack[top++] = near_child;
            }
        }
        out_ids.reserve(count);
        for (int i = 0; i < count; ++i) out_ids.push_back(idx_[heap[i].second]);
    }

private:
    static constexpr int kLeafSize = 16;

    int build(const std::vector<Vec3>& pts, int start, int end) {
        const int my_id = static_cast<int>(nodes_.size());
        nodes_.push_back(Node{});
        const int count = end - start;
        if (count <= kLeafSize) {
            nodes_[my_id].start = start;
            nodes_[my_id].end = end;
            return my_id;
        }
        Vec3 mn = pts[idx_[start]];
        Vec3 mx = mn;
        for (int i = start + 1; i < end; ++i) {
            const Vec3& p = pts[idx_[i]];
            mn = mn.cwiseMin(p);
            mx = mx.cwiseMax(p);
        }
        const Vec3 span = mx - mn;
        int axis = 0;
        if (span.y() > span.x()) axis = 1;
        if (span.z() > span[axis]) axis = 2;

        const int mid = start + count / 2;
        std::nth_element(idx_.begin() + start, idx_.begin() + mid, idx_.begin() + end,
                         [&](int a, int b) { return pts[a][axis] < pts[b][axis]; });
        const double split_val = pts[idx_[mid]][axis];
        const int left_id = build(pts, start, mid);
        const int right_id = build(pts, mid, end);
        nodes_[my_id].axis = axis;
        nodes_[my_id].split = split_val;
        nodes_[my_id].left = left_id;
        nodes_[my_id].right = right_id;
        return my_id;
    }

    std::vector<Vec3> ordered_pts_;
    std::vector<int> idx_;
    std::vector<Node> nodes_;
};

struct VoxelKey {
    int64_t x, y, z;
    bool operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
    bool operator<(const VoxelKey& o) const {
        if (x != o.x) return x < o.x;
        if (y != o.y) return y < o.y;
        return z < o.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& k) const noexcept {
        uint64_t h = static_cast<uint64_t>(k.x) * 73856093ULL
                   ^ static_cast<uint64_t>(k.y) * 19349663ULL
                   ^ static_cast<uint64_t>(k.z) * 83492791ULL;
        h ^= (h >> 33);
        h *= 0xff51afd7ed558ccdULL;
        h ^= (h >> 33);
        return static_cast<std::size_t>(h);
    }
};

struct VoxelAccum {
    double sx = 0.0, sy = 0.0, sz = 0.0;
    int count = 0;
};

inline bool safe_voxel_coord(double x, double voxel_m, int64_t& out) {
    if (!std::isfinite(x)) return false;
    const double q = std::floor(x / voxel_m);
    if (!std::isfinite(q) || std::abs(q) >= 1e15) return false;
    out = static_cast<int64_t>(q);
    return true;
}

bool voxel_downsample_vec(const double* pts, int n, double voxel_m, std::vector<Vec3>& out) {
    out.clear();
    if (n < 0 || !std::isfinite(voxel_m) || voxel_m <= 0.0) return false;
    if (n == 0) return true;
    if (pts == nullptr) return false;
    std::unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash> grid;
    grid.reserve(static_cast<size_t>(n / 2 + 16));
    for (int i = 0; i < n; ++i) {
        const double x = pts[3 * i + 0];
        const double y = pts[3 * i + 1];
        const double z = pts[3 * i + 2];
        int64_t kx = 0, ky = 0, kz = 0;
        if (!safe_voxel_coord(x, voxel_m, kx) ||
            !safe_voxel_coord(y, voxel_m, ky) ||
            !safe_voxel_coord(z, voxel_m, kz)) {
            return false;
        }
        const VoxelKey key{kx, ky, kz};
        auto& acc = grid[key];
        acc.sx += x;
        acc.sy += y;
        acc.sz += z;
        acc.count += 1;
    }
    // Sort keys lexicographically to match np.unique(cells, axis=0) deterministic ordering.
    std::vector<std::pair<VoxelKey, VoxelAccum>> items;
    items.reserve(grid.size());
    for (const auto& kv : grid) items.push_back(kv);
    std::sort(items.begin(), items.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    out.reserve(items.size());
    for (const auto& kv : items) {
        const double inv_c = 1.0 / static_cast<double>(kv.second.count);
        out.emplace_back(kv.second.sx * inv_c, kv.second.sy * inv_c, kv.second.sz * inv_c);
    }
    return true;
}

void compute_normals(const std::vector<Vec3>& pts, const FlatKDTree3D& tree, std::vector<Vec3>& normals) {
    const int n = static_cast<int>(pts.size());
    normals.resize(n);
    std::vector<int> nn;
    nn.reserve(kNormalNeighbors);
    Eigen::SelfAdjointEigenSolver<Mat3> solver;
    for (int i = 0; i < n; ++i) {
        tree.knn(pts[i], kNormalNeighbors, nn);
        Vec3 mean = Vec3::Zero();
        for (int id : nn) mean += pts[id];
        mean /= static_cast<double>(nn.size());
        Mat3 cov = Mat3::Zero();
        for (int id : nn) {
            const Vec3 d = pts[id] - mean;
            cov.noalias() += d * d.transpose();
        }
        solver.computeDirect(cov);
        normals[i] = solver.eigenvectors().col(0);
    }
}

}  // namespace

extern "C" {

// ---------------------------------------------------------------------------
// 1. Voxel downsample: returns number of output points written to out_pts
// ---------------------------------------------------------------------------
int cpp_voxel_downsample(const double* pts, int n, double voxel_m, double* out_pts) {
    try {
        std::vector<Vec3> res;
        if (!voxel_downsample_vec(pts, n, voxel_m, res)) return -1;
        const int m = static_cast<int>(res.size());
        if (m > 0 && out_pts == nullptr) return -1;
        for (int i = 0; i < m; ++i) {
            out_pts[3 * i + 0] = res[i].x();
            out_pts[3 * i + 1] = res[i].y();
            out_pts[3 * i + 2] = res[i].z();
        }
        return m;
    } catch (...) {
        return -1;
    }
}

// ---------------------------------------------------------------------------
// 2. Point-to-plane ICP with optional gravity prior and optional Fisher H out
// ---------------------------------------------------------------------------
void cpp_icp_point_to_plane(
    const double* src_ptr, int n_src,
    const double* tgt_ptr, int n_tgt,
    const double* T_init_ptr,
    double max_correspondence_m,
    int iterations,
    double voxel_m,
    int has_gravity,
    const double* source_up_body_ptr,
    const double* target_up_world_ptr,
    double gravity_sigma_rad,
    double gravity_ref_point_sigma_m,
    double* T_out_ptr,
    double* fitness_out,
    double* rmse_out,
    int* converged_out,
    double* hessian_out  // optional 36 doubles (nullable)
) {
    if (!T_init_ptr || !T_out_ptr || !fitness_out || !rmse_out || !converged_out) return;
    Mat4 T_init = load_mat4_rm(T_init_ptr);
    store_mat4_rm(T_init, T_out_ptr);
    *fitness_out = 0.0;
    *rmse_out = std::numeric_limits<double>::infinity();
    *converged_out = 0;
    if (hessian_out) {
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> hout(hessian_out);
        hout.setZero();
    }
    if (n_src < 0 || n_tgt < 0 || iterations < 0 ||
        !std::isfinite(max_correspondence_m) || max_correspondence_m <= 0.0 ||
        !std::isfinite(voxel_m) || voxel_m < 0.0 ||
        !T_init.allFinite()) {
        return;
    }

    std::vector<Vec3> source;
    if (voxel_m > 0.0) {
        if (!voxel_downsample_vec(src_ptr, n_src, voxel_m, source)) return;
    } else if (n_src > 0) {
        if (!src_ptr) return;
        source.reserve(n_src);
        for (int i = 0; i < n_src; ++i) {
            const Vec3 p(src_ptr[3 * i], src_ptr[3 * i + 1], src_ptr[3 * i + 2]);
            if (!p.allFinite()) return;
            source.push_back(p);
        }
    }
    std::vector<Vec3> target;
    if (voxel_m > 0.0) {
        if (!voxel_downsample_vec(tgt_ptr, n_tgt, voxel_m, target)) return;
    } else if (n_tgt > 0) {
        if (!tgt_ptr) return;
        target.reserve(n_tgt);
        for (int i = 0; i < n_tgt; ++i) {
            const Vec3 p(tgt_ptr[3 * i], tgt_ptr[3 * i + 1], tgt_ptr[3 * i + 2]);
            if (!p.allFinite()) return;
            target.push_back(p);
        }
    }

    if (static_cast<int>(source.size()) < kMinPoints || static_cast<int>(target.size()) < kMinPoints) {
        return;
    }

    bool use_gravity = false;
    Vec3 up_body = Vec3::Zero();
    Vec3 up_world_target(0.0, 0.0, 1.0);
    if (has_gravity && source_up_body_ptr != nullptr &&
        std::isfinite(gravity_sigma_rad) && gravity_sigma_rad > 0.0 &&
        std::isfinite(gravity_ref_point_sigma_m)) {
        Vec3 v(source_up_body_ptr[0], source_up_body_ptr[1], source_up_body_ptr[2]);
        if (!v.allFinite()) return;
        const double nrm = v.norm();
        if (nrm > 1e-6) {
            up_body = v / nrm;
            use_gravity = true;
            if (target_up_world_ptr != nullptr) {
                Vec3 tw(target_up_world_ptr[0], target_up_world_ptr[1], target_up_world_ptr[2]);
                if (!tw.allFinite()) return;
                const double tnrm = tw.norm();
                if (tnrm > 1e-6) {
                    up_world_target = tw / tnrm;
                }
            }
        }
    }

    Vec3 center = Vec3::Zero();
    for (const auto& p : target) center += p;
    center /= static_cast<double>(target.size());

    std::vector<Vec3> target_c(target.size());
    for (size_t i = 0; i < target.size(); ++i) target_c[i] = target[i] - center;

    Mat4 T_c = T_init;
    T_c.block<3, 1>(0, 3) -= center;

    FlatKDTree3D tree(target_c);
    std::vector<Vec3> normals;
    compute_normals(target_c, tree, normals);

    const double max_d2 = max_correspondence_m * max_correspondence_m;
    Mat6 last_body_H = Mat6::Zero();
    bool converged = false;

    for (int iter = 0; iter < iterations; ++iter) {
        const Mat3 R = T_c.block<3, 3>(0, 0);
        const Vec3 t = T_c.block<3, 1>(0, 3);
        Mat6 H = Mat6::Zero();
        Mat6 body_H = Mat6::Zero();
        Vec6 rhs = Vec6::Zero();
        int inliers = 0;
        double sum_p2 = 0.0;

        for (const auto& sp : source) {
            const Vec3 p = R * sp + t;
            double d2 = 0.0;
            const int nid = tree.nearest(p, max_d2, d2);
            if (nid < 0) continue;
            const Vec3& n = normals[nid];
            const double res = (p - target_c[nid]).dot(n);
            Vec6 J;
            J.head<3>() = n;
            J.tail<3>() = p.cross(n);
            H.noalias() += J * J.transpose();
            rhs.noalias() += J * res;
            if (hessian_out) {
                Vec6 J_body;
                J_body.head<3>() = n;
                J_body.tail<3>() = (p - t).cross(n);
                body_H.noalias() += J_body * J_body.transpose();
            }
            sum_p2 += p.squaredNorm();
            inliers += 1;
        }

        if (inliers < kMinPoints) {
            Mat4 T_out = T_c;
            T_out.block<3, 1>(0, 3) += center;
            store_mat4_rm(T_out, T_out_ptr);
            if (hessian_out) {
                Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> hout(hessian_out);
                hout = last_body_H;
            }
            return;
        }

        if (hessian_out) {
            last_body_H = body_H;
        }
        if (use_gravity) {
            const Vec3 u = R * up_body;
            Eigen::Matrix<double, 3, 6> J_g = Eigen::Matrix<double, 3, 6>::Zero();
            J_g.block<3, 3>(0, 3) = -hat3(u);
            const Vec3 r_g = u - up_world_target;
            const double w_g = (gravity_ref_point_sigma_m / gravity_sigma_rad)
                             * (gravity_ref_point_sigma_m / gravity_sigma_rad);
            H.noalias() += w_g * (J_g.transpose() * J_g);
            rhs.noalias() += w_g * (J_g.transpose() * r_g);
        }

        const double scale = std::max(std::sqrt(sum_p2 / static_cast<double>(inliers)), 1.0);
        const double inv_s = 1.0 / scale;
        Vec6 inv_scale;
        inv_scale << 1.0, 1.0, 1.0, inv_s, inv_s, inv_s;
        const Mat6 scaled_h = H.array() * (inv_scale * inv_scale.transpose()).array();
        Eigen::SelfAdjointEigenSolver<Mat6> es(scaled_h, Eigen::EigenvaluesOnly);
        const auto& evals = es.eigenvalues();
        if (evals[0] <= 1e-9 * evals[5]) {
            Mat4 T_out = T_c;
            T_out.block<3, 1>(0, 3) += center;
            store_mat4_rm(T_out, T_out_ptr);
            if (hessian_out) {
                Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> hout(hessian_out);
                hout = last_body_H;
            }
            return;
        }

        const Vec6 step = H.ldlt().solve(-rhs);
        T_c = se3_exp(step) * T_c;
        if (step.head<3>().norm() < 1e-3 && step.tail<3>().norm() < 1e-4) {
            converged = true;
            break;
        }
    }

    // Final scoring pass
    const Mat3 R = T_c.block<3, 3>(0, 0);
    const Vec3 t = T_c.block<3, 1>(0, 3);
    int inliers = 0;
    double sum_r2 = 0.0;
    Mat6 final_H = Mat6::Zero();
    for (const auto& sp : source) {
        const Vec3 p = R * sp + t;
        double d2 = 0.0;
        const int nid = tree.nearest(p, max_d2, d2);
        if (nid < 0) continue;
        const Vec3& n = normals[nid];
        const double res = (p - target_c[nid]).dot(n);
        sum_r2 += res * res;
        inliers += 1;
        if (hessian_out) {
            Vec6 J;
            J.head<3>() = n;
            J.tail<3>() = (p - t).cross(n);
            final_H.noalias() += J * J.transpose();
        }
    }

    Mat4 T_out = T_c;
    T_out.block<3, 1>(0, 3) += center;
    store_mat4_rm(T_out, T_out_ptr);
    *fitness_out = static_cast<double>(inliers) / static_cast<double>(source.size());
    *rmse_out = (inliers > 0) ? std::sqrt(sum_r2 / static_cast<double>(inliers))
                              : std::numeric_limits<double>::infinity();
    *converged_out = converged ? 1 : 0;
    if (hessian_out) {
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> hout(hessian_out);
        hout = (inliers > 0) ? final_H : last_body_H;
    }
}

// ---------------------------------------------------------------------------
// 3. SE(3) Pose Graph LM Optimizer (edges + gravity priors + plane Z priors)
// ---------------------------------------------------------------------------
void cpp_optimize_pose_graph(
    double* poses_ptr, int n_nodes,
    const int* edge_a, const int* edge_b,
    const double* edge_Z_inv, const double* edge_info,
    const int* edge_robust, int n_edges,
    const int* grav_node, const double* grav_body,
    const double* grav_weight, const int* grav_robust, int n_grav,
    const double* grav_world_ptr,
    const int* z_node, const double* z_target,
    const double* z_weight, const int* z_robust, int n_z_priors,
    double cauchy_scale,
    int max_iterations,
    int* out_converged,
    int* out_iterations,
    double* out_error_before,
    double* out_error_after
) {
    Vec3 g_world(0.0, 0.0, 1.0);
    if (grav_world_ptr != nullptr) {
        Vec3 gw(grav_world_ptr[0], grav_world_ptr[1], grav_world_ptr[2]);
        double gwn = gw.norm();
        if (std::isfinite(gwn) && gwn > 1e-6) {
            g_world = gw / gwn;
        }
    }
    std::vector<Mat4> poses(n_nodes);
    for (int i = 0; i < n_nodes; ++i) {
        poses[i] = load_mat4_rm(poses_ptr + 16 * i);
    }

    std::vector<Mat4> Z_invs(n_edges);
    std::vector<Mat6> infos(n_edges);
    for (int e = 0; e < n_edges; ++e) {
        Z_invs[e] = load_mat4_rm(edge_Z_inv + 16 * e);
        infos[e] = load_mat6_rm(edge_info + 36 * e);
    }

    const double c2 = cauchy_scale * cauchy_scale;

    auto compute_cost = [&](const std::vector<Mat4>& cur) -> double {
        double total = 0.0;
        for (int e = 0; e < n_edges; ++e) {
            const int a = edge_a[e];
            const int b = edge_b[e];
            const Vec6 err = se3_log(Z_invs[e] * invert_se3(cur[a]) * cur[b]);
            const double s = err.transpose() * infos[e] * err;
            total += edge_robust[e] ? (c2 * std::log1p(s / c2)) : s;
        }
        for (int g = 0; g < n_grav; ++g) {
            const int ni = grav_node[g];
            const Vec3 gb(grav_body[3 * g], grav_body[3 * g + 1], grav_body[3 * g + 2]);
            const Vec3 u = cur[ni].block<3, 3>(0, 0) * gb;
            const Vec3 eg = u - g_world;
            const double s = grav_weight[g] * eg.squaredNorm();
            total += grav_robust[g] ? (c2 * std::log1p(s / c2)) : s;
        }
        for (int z = 0; z < n_z_priors; ++z) {
            const int ni = z_node[z];
            const double ez = cur[ni](2, 3) - z_target[z];
            const double s = z_weight[z] * ez * ez;
            total += z_robust[z] ? (c2 * std::log1p(s / c2)) : s;
        }
        return total;
    };

    const double before = compute_cost(poses);
    double cost = before;
    double damping = 1e-4;
    bool converged = false;
    int done = 0;
    const int size = 6 * std::max(0, n_nodes - 1);

    if (size == 0 || max_iterations <= 0) {
        *out_converged = 1;
        *out_iterations = 0;
        *out_error_before = before;
        *out_error_after = cost;
        return;
    }

    using Triplet = Eigen::Triplet<double>;
    std::vector<Triplet> triplets;
    triplets.reserve(static_cast<size_t>(n_edges) * 144 + static_cast<size_t>(n_grav) * 9 + static_cast<size_t>(n_z_priors) * 9);
    Eigen::VectorXd grad(size);
    std::vector<Mat4> trial(n_nodes);

    for (int iter = 1; iter <= max_iterations; ++iter) {
        done = iter;
        triplets.clear();
        grad.setZero();

        for (int e = 0; e < n_edges; ++e) {
            const int a = edge_a[e];
            const int b = edge_b[e];
            const Mat4 inv_a = invert_se3(poses[a]);
            const Vec6 err = se3_log(Z_invs[e] * inv_a * poses[b]);
            const double s = err.transpose() * infos[e] * err;
            const double w_scale = edge_robust[e] ? (1.0 / (1.0 + s / c2)) : 1.0;
            const Mat6 W = w_scale * infos[e];
            const Mat6 J_b = se3_right_jacobian_inv(err);
            const Mat6 J_a = -(J_b * adjoint_se3(invert_se3(poses[b]) * poses[a]));

            const int idxs[2] = {a, b};
            const Mat6* Js[2] = {&J_a, &J_b};
            for (int ii = 0; ii < 2; ++ii) {
                const int i = idxs[ii];
                if (i == 0) continue;
                const int row0 = 6 * (i - 1);
                const Mat6& Ji = *Js[ii];
                const Mat6 JiT_W = Ji.transpose() * W;
                grad.segment<6>(row0).noalias() += JiT_W * err;
                for (int jj = 0; jj < 2; ++jj) {
                    const int j = idxs[jj];
                    if (j == 0) continue;
                    const int col0 = 6 * (j - 1);
                    const Mat6 H_ij = JiT_W * (*Js[jj]);
                    for (int r = 0; r < 6; ++r) {
                        for (int c = 0; c < 6; ++c) {
                            const double val = H_ij(r, c);
                            if (val != 0.0) triplets.emplace_back(row0 + r, col0 + c, val);
                        }
                    }
                }
            }
        }

        for (int g = 0; g < n_grav; ++g) {
            const int ni = grav_node[g];
            if (ni == 0) continue;
            const Mat3 R = poses[ni].block<3, 3>(0, 0);
            const Vec3 gb(grav_body[3 * g], grav_body[3 * g + 1], grav_body[3 * g + 2]);
            const Vec3 u = R * gb;
            const Vec3 eg = u - g_world;
            const double s = grav_weight[g] * eg.squaredNorm();
            const double w_eff = grav_robust[g] ? (grav_weight[g] / (1.0 + s / c2)) : grav_weight[g];
            const Mat3 J_rot = -(R * hat3(gb));
            const int slot = 6 * (ni - 1) + 3;
            grad.segment<3>(slot).noalias() += w_eff * (J_rot.transpose() * eg);
            const Mat3 H_rot = w_eff * (J_rot.transpose() * J_rot);
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    const double val = H_rot(r, c);
                    if (val != 0.0) triplets.emplace_back(slot + r, slot + c, val);
                }
            }
        }

        // Optional world-Z floor plane height priors:
        // Under right update T_new = T * exp(rho, phi), t_new = t + R * rho,
        // so dz / drho = R.row(2) (world Z row of R).
        for (int z = 0; z < n_z_priors; ++z) {
            const int ni = z_node[z];
            if (ni == 0) continue;
            const double ez = poses[ni](2, 3) - z_target[z];
            const double s = z_weight[z] * ez * ez;
            const double w_eff = z_robust[z] ? (z_weight[z] / (1.0 + s / c2)) : z_weight[z];
            const Vec3 J_t = poses[ni].block<1, 3>(2, 0).transpose();
            const int slot = 6 * (ni - 1);
            grad.segment<3>(slot).noalias() += (w_eff * ez) * J_t;
            const Mat3 H_t = w_eff * (J_t * J_t.transpose());
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    const double val = H_t(r, c);
                    if (val != 0.0) triplets.emplace_back(slot + r, slot + c, val);
                }
            }
        }

        Eigen::SparseMatrix<double> H(size, size);
        H.setFromTriplets(triplets.begin(), triplets.end());

        // Apply LM diagonal damping: H_damped = H + damping * diag(H), with floor when d == 0
        Eigen::SparseMatrix<double> H_damped = H;
        for (int i = 0; i < size; ++i) {
            const double d = H.coeff(i, i);
            const double diag_scale = (d > 0.0) ? d : 1e-6;
            H_damped.coeffRef(i, i) += damping * diag_scale;
        }
        H_damped.makeCompressed();

        Eigen::VectorXd step;
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt;
        ldlt.compute(H_damped);
        if (ldlt.info() == Eigen::Success) {
            step = ldlt.solve(-grad);
        }
        if (ldlt.info() != Eigen::Success || !step.allFinite()) {
            Eigen::SparseLU<Eigen::SparseMatrix<double>> lu;
            lu.compute(H_damped);
            if (lu.info() != Eigen::Success) {
                damping *= 10.0;
                continue;
            }
            step = lu.solve(-grad);
            if (!step.allFinite()) {
                damping *= 10.0;
                continue;
            }
        }

        trial[0] = poses[0];
        for (int i = 1; i < n_nodes; ++i) {
            trial[i] = poses[i] * se3_exp(step.segment<6>(6 * (i - 1)));
        }
        const double trial_cost = compute_cost(trial);
        const double max_step = step.cwiseAbs().maxCoeff();
        if (trial_cost < cost) {
            poses.swap(trial);
            damping /= 10.0;
            converged = (cost - trial_cost <= 1e-9 * std::max(cost, 1e-12)) || (max_step < 1e-8);
            cost = trial_cost;
        } else {
            damping *= 10.0;
            converged = (max_step < 1e-8);
        }
        if (converged) break;
    }

    for (int i = 0; i < n_nodes; ++i) {
        store_mat4_rm(poses[i], poses_ptr + 16 * i);
    }
    *out_converged = converged ? 1 : 0;
    *out_iterations = done;
    *out_error_before = before;
    *out_error_after = cost;
}

}  // extern "C"

// ---------------------------------------------------------------------------
// 4. Oriented-Normal 3D Planar BA (BALM2 point-cluster Schur complement)
//    jointly solved with PoseGraph edges + calibrated gravity priors.
//    Every per-frame voxel patch normal is oriented toward its observing sensor
//    (ray = mean_w - t_i, n_w . (-ray) > 0) so opposite sides of thin slabs
//    (basement ceiling vs floor-2 floor) and thin interior walls NEVER merge.
// ---------------------------------------------------------------------------
namespace {

struct FramePatchObs {
    int node_idx;
    int count;
    Vec3 sum_b;
    Mat3 M_b;
    Vec3 mean_w;
    Vec3 normal_w;
};

struct OrientedCluster {
    Vec3 ref_mean;
    Vec3 ref_normal;
    std::vector<FramePatchObs> obs;
};

struct SchurTerm {
    int free_idx;
    Eigen::Matrix<double, 6, 3> B;
};

struct ClusterSchur {
    double w_pt;
    Vec3 inv_D;
    int offset;
    int count;
};

}  // namespace

extern "C" {

void cpp_run_oriented_planar_ba(
    double* poses_ptr, int n_nodes, int n_fixed,
    const double* all_pts_b, const int* pts_offsets,
    const int* edge_a, const int* edge_b,
    const double* edge_Z_inv, const double* edge_info,
    const int* edge_robust, int n_edges,
    const int* grav_node, const double* grav_body,
    const double* grav_weight, const int* grav_robust, int n_grav,
    const double* grav_world_ptr,
    double ba_voxel_m,
    int min_frame_pts,
    double plane_ratio_max,
    double plane_thickness_max_m,
    double min_normal_cos,
    double max_plane_offset_m,
    int max_frames_per_cluster,
    double plane_sigma_m,
    double cauchy_scale,
    int outer_rounds,
    int inner_iterations,
    int* out_converged,
    int* out_clusters_used,
    int* out_obs_used,
    double* out_error_before,
    double* out_error_after
) {
    if (!poses_ptr || !pts_offsets || !out_converged || !out_clusters_used ||
        !out_obs_used || !out_error_before || !out_error_after) {
        return;
    }
    *out_converged = 0;
    *out_clusters_used = 0;
    *out_obs_used = 0;
    *out_error_before = 0.0;
    *out_error_after = 0.0;

    if (n_fixed < 1) n_fixed = 1;
    if (n_nodes <= n_fixed || outer_rounds <= 0 || inner_iterations <= 0 ||
        !std::isfinite(ba_voxel_m) || ba_voxel_m <= 0.0 ||
        !std::isfinite(plane_sigma_m) || plane_sigma_m <= 0.0 ||
        !std::isfinite(cauchy_scale) || cauchy_scale <= 0.0) {
        *out_converged = 1;
        return;
    }

    Vec3 g_world(0.0, 0.0, 1.0);
    if (grav_world_ptr != nullptr) {
        Vec3 gw(grav_world_ptr[0], grav_world_ptr[1], grav_world_ptr[2]);
        double gwn = gw.norm();
        if (std::isfinite(gwn) && gwn > 1e-6) {
            g_world = gw / gwn;
        }
    }

    std::vector<Mat4> poses(n_nodes);
    for (int i = 0; i < n_nodes; ++i) {
        poses[i] = load_mat4_rm(poses_ptr + 16 * i);
    }

    std::vector<Mat4> Z_invs(n_edges);
    std::vector<Mat6> infos(n_edges);
    for (int e = 0; e < n_edges; ++e) {
        Z_invs[e] = load_mat4_rm(edge_Z_inv + 16 * e);
        infos[e] = load_mat6_rm(edge_info + 36 * e);
    }

    const double c2 = cauchy_scale * cauchy_scale;
    const double inv_plane_var = 1.0 / (plane_sigma_m * plane_sigma_m);
    const double inv_vox = 1.0 / ba_voxel_m;
    const int eff_min_frame_pts = std::max(3, min_frame_pts);
    const int n_free = n_nodes - n_fixed;
    const int size = 6 * n_free;
    bool overall_converged = false;
    double first_cost = 0.0;
    double last_cost = 0.0;
    int last_clusters = 0;
    int last_obs = 0;

    using Triplet = Eigen::Triplet<double>;
    std::vector<Triplet> triplets;
    Eigen::VectorXd grad(size);
    Eigen::VectorXd schur_diag_sub(size);
    std::vector<SchurTerm> schur_terms;
    std::vector<ClusterSchur> cluster_schurs;
    std::vector<Mat4> trial(n_nodes);

    for (int round = 0; round < outer_rounds; ++round) {
        // 1. Extract per-frame voxel patches with sensor-ray-oriented normals
        std::unordered_map<VoxelKey, std::vector<OrientedCluster>, VoxelKeyHash> voxel_clusters;
        voxel_clusters.reserve(static_cast<size_t>(n_nodes) * 32);

        struct LocalAccum {
            int count = 0;
            Vec3 sum_b = Vec3::Zero();
            Mat3 M_b = Mat3::Zero();
        };

        for (int ni = 0; ni < n_nodes; ++ni) {
            const int p0 = pts_offsets[ni];
            const int p1 = pts_offsets[ni + 1];
            if (p0 < 0 || p1 < p0 || p1 - p0 < eff_min_frame_pts) continue;
            if (all_pts_b == nullptr) continue;
            const Mat3 R = poses[ni].block<3, 3>(0, 0);
            const Vec3 t = poses[ni].block<3, 1>(0, 3);

            std::unordered_map<VoxelKey, LocalAccum, VoxelKeyHash> frame_vox;
            frame_vox.reserve(256);
            for (int k = p0; k < p1; ++k) {
                const Vec3 pb(all_pts_b[3 * k], all_pts_b[3 * k + 1], all_pts_b[3 * k + 2]);
                if (!pb.allFinite()) continue;
                const Vec3 pw = R * pb + t;
                const double qx = std::floor(pw.x() * inv_vox);
                const double qy = std::floor(pw.y() * inv_vox);
                const double qz = std::floor(pw.z() * inv_vox);
                if (!std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz) ||
                    std::abs(qx) > 1e14 || std::abs(qy) > 1e14 || std::abs(qz) > 1e14) {
                    continue;
                }
                VoxelKey key{static_cast<int64_t>(qx), static_cast<int64_t>(qy), static_cast<int64_t>(qz)};
                auto& acc = frame_vox[key];
                acc.count += 1;
                acc.sum_b += pb;
                acc.M_b.noalias() += pb * pb.transpose();
            }

            for (const auto& kv : frame_vox) {
                const auto& acc = kv.second;
                if (acc.count < eff_min_frame_pts) continue;
                const double inv_n = 1.0 / static_cast<double>(acc.count);
                const Vec3 mean_b = acc.sum_b * inv_n;
                const Mat3 cov_b = acc.M_b * inv_n - mean_b * mean_b.transpose();
                Eigen::SelfAdjointEigenSolver<Mat3> es(cov_b);
                if (es.info() != Eigen::Success) continue;
                const Vec3 ev = es.eigenvalues();
                if (ev[1] <= 1e-4) continue;
                const double lam0 = std::max(0.0, ev[0]);
                if (lam0 > plane_ratio_max * ev[1]) continue;
                if (std::sqrt(lam0) > plane_thickness_max_m) continue;

                Vec3 n_w = R * es.eigenvectors().col(0);
                const double nn = n_w.norm();
                if (!std::isfinite(nn) || nn <= 1e-6) continue;
                n_w /= nn;
                const Vec3 mean_w = R * mean_b + t;
                // Orient normal strictly toward the observing sensor origin!
                const Vec3 ray = mean_w - t;
                if (n_w.dot(-ray) < 0.0) {
                    n_w = -n_w;
                }

                FramePatchObs obs{ni, acc.count, acc.sum_b, acc.M_b, mean_w, n_w};
                auto& clist = voxel_clusters[kv.first];
                bool placed = false;
                for (auto& cl : clist) {
                    if (cl.ref_normal.dot(n_w) >= min_normal_cos &&
                        std::abs(cl.ref_normal.dot(mean_w - cl.ref_mean)) <= max_plane_offset_m) {
                        cl.obs.push_back(obs);
                        placed = true;
                        break;
                    }
                }
                if (!placed) {
                    OrientedCluster cl;
                    cl.ref_mean = mean_w;
                    cl.ref_normal = n_w;
                    cl.obs.push_back(obs);
                    clist.push_back(std::move(cl));
                }
            }
        }

        // Flatten multi-frame clusters (and evenly stride to <= max_frames_per_cluster)
        std::vector<std::vector<FramePatchObs>> clusters;
        clusters.reserve(voxel_clusters.size());
        int total_obs = 0;
        for (auto& kv : voxel_clusters) {
            for (auto& cl : kv.second) {
                if (cl.obs.size() < 2) continue;
                if (max_frames_per_cluster > 1 && static_cast<int>(cl.obs.size()) > max_frames_per_cluster) {
                    std::vector<FramePatchObs> sub;
                    sub.reserve(max_frames_per_cluster);
                    const size_t m = cl.obs.size();
                    for (int k = 0; k < max_frames_per_cluster; ++k) {
                        const size_t idx = (static_cast<size_t>(k) * (m - 1)) / static_cast<size_t>(max_frames_per_cluster - 1);
                        sub.push_back(cl.obs[idx]);
                    }
                    total_obs += static_cast<int>(sub.size());
                    clusters.push_back(std::move(sub));
                } else {
                    total_obs += static_cast<int>(cl.obs.size());
                    clusters.push_back(std::move(cl.obs));
                }
            }
        }
        last_clusters = static_cast<int>(clusters.size());
        last_obs = total_obs;
        if (clusters.empty()) {
            overall_converged = true;
            break;
        }

        auto compute_total_cost = [&](const std::vector<Mat4>& cur) -> double {
            double total = 0.0;
            for (int e = 0; e < n_edges; ++e) {
                const int a = edge_a[e];
                const int b = edge_b[e];
                const Vec6 err = se3_log(Z_invs[e] * invert_se3(cur[a]) * cur[b]);
                const double s = err.transpose() * infos[e] * err;
                total += edge_robust[e] ? (c2 * std::log1p(s / c2)) : s;
            }
            for (int g = 0; g < n_grav; ++g) {
                const int ni = grav_node[g];
                const Vec3 gb(grav_body[3 * g], grav_body[3 * g + 1], grav_body[3 * g + 2]);
                const Vec3 u = cur[ni].block<3, 3>(0, 0) * gb;
                const Vec3 eg = u - g_world;
                const double s = grav_weight[g] * eg.squaredNorm();
                total += grav_robust[g] ? (c2 * std::log1p(s / c2)) : s;
            }
            for (const auto& cl : clusters) {
                int N = 0;
                Vec3 sum_w = Vec3::Zero();
                Mat3 M_w = Mat3::Zero();
                for (const auto& o : cl) {
                    const Mat3 R = cur[o.node_idx].block<3, 3>(0, 0);
                    const Vec3 t = cur[o.node_idx].block<3, 1>(0, 3);
                    const Vec3 Rs = R * o.sum_b;
                    N += o.count;
                    sum_w.noalias() += Rs + static_cast<double>(o.count) * t;
                    M_w.noalias() += R * o.M_b * R.transpose()
                                   + Rs * t.transpose() + t * Rs.transpose()
                                   + static_cast<double>(o.count) * (t * t.transpose());
                }
                if (N < 6) continue;
                const double inv_N = 1.0 / static_cast<double>(N);
                const Vec3 mean_w = sum_w * inv_N;
                const Mat3 cov_w = M_w * inv_N - mean_w * mean_w.transpose();
                Eigen::SelfAdjointEigenSolver<Mat3> es(cov_w, Eigen::EigenvaluesOnly);
                if (es.info() != Eigen::Success) continue;
                const double lam0 = std::max(0.0, es.eigenvalues()[0]);
                const double s = lam0 * inv_plane_var;
                const double cl_scale = static_cast<double>(cl.size());
                total += cl_scale * (c2 * std::log1p(s / c2));
            }
            return total;
        };

        double cost = compute_total_cost(poses);
        if (round == 0) first_cost = cost;
        double damping = 1e-3;
        bool round_conv = false;

        for (int iter = 1; iter <= inner_iterations; ++iter) {
            triplets.clear();
            grad.setZero();
            schur_diag_sub.setZero();
            schur_terms.clear();
            cluster_schurs.clear();
            schur_terms.reserve(static_cast<size_t>(total_obs));
            cluster_schurs.reserve(clusters.size());

            // A. PoseGraph odometry + loop edges
            for (int e = 0; e < n_edges; ++e) {
                const int a = edge_a[e];
                const int b = edge_b[e];
                if (a < n_fixed && b < n_fixed) continue;
                const Mat4 inv_a = invert_se3(poses[a]);
                const Vec6 err = se3_log(Z_invs[e] * inv_a * poses[b]);
                const double s = err.transpose() * infos[e] * err;
                const double w_scale = edge_robust[e] ? (1.0 / (1.0 + s / c2)) : 1.0;
                const Mat6 W = w_scale * infos[e];
                const Mat6 J_b = se3_right_jacobian_inv(err);
                const Mat6 J_a = -(J_b * adjoint_se3(invert_se3(poses[b]) * poses[a]));
                const int idxs[2] = {a, b};
                const Mat6* Js[2] = {&J_a, &J_b};
                for (int ii = 0; ii < 2; ++ii) {
                    const int i = idxs[ii];
                    if (i < n_fixed) continue;
                    const int row0 = 6 * (i - n_fixed);
                    const Mat6& Ji = *Js[ii];
                    const Mat6 JiT_W = Ji.transpose() * W;
                    grad.segment<6>(row0).noalias() += JiT_W * err;
                    for (int jj = 0; jj < 2; ++jj) {
                        const int j = idxs[jj];
                        if (j < n_fixed) continue;
                        const int col0 = 6 * (j - n_fixed);
                        const Mat6 H_ij = JiT_W * (*Js[jj]);
                        for (int r = 0; r < 6; ++r) {
                            for (int c = 0; c < 6; ++c) {
                                const double val = H_ij(r, c);
                                if (val != 0.0) triplets.emplace_back(row0 + r, col0 + c, val);
                            }
                        }
                    }
                }
            }

            // B. Calibrated gravity priors
            for (int g = 0; g < n_grav; ++g) {
                const int ni = grav_node[g];
                if (ni < n_fixed) continue;
                const Mat3 R = poses[ni].block<3, 3>(0, 0);
                const Vec3 gb(grav_body[3 * g], grav_body[3 * g + 1], grav_body[3 * g + 2]);
                const Vec3 u = R * gb;
                const Vec3 eg = u - g_world;
                const double s = grav_weight[g] * eg.squaredNorm();
                const double w_eff = grav_robust[g] ? (grav_weight[g] / (1.0 + s / c2)) : grav_weight[g];
                const Mat3 J_rot = -(R * hat3(gb));
                const int slot = 6 * (ni - n_fixed) + 3;
                grad.segment<3>(slot).noalias() += w_eff * (J_rot.transpose() * eg);
                const Mat3 H_rot = w_eff * (J_rot.transpose() * J_rot);
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) {
                        const double val = H_rot(r, c);
                        if (val != 0.0) triplets.emplace_back(slot + r, slot + c, val);
                    }
                }
            }

            // C. Oriented 3D Planar BA via Matrix-Free Implicit Schur Complement
            //    Assembles only M = H_graph + H_diag_BA into triplets (zero O(m^2) fill-in),
            //    and stores compact O(m) Schur factors (B_i, inv_D) for implicit matvec in PCG.
            for (const auto& cl : clusters) {
                int N = 0;
                Vec3 sum_w = Vec3::Zero();
                Mat3 M_w = Mat3::Zero();
                for (const auto& o : cl) {
                    const Mat3 R = poses[o.node_idx].block<3, 3>(0, 0);
                    const Vec3 t = poses[o.node_idx].block<3, 1>(0, 3);
                    const Vec3 Rs = R * o.sum_b;
                    N += o.count;
                    sum_w.noalias() += Rs + static_cast<double>(o.count) * t;
                    M_w.noalias() += R * o.M_b * R.transpose()
                                   + Rs * t.transpose() + t * Rs.transpose()
                                   + static_cast<double>(o.count) * (t * t.transpose());
                }
                if (N < 6) continue;
                const double dN = static_cast<double>(N);
                const double inv_N = 1.0 / dN;
                const Vec3 mean_w = sum_w * inv_N;
                const Mat3 cov_w = M_w * inv_N - mean_w * mean_w.transpose();
                Eigen::SelfAdjointEigenSolver<Mat3> es(cov_w);
                if (es.info() != Eigen::Success) continue;
                const Vec3 ev = es.eigenvalues();
                if (ev[1] <= 1e-4) continue;
                const double lam0 = std::max(0.0, ev[0]);
                const double s = lam0 * inv_plane_var;
                const double w_cauchy = 1.0 / (1.0 + s / c2);
                const double w_pt = (static_cast<double>(cl.size()) * inv_N) * inv_plane_var * w_cauchy;

                const Vec3 n_c = es.eigenvectors().col(0);
                const Vec3 u1 = es.eigenvectors().col(1);
                const Vec3 u2 = es.eigenvectors().col(2);
                const Vec3 inv_D(inv_N, 1.0 / (dN * ev[1] + 1e-6), 1.0 / (dN * ev[2] + 1e-6));

                const int term_offset = static_cast<int>(schur_terms.size());
                int free_count = 0;
                for (const auto& o : cl) {
                    if (o.node_idx < n_fixed) continue;
                    const int free_idx = o.node_idx - n_fixed;
                    const int row0 = 6 * free_idx;
                    const Mat3 R = poses[o.node_idx].block<3, 3>(0, 0);
                    const Vec3 t = poses[o.node_idx].block<3, 1>(0, 3);
                    const Vec3 a_i = R.transpose() * n_c;
                    const Mat3 S = -hat3(a_i);  // S * p_b = p_b x a_i
                    const Vec3 Ss = S * o.sum_b;
                    const Vec3 dt = t - mean_w;
                    const double c_i = n_c.dot(dt);
                    const double n_i = static_cast<double>(o.count);

                    // H_{xi_i, pi_c} (6x3)
                    Eigen::Matrix<double, 6, 3> B;
                    B.block<3, 1>(0, 0) = n_i * a_i;
                    B.block<3, 1>(3, 0) = Ss;

                    const Vec3 b1 = R.transpose() * u1;
                    const double e1 = u1.dot(dt);
                    B.block<3, 1>(0, 1) = (b1.dot(o.sum_b) + n_i * e1) * a_i;
                    B.block<3, 1>(3, 1) = S * (o.M_b * b1 + e1 * o.sum_b);

                    const Vec3 b2 = R.transpose() * u2;
                    const double e2 = u2.dot(dt);
                    B.block<3, 1>(0, 2) = (b2.dot(o.sum_b) + n_i * e2) * a_i;
                    B.block<3, 1>(3, 2) = S * (o.M_b * b2 + e2 * o.sum_b);

                    schur_terms.push_back(SchurTerm{free_idx, B});
                    free_count += 1;

                    // Gradient g_{xi_i} = w_pt * sum_k J_{xi,k}^T r_k
                    Vec6 g_i;
                    g_i.head<3>() = (a_i.dot(o.sum_b) + n_i * c_i) * a_i;
                    g_i.tail<3>() = S * (o.M_b * a_i + c_i * o.sum_b);
                    grad.segment<6>(row0).noalias() += w_pt * g_i;

                    // Diagonal block H_{xi_i, xi_i} -> enters preconditioner M directly
                    Mat6 H_ii;
                    H_ii.block<3, 3>(0, 0) = (n_i * w_pt) * (a_i * a_i.transpose());
                    H_ii.block<3, 3>(0, 3) = w_pt * (a_i * Ss.transpose());
                    H_ii.block<3, 3>(3, 0) = H_ii.block<3, 3>(0, 3).transpose();
                    H_ii.block<3, 3>(3, 3) = w_pt * (S * o.M_b * S.transpose());
                    for (int r = 0; r < 6; ++r) {
                        schur_diag_sub[row0 + r] += w_pt * (
                            B(r, 0) * B(r, 0) * inv_D[0]
                            + B(r, 1) * B(r, 1) * inv_D[1]
                            + B(r, 2) * B(r, 2) * inv_D[2]
                        );
                        for (int c = 0; c < 6; ++c) {
                            const double val = H_ii(r, c);
                            if (val != 0.0) triplets.emplace_back(row0 + r, row0 + c, val);
                        }
                    }
                }
                if (free_count > 0) {
                    cluster_schurs.push_back(ClusterSchur{w_pt, inv_D, term_offset, free_count});
                }
            }

            // Build sparse SPD preconditioner M_damped = H_graph + H_diag_BA + damping * diag(H)
            Eigen::SparseMatrix<double> M_damped(size, size);
            M_damped.setFromTriplets(triplets.begin(), triplets.end());
            for (int i = 0; i < size; ++i) {
                const double d_h = M_damped.coeff(i, i) - schur_diag_sub[i];
                const double diag_scale = (d_h > 0.0) ? d_h : 1e-6;
                M_damped.coeffRef(i, i) += damping * diag_scale;
            }
            M_damped.makeCompressed();

            Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt;
            ldlt.compute(M_damped);
            if (ldlt.info() != Eigen::Success) {
                damping *= 10.0;
                continue;
            }

            // Matrix-free matvec: A * v = M_damped * v - sum_c w_c * B_c * inv(D_c) * (B_c^T * v)
            auto apply_A = [&](const Eigen::VectorXd& v, Eigen::VectorXd& out) {
                out.noalias() = M_damped * v;
                for (const auto& cs : cluster_schurs) {
                    Vec3 u = Vec3::Zero();
                    const int end = cs.offset + cs.count;
                    for (int k = cs.offset; k < end; ++k) {
                        const auto& st = schur_terms[k];
                        u.noalias() += st.B.transpose() * v.segment<6>(6 * st.free_idx);
                    }
                    const Vec3 w_invD_u = cs.w_pt * cs.inv_D.cwiseProduct(u);
                    for (int k = cs.offset; k < end; ++k) {
                        const auto& st = schur_terms[k];
                        out.segment<6>(6 * st.free_idx).noalias() -= st.B * w_invD_u;
                    }
                }
            };

            // Pose-Graph-Preconditioned Conjugate Gradient (exact in M-inner product, O(N) per step)
            const Eigen::VectorXd rhs = -grad;
            const double rhs_norm = rhs.norm();
            Eigen::VectorXd step = Eigen::VectorXd::Zero(size);
            if (rhs_norm > 1e-15) {
                Eigen::VectorXd r = rhs;
                Eigen::VectorXd z = ldlt.solve(r);
                if (!z.allFinite()) {
                    damping *= 10.0;
                    continue;
                }
                Eigen::VectorXd p = z;
                double rz = r.dot(z);
                const double tol_abs = std::max(1e-9, 1e-4 * rhs_norm);
                Eigen::VectorXd Ap(size);
                const int max_cg = std::min(120, std::max(30, size));
                for (int cg = 0; cg < max_cg; ++cg) {
                    if (r.norm() <= tol_abs) break;
                    apply_A(p, Ap);
                    const double pAp = p.dot(Ap);
                    if (!std::isfinite(pAp) || pAp <= 1e-18) {
                        if (cg == 0) step = z;
                        break;
                    }
                    const double alpha = rz / pAp;
                    step.noalias() += alpha * p;
                    r.noalias() -= alpha * Ap;
                    if (r.norm() <= tol_abs) break;
                    z = ldlt.solve(r);
                    if (!z.allFinite()) break;
                    const double rz_new = r.dot(z);
                    if (!std::isfinite(rz_new) || rz <= 1e-30) break;
                    const double beta = rz_new / rz;
                    p = z + beta * p;
                    rz = rz_new;
                }
            }
            if (!step.allFinite()) {
                damping *= 10.0;
                continue;
            }

            for (int i = 0; i < n_fixed; ++i) {
                trial[i] = poses[i];
            }
            for (int i = n_fixed; i < n_nodes; ++i) {
                trial[i] = poses[i] * se3_exp(step.segment<6>(6 * (i - n_fixed)));
            }
            const double trial_cost = compute_total_cost(trial);
            const double max_step = step.cwiseAbs().maxCoeff();
            if (trial_cost < cost) {
                poses.swap(trial);
                damping = std::max(1e-6, damping / 5.0);
                round_conv = (cost - trial_cost <= 1e-7 * std::max(cost, 1e-12)) || (max_step < 1e-6);
                cost = trial_cost;
            } else {
                damping *= 10.0;
                round_conv = (max_step < 1e-7);
            }
            if (round_conv) break;
        }
        last_cost = cost;
        overall_converged = round_conv;
    }

    for (int i = 0; i < n_nodes; ++i) {
        store_mat4_rm(poses[i], poses_ptr + 16 * i);
    }
    *out_converged = overall_converged ? 1 : 0;
    *out_clusters_used = last_clusters;
    *out_obs_used = last_obs;
    *out_error_before = first_cost;
    *out_error_after = last_cost;
}

}  // extern "C"
