#pragma once
// Read-only VIO-query geometry and native full-state snapshot, never ROS pose covariance.
#include "common_lib.h"
#include "voxel_map.h"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <memory>

namespace fastlivo_visual_snapshot {
using Json=nlohmann::json;
using PlaneMap=std::unordered_map<VOXEL_LOCATION,VoxelOctoTree*>;
struct Owned {
    StatesGroup state;
    std::vector<pointWithVar> points;
    std::vector<std::unique_ptr<VoxelOctoTree>> roots;
    PlaneMap planes;
    Json metadata;
};
Json encode(const StatesGroup&,const std::vector<pointWithVar>&,const PlaneMap&,
            uint64_t source_ns,int64_t native_target_ns,const std::string &generation,
            const std::string &map_frame,const std::string &child_frame,double voxel_size);
Owned decode(const Json&);
Owned read(const std::filesystem::path&);
// Disabled unless explicitly configured; errors are diagnostic, never replace data.
void export_if_requested(const StatesGroup&,const std::vector<pointWithVar>&,const PlaneMap&,
                         double native_target_s,uint64_t original_image_ns,
                         const std::string &map_frame,const std::string &child_frame,
                         double voxel_size) noexcept;
}
