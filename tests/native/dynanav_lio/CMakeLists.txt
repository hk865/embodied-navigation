cmake_minimum_required(VERSION 3.16)
project(dynanav_lio_native_checks LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fast_livo2_core REQUIRED)
find_package(image_transport REQUIRED)
find_package(Sophus REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vikit_ros REQUIRED)
find_package(OpenMP REQUIRED)
find_package(fmt REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
add_executable(test_dynanav_lio test_lio_mode.cpp)
get_filename_component(native_prefix "${fast_livo2_core_DIR}/../../.." ABSOLUTE)
target_include_directories(test_dynanav_lio PRIVATE "${native_prefix}/include")
target_compile_definitions(test_dynanav_lio PRIVATE X86_ARCH ROOT_DIR="${CMAKE_CURRENT_BINARY_DIR}/")
ament_target_dependencies(test_dynanav_lio rclcpp fast_livo2_core image_transport)
target_link_libraries(test_dynanav_lio fast_livo2_core::fast_livo2_core)
enable_testing()
add_test(NAME dynanav_lio_native COMMAND test_dynanav_lio)

add_executable(test_dynanav_post_vio test_post_vio_output.cpp)
target_include_directories(test_dynanav_post_vio PRIVATE "${native_prefix}/include")
target_compile_definitions(test_dynanav_post_vio PRIVATE X86_ARCH ROOT_DIR="${CMAKE_CURRENT_BINARY_DIR}/")
ament_target_dependencies(test_dynanav_post_vio rclcpp fast_livo2_core image_transport)
target_link_libraries(test_dynanav_post_vio fast_livo2_core::fast_livo2_core)
add_test(NAME dynanav_post_vio_native COMMAND test_dynanav_post_vio)

add_executable(test_dynanav_ouster test_ouster_mode.cpp)
target_include_directories(test_dynanav_ouster PRIVATE "${native_prefix}/include")
target_compile_definitions(test_dynanav_ouster PRIVATE X86_ARCH ROOT_DIR="${CMAKE_CURRENT_BINARY_DIR}/")
ament_target_dependencies(test_dynanav_ouster rclcpp fast_livo2_core image_transport)
target_link_libraries(test_dynanav_ouster fast_livo2_core::fast_livo2_core)
add_test(NAME dynanav_ouster_native COMMAND test_dynanav_ouster)
set_tests_properties(dynanav_ouster_native PROPERTIES TIMEOUT 30)

add_executable(test_dynanav_sync_observation test_sync_observation.cpp)
target_include_directories(test_dynanav_sync_observation PRIVATE "${native_prefix}/include")
target_compile_definitions(test_dynanav_sync_observation PRIVATE X86_ARCH ROOT_DIR="${CMAKE_CURRENT_BINARY_DIR}/")
ament_target_dependencies(test_dynanav_sync_observation rclcpp fast_livo2_core image_transport)
target_link_libraries(test_dynanav_sync_observation fast_livo2_core::fast_livo2_core)
add_test(NAME dynanav_sync_observation_native COMMAND test_dynanav_sync_observation)
set_tests_properties(dynanav_sync_observation_native PROPERTIES TIMEOUT 30)

add_executable(test_dynanav_same_pair test_same_pair_handoff.cpp)
target_include_directories(test_dynanav_same_pair PRIVATE "${native_prefix}/include")
target_compile_definitions(test_dynanav_same_pair PRIVATE X86_ARCH ROOT_DIR="${CMAKE_CURRENT_BINARY_DIR}/")
ament_target_dependencies(test_dynanav_same_pair rclcpp fast_livo2_core image_transport)
target_link_libraries(test_dynanav_same_pair fast_livo2_core::fast_livo2_core)
add_test(NAME dynanav_same_pair_native COMMAND test_dynanav_same_pair)
set_tests_properties(dynanav_same_pair_native PROPERTIES TIMEOUT 60)

add_executable(test_dynanav_complete_scan test_complete_scan.cpp)
target_include_directories(test_dynanav_complete_scan PRIVATE "${native_prefix}/include")
target_compile_definitions(test_dynanav_complete_scan PRIVATE X86_ARCH ROOT_DIR="${CMAKE_CURRENT_BINARY_DIR}/")
ament_target_dependencies(test_dynanav_complete_scan rclcpp fast_livo2_core image_transport)
target_link_libraries(test_dynanav_complete_scan fast_livo2_core::fast_livo2_core)
add_test(NAME dynanav_complete_scan_native COMMAND test_dynanav_complete_scan)
set_tests_properties(dynanav_complete_scan_native PROPERTIES TIMEOUT 60)

# Experimental serial branches against the real native VIO implementation.
# Synthetic geometry only; no live estimator feedback or GPU requirement.
add_executable(test_multicamera_shadow test_multicamera_shadow.cpp)
target_include_directories(test_multicamera_shadow PRIVATE "${native_prefix}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/embodied_navigation/localization_mapping/frontend/native")
target_compile_definitions(test_multicamera_shadow PRIVATE X86_ARCH ROOT_DIR="${CMAKE_CURRENT_BINARY_DIR}/")
ament_target_dependencies(test_multicamera_shadow rclcpp fast_livo2_core image_transport)
target_link_libraries(test_multicamera_shadow fast_livo2_core::fast_livo2_core)
add_test(NAME dynanav_multicamera_shadow_native COMMAND test_multicamera_shadow)
set_tests_properties(dynanav_multicamera_shadow_native PROPERTIES TIMEOUT 30)

add_executable(test_visual_snapshot test_visual_snapshot.cpp
    ../../../third_party/fast_livo2/fast_livo2_core/src/visual_snapshot.cpp)
target_include_directories(test_visual_snapshot PRIVATE "${native_prefix}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/fast_livo2/fast_livo2_core/include/fast_livo2_core/core"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/embodied_navigation/localization_mapping/frontend/native")
target_compile_definitions(test_visual_snapshot PRIVATE X86_ARCH ROOT_DIR="${CMAKE_CURRENT_BINARY_DIR}/")
ament_target_dependencies(test_visual_snapshot rclcpp fast_livo2_core image_transport)
target_link_libraries(test_visual_snapshot fast_livo2_core::fast_livo2_core)
add_test(NAME dynanav_visual_snapshot_native COMMAND test_visual_snapshot)
set_tests_properties(dynanav_visual_snapshot_native PROPERTIES TIMEOUT 30)

# Read-only real-data shadow replay; no ROS publisher or estimator feedback.
add_executable(replay_multicamera_shadow replay_multicamera_shadow.cpp
    ../../../third_party/fast_livo2/fast_livo2_core/src/visual_snapshot.cpp)
target_include_directories(replay_multicamera_shadow PRIVATE "${native_prefix}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/fast_livo2/fast_livo2_core/include/fast_livo2_core/core"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/embodied_navigation/localization_mapping/frontend/native")
target_compile_definitions(replay_multicamera_shadow PRIVATE X86_ARCH ROOT_DIR="${CMAKE_CURRENT_BINARY_DIR}/")
ament_target_dependencies(replay_multicamera_shadow rclcpp fast_livo2_core image_transport)
target_link_libraries(replay_multicamera_shadow fast_livo2_core::fast_livo2_core)

# Common-prior four-camera update: nuisance marginalization, correlated copies,
# SO(3) covariance chart transport, and invalid-factor rejection.
find_package(GTest REQUIRED)
add_executable(test_multicamera_joint test_multicamera_joint.cpp)
target_include_directories(test_multicamera_joint PRIVATE "${native_prefix}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/fast_livo2/fast_livo2_core/include/fast_livo2_core/core")
target_compile_definitions(test_multicamera_joint PRIVATE X86_ARCH ROOT_DIR="${CMAKE_CURRENT_BINARY_DIR}/")
ament_target_dependencies(test_multicamera_joint rclcpp fast_livo2_core image_transport)
target_link_libraries(test_multicamera_joint GTest::gtest_main fast_livo2_core::fast_livo2_core)
add_test(NAME dynanav_multicamera_joint_native COMMAND test_multicamera_joint)
set_tests_properties(dynanav_multicamera_joint_native PROPERTIES TIMEOUT 30)
