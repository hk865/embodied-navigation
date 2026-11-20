cmake_minimum_required(VERSION 3.16)
project(pct_cli LANGUAGES CXX)
find_package(Eigen3 REQUIRED)
# Standalone A* CLI: pct_cli.cpp holds main(), a_star_search.cc the 3rd-party
# grid search implementation (header-only deps under native/).
add_executable(pct_astar pct_cli.cpp native/a_star/a_star_search.cc)
target_include_directories(pct_astar PRIVATE native)
target_link_libraries(pct_astar PRIVATE Eigen3::Eigen)
target_compile_features(pct_astar PRIVATE cxx_std_17)
# Installed into the prefix root instead of bin/ because the CLI is run in place
# as a build artifact by the packaging pipeline.
install(TARGETS pct_astar RUNTIME DESTINATION .)
