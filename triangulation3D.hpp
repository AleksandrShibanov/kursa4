#pragma once

#include <Eigen/Dense>

namespace Triangulation {
struct Triangle {
	uint32_t V0;
	uint32_t V1;
	uint32_t V2;
};

/// 3d triangulation based on extended delaunay triangulation with optional surface extraction
std::vector<Triangle> triangulate3D(const std::vector<Eigen::Vector3f>& vertices, bool surfaceOnly = true);

} // namespace Triangulation
