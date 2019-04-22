#include <glm/glm.hpp>

#include <vector>
#include <fstream>
#include <sstream>

#include <nanoflann.hpp>

#include <Eigen/Core>
#include <Eigen/EigenValues>

struct Vertex
{
	glm::vec3 position;
	glm::vec2 texture_coord;
	glm::vec3 normal;
};

struct VertRef
{
	VertRef(float v, float vt, float vn) : v(v), vt(vt), vn(vn) { }
	float v, vt, vn;
};

std::vector<Vertex> loadOBJ(std::istream&);

