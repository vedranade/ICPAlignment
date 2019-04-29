#include <glm/glm.hpp>

#include <vector>
#include <map>
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

typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd, 3, nanoflann::metric_L1> kd_tree_t;
typedef nanoflann::KDTreeEigenMatrixAdaptor<std::vector<Vertex>, 3, nanoflann::metric_L1> kd_tree_t1;

class Aligner 
{
	public:
	Eigen::MatrixXd firstModel_verts; size_t N_data;
	Eigen::MatrixXd secondModel_verts;
	std::map<int, int> point_correspondence;
	std::map<int, double> weights;

	Eigen::Vector3d translation, final_translation = Eigen::Vector3d::Zero();
	Eigen::Matrix3d rotation, final_rotation = Eigen::Matrix3d::Identity();
	bool iteration_has_converged = false;

	kd_tree_t *model_kd_tree;

	double error = FLT_MAX;
	double old_error = 0;
	int iter_counter = 0;
	size_t max_it = 5;
	double threshold = 0.000001;

	Aligner(Eigen::MatrixXd firstModel_verts, Eigen::MatrixXd secondModel_verts);

	bool check_iter();

	bool perform_icp();

	void pointSearch();

	void compute_registration(Eigen::Vector3d &translation,
		Eigen::Matrix3d &rotation);

	double calculateError(Eigen::Vector3d translation,
		Eigen::Matrix3d rotation);

	void calculateQMatrix(Eigen::Vector4d q, Eigen::Matrix3d &R);

};

std::vector<Vertex> loadOBJ(std::istream&);

