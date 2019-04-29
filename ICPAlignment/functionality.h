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

class ICP_Solver {
	public:
	Eigen::MatrixXd data_verts; size_t N_data;
	Eigen::MatrixXd model_verts;
	std::map<int, int> point_correspondence;
	std::map<int, double> weights;

	Eigen::Vector3d translation, final_translation = Eigen::Vector3d::Zero();
	Eigen::Matrix3d rotation, final_rotation = Eigen::Matrix3d::Identity();
	bool iteration_has_converged = false;

	private:
	kd_tree_t *model_kd_tree;

	double error = FLT_MAX;
	double old_error = 0;
	int iter_counter = 0;
	const size_t max_it = 5;
	const double tolerance = 0.000001;
	const float sampling_quotient = 1.0;

	public:
	ICP_Solver(Eigen::MatrixXd data_verts, Eigen::MatrixXd model_verts);

	/*
	 * The main entry point for ICP alignment of the loaded meshes.
	 * Iterates until convergence or maximum number of iterations is reached.
	 */

	bool check_iter();

	bool perform_icp();

	private:
	void compute_closest_points();

	void compute_registration(Eigen::Vector3d &translation,
		Eigen::Matrix3d &rotation);

	double compute_rms_error(Eigen::Vector3d translation,
		Eigen::Matrix3d rotation);

	void quaternion_to_matrix(Eigen::Vector4d q, Eigen::Matrix3d &R);

};

std::vector<Vertex> loadOBJ(std::istream&);

