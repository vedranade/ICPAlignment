#include <glm/glm.hpp>

#include <vector>
#include <map>
#include <fstream>
#include <sstream>

#include <nanoflann.hpp>

#include <Eigen/Core>
#include <Eigen/EigenValues>

extern int btn;
extern glm::ivec2 startMouse;
extern glm::ivec2 startRot, curRot;
extern glm::ivec2 startTrans, curTrans;
extern GLfloat zoom;

struct Vertex
{
	glm::vec3 position;
	glm::vec2 texture_coord;
	glm::vec3 normal;
};

extern std::vector<Vertex> first_model;
extern std::vector<Vertex> second_model;

extern std::vector<Vertex> first_modelVec;
extern std::vector<Vertex> second_modelVec;

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
	Eigen::MatrixXd firstModel_verts, firstModel_verts_copy; size_t N_data, N_data_copy;
	Eigen::MatrixXd secondModel_verts, secondModel_verts_copy;
	std::map<int, int> point_correspondence;
	std::map<int, double> weights;
	const float sampling_quotient = 1.0;

	Eigen::Vector3d translation, final_translation = Eigen::Vector3d::Zero();
	Eigen::Matrix3d rotation, final_rotation = Eigen::Matrix3d::Identity();
	bool iteration_has_converged = false;


	kd_tree_t *model_kd_tree;

	double error = FLT_MAX;
	double old_error = 0;
	int iter_counter = 0;
	const size_t max_it = 500;
	const double threshold = 0.0001;

	void initialize(Eigen::MatrixXd d, Eigen::MatrixXd m);

	void pointSearch();

	void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove);

	void getMinThreaded(Eigen::MatrixXd &A, Eigen::MatrixXd &B, int startA, int endA, int startB, int endB,
		std::map<int, int> &pc, std::vector<GLfloat> &d);

	void getMinDistance(Eigen::MatrixXd A, Eigen::MatrixXd B, std::vector<GLfloat> &distances);

	void calculateTransformation(Eigen::Vector3d &translation,
		Eigen::Matrix3d &rotation);

	void calculateQMatrix(Eigen::Vector4d q, Eigen::Matrix3d &R);

	bool step();

	double calculateError(Eigen::Vector3d translation, Eigen::Matrix3d rotation);

};

std::vector<Vertex> convertToVec(Eigen::MatrixXd input);
Eigen::MatrixXd convertToMat(std::vector<Vertex> input);

void displayOBJ(int r, int g, int b, float x, float y, float z, std::vector<Vertex> model);
void display();
void motion(int x, int y);
void mouse(int button, int state, int x, int y);
void keyboard(unsigned char key, int x, int y);

std::vector<Vertex> loadOBJ(std::istream&);
