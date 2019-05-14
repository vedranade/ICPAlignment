#include <glm/glm.hpp>

#include <vector>
#include <map>
#include <fstream>
#include <sstream>

#include <nanoflann.hpp>

#include <Eigen/Core>
#include <Eigen/EigenValues>

using namespace Eigen;

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

class Aligner
{
	public:
	MatrixXd firstModel_verts; size_t N_data;
	MatrixXd secondModel_verts;
	std::map<int, int> point_correspondence;

	Vector3d translation, final_translation = Vector3d::Zero();
	Matrix3d rotation, final_rotation = Matrix3d::Identity();
	bool iteration_has_converged = false;


	kd_tree_t *model_kd_tree;

	double error = FLT_MAX;
	double old_error = 0;
	int iter_counter = 0;
	const size_t max_it = 500;
	const double threshold = 0.01;

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

std::vector<Vertex> convertToVec(Eigen::MatrixXd input, std::vector<Vertex> Verts);
Eigen::MatrixXd convertToMat(std::vector<Vertex> input);

void displayOBJ(int r, int g, int b, float x, float y, float z, std::vector<Vertex> model);
void display();
void motion(int x, int y);
void mouse(int button, int state, int x, int y);
void keyboard(unsigned char key, int x, int y);

std::vector<Vertex> loadOBJ(std::istream&);
