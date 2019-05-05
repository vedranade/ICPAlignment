#include <GL/glew.h>
#include <GL/freeglut.h>

#include <glm/glm.hpp>
#include <glm/gtx/component_wise.hpp>

#include <Eigen/Core>
#include <Eigen/EigenValues>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <numeric>
#include <sys/utime.h>

#include "functionality.h"

using namespace std;

glm::ivec2 startMouse;
glm::ivec2 startRot, curRot;
glm::ivec2 startTrans, curTrans;

std::vector<Vertex> first_model;
std::vector<Vertex> second_model;

std::vector<Vertex> first_modelVec;
std::vector<Vertex> second_modelVec;

Aligner solver;

static bool initialized;

float my_random(void) 
{
	float tmp = rand() % 100;
	return tmp / 1000;
}

void my_random_shuffle(Eigen::MatrixXd &matrix) 
{
	int row = matrix.rows();
	std::vector<Eigen::Vector3d> temp;
	for (int jj = 0; jj < row; jj++) {
		temp.push_back(matrix.block<1, 3>(jj, 0));
	}
	random_shuffle(temp.begin(), temp.end());
	for (int jj = 0; jj < row; jj++) {
		matrix.block<1, 3>(jj, 0) = temp[jj].transpose();
		// cout << temp[jj].transpose() << endl;
		// cout << "row  " << row << endl;
	}
}

Eigen::Matrix3d rotation_matrix(Eigen::Vector3d axis, float theta) 
{
	axis = axis / sqrt(axis.transpose()*axis);
	float a = cos(theta / 2);
	Eigen::Vector3d temp = -axis * sin(theta / 2);
	float b, c, d;
	b = temp(0);
	c = temp(1);
	d = temp(2);
	Eigen::Matrix3d R;
	R << a * a + b * b - c * c - d * d, 2 * (b*c - a * d), 2 * (b*d + a * c),
		2 * (b*c + a * d), a*a + c * c - b * b - d * d, 2 * (c*d - a * b),
		2 * (b*d - a * c), 2 * (c*d + a * b), a*a + d * d - b * b - c * c;

	return R;
}

void test_icp(Eigen::MatrixXd A, Eigen::MatrixXd B)
{
	Eigen::MatrixXd C;
	Eigen::Vector3d t;
	Eigen::Matrix3d R;
	Eigen::Matrix4d T;
	Eigen::Vector3d t1;
	Eigen::Matrix3d R1;
	ICP_OUT icp_result;
	std::vector<float> dist;
	int iter;
	float mean;

	float total_time = 0;
	unsigned start, end;
	float interval;

	for (int i = 0; i < N_tests; i++) 
	{
		//B = A;
		//t = Eigen::Vector3d::Random()*translation2;

		//for (int jj = 0; jj < B.size(); jj++) 
		//{
		//	std::cout << jj << std::endl;
		//	B.block<1, 3>(jj, 0) = B.block<1, 3>(jj, 0) + t.transpose();
		//}

		//R = rotation_matrix(Eigen::Vector3d::Random(), my_random()*rotation2);
		//B = (R * B.transpose()).transpose();

		//B += Eigen::MatrixXd::Random(B.size(), 3) * noise_sigma;

		//// shuffle
		//my_random_shuffle(B);

		start = GetTickCount();
		icp_result = icp(B, A, 20, 0.000001);
		end = GetTickCount();
		interval = float((end - start)) / 1000;
		// cout << "interval" << interval << endl;
		total_time += interval;

		first_model = convertToVec(icp_result.trans);

		T = icp_result.trans;
		dist = icp_result.distances;
		iter = icp_result.iter;
		mean = std::accumulate(dist.begin(), dist.end(), 0.0) / dist.size();

		/*C = Eigen::MatrixXd::Ones(B.size(), 4);
		int size = B.size();
		C.block<size, 3>(0, 0) = B;

		C = (T * C.transpose()).transpose();
		t1 = T.block<3, 1>(0, 3);
		R1 = T.block<3, 3>(0, 0);


		if (i == 3) 
		{
			std::cout << "mean error is " << mean - 6 * noise_sigma << endl << endl;
			std::cout << "icp trans error" << endl << -t1 - t << endl << endl;
			std::cout << "icp R error " << endl << R1.inverse() - R << endl << endl;
		}*/

	}
	cout << "icp time: " << total_time / N_tests << endl;
	glutPostRedisplay();
}

void display()
{
	std::cout << "Display called\n";
	glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	double w = glutGet(GLUT_WINDOW_WIDTH);
	double h = glutGet(GLUT_WINDOW_HEIGHT);
	double ar = w / h;
	glTranslatef(curTrans.x / w * 2, curTrans.y / h * 2, zoom);
	gluPerspective(60, ar, 0.1, 100);

	displayOBJ(255, 0, 0, 0, 0, -20, first_model);
	
	displayOBJ(0, 255, 0, 0, 0, -20, second_model);
	glutSwapBuffers();
	//std::cin.get();
	if (!initialized)
	{
		initialized = true;
		Eigen::MatrixXd first_modelMat = convertToMat(first_model);
		Eigen::MatrixXd second_modelMat = convertToMat(second_model);
		solver.initialize(first_modelMat, second_modelMat);
		//test_icp(first_modelMat, second_modelMat);
		
	}

	glutSwapBuffers();
}

void motion(int x, int y)
{
	glm::ivec2 curMouse(x, glutGet(GLUT_WINDOW_HEIGHT) - y);
	if (btn == GLUT_RIGHT_BUTTON)
	{
		curRot = startRot + (curMouse - startMouse);
	}
	else if (btn == GLUT_LEFT_BUTTON)
	{
		curTrans = startTrans + (curMouse - startMouse);
	}
	glutPostRedisplay();
}

void mouse(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
	{
		btn = button;
		startMouse = glm::ivec2(x, glutGet(GLUT_WINDOW_HEIGHT) - y);
		startRot = curRot;
	}
	if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
	{
		btn = button;
		startMouse = glm::ivec2(x, glutGet(GLUT_WINDOW_HEIGHT) - y);
		startTrans = curTrans;
	}
}

void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'f':
		solver.step();
		glutPostRedisplay();
		break;
	case 'F':
		solver.step();
		glutPostRedisplay();
		break;
	}
}

int main(int argc, char **argv)
{
	std::ifstream inputfile1("shuttle.obj");
	first_model = loadOBJ(inputfile1);

	std::ifstream inputfile2("shuttle.obj");
	second_model = loadOBJ(inputfile2);

	//Shifting second model on x-axis a little:
	for (int i = 0; i < second_model.size(); i++)
		second_model[i].position.x += 10.0f;

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_SINGLE);
	glutInitWindowSize(1280, 720);
	glutCreateWindow("Input Objects");
	glutDisplayFunc(display);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutKeyboardFunc(keyboard);

	glEnable(GL_DEPTH_TEST);

	// set up "headlamp"-like light
	glShadeModel(GL_SMOOTH);
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	GLfloat position[] = { 0, 0, 1, 0 };
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glPolygonMode(GL_FRONT, GL_FILL);
	glPolygonMode(GL_BACK, GL_LINE);

	glutMainLoop();

	return 0;
}