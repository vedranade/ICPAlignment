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

#include "functionality.h"

//int btn;
glm::ivec2 startMouse;
glm::ivec2 startRot, curRot;
glm::ivec2 startTrans, curTrans;
//GLfloat zoom = 0.0f;

std::vector<Vertex> first_model;
std::vector<Vertex> second_model;

std::vector<Vertex> first_modelVec;
std::vector<Vertex> second_modelVec;

std::vector<Vertex> convertToVec(Eigen::MatrixXd input)
{
	//Initializes and populates the vector:
	std::vector<Vertex> Vec;
	Vertex VecObject;
	
	for (int i = 0; i < input.rows(); i++)
	{
		/*Vec[i].position.x = input(i, 0);
		Vec[i].position.y = input(i, 1);
		Vec[i].position.z = input(i, 2);*/
		VecObject.position = glm::vec3(input(i, 0), input(i, 1), input(i, 2));
		VecObject.normal = glm::vec3(0, 0, 0);
		VecObject.texture_coord = glm::vec3(0, 0, 0);
		Vec.push_back(VecObject);
	}
	return Vec;
}

Eigen::MatrixXd convertToMat(std::vector<Vertex> input)
{
	Eigen::MatrixXd Mat(input.size(), 3);
	for (int i = 0; i < input.size(); i++)
	{
		Mat(i, 0) = input[i].position.x;
		Mat(i, 1) = input[i].position.y;
		Mat(i, 2) = input[i].position.z;
	}

	return Mat;
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


	Eigen::MatrixXd first_modelMat = convertToMat(first_model);
	Eigen::MatrixXd second_modelMat = convertToMat(second_model);

	Aligner solver = Aligner(first_modelMat, second_modelMat);
	solver.calculateAlignment();

	first_modelVec = convertToVec(solver.firstModel_verts);
	second_modelVec = convertToVec(solver.secondModel_verts);
	

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_SINGLE);
	glutInitWindowSize(1280, 720);

	//Display input:
	glutCreateWindow("Input Objects");
	glutDisplayFunc(display_input);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	//glutMouseWheelFunc(mouse_wheel);

	glEnable(GL_DEPTH_TEST);

	//Display output:
	glutCreateWindow("Output Objects");
	glutDisplayFunc(display_output);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	//glutMouseWheelFunc(mouse_wheel);

	glEnable(GL_DEPTH_TEST);
	glutMainLoop();

	return 0;
}