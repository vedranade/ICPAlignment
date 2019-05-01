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

glm::ivec2 startMouse;
glm::ivec2 startRot, curRot;
glm::ivec2 startTrans, curTrans;

std::vector<Vertex> first_model;
std::vector<Vertex> second_model;

std::vector<Vertex> first_modelVec;
std::vector<Vertex> second_modelVec;

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