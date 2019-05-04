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

static bool initialized;

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
		Aligner solver = Aligner(first_modelMat, second_modelMat);
		solver.calculateAlignment();
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
	glutMainLoop();

	return 0;
}