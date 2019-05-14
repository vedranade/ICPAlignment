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

void display()
{
	glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	double w = glutGet(GLUT_WINDOW_WIDTH);
	double h = glutGet(GLUT_WINDOW_HEIGHT);
	double ar = w / h;
	glTranslatef(curTrans.x / w * 2, curTrans.y / h * 2, 0.0);
	gluPerspective(60, ar, 0.01, 1000);
	gluLookAt
	(
		0.0, 0.0, zoom,
		0.0, 0.0, 0.0,
		0.0, 1.0, 0.0
	);
	displayOBJ(0, 255, 255, 0, 0, -20, first_model);
	
	displayOBJ(0, 255, 0, 0, 0, -20, second_model);

	if (!initialized)
	{
		initialized = true;
		Eigen::MatrixXd first_modelMat = convertToMat(first_model);
		Eigen::MatrixXd second_modelMat = convertToMat(second_model);
		solver.initialize(first_modelMat, second_modelMat);
	}

	glutSwapBuffers();
}

void motion(int x, int y)
{
	glm::ivec2 curMouse(x, glutGet(GLUT_WINDOW_HEIGHT) - y);
	if (btn == GLUT_RIGHT_BUTTON)
		curRot = startRot + (curMouse - startMouse);
	else if (btn == GLUT_LEFT_BUTTON)
		curTrans = startTrans + (curMouse - startMouse);
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
	case 'w':
		zoom -= 1.0f;
		glutPostRedisplay();
		break;
	case 'W':
		zoom -= 1.0f;
		glutPostRedisplay();
		break;
	case 's':
		zoom += 1.0f;
		glutPostRedisplay();
		break;
	case 'S':
		zoom += 1.0f;
		glutPostRedisplay();
		break;
	}
}

int main(int argc, char **argv)
{
	std::ifstream inputfile1("magnolia.obj");
	first_model = loadOBJ(inputfile1);

	std::ifstream inputfile2("magnolia.obj");
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