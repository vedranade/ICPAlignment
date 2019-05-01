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

int btn;
glm::ivec2 startMouse;
glm::ivec2 startRot, curRot;
glm::ivec2 startTrans, curTrans;
GLfloat zoom = 0.0f;

std::vector<Vertex> first_model;
std::vector<Vertex> second_model;

std::vector<Vertex> first_modelVec;
std::vector<Vertex> second_modelVec;

void displayOBJ(int r, int g, int b, float x, float y, float z, std::vector<Vertex> model)
{
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glTranslatef(x, y, z);
	glPushMatrix();
	{
		//Handles mouse rotation:
		glRotatef(curRot.x % 360, 0, 1, 0);
		glRotatef(-curRot.y % 360, 1, 0, 0);

		// object
		glColor3ub(r, g, b);
		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_FLOAT, sizeof(Vertex), &model[0].position);
		glDrawArrays(GL_POINTS, 0, model.size());
		glDisableClientState(GL_VERTEX_ARRAY);
	}
	glPopMatrix();
}

void displayInputOBJ(int r, int g, int b, float x, float y, float z, std::vector<Vertex> model)
{
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glTranslatef(x, y, z);
	glPushMatrix();
	{
		//Handles mouse rotation:
		glRotatef(curRot.x % 360, 0, 1, 0);
		glRotatef(-curRot.y % 360, 1, 0, 0);

		// object
		glColor3ub(r, g, b);
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);
		glVertexPointer(3, GL_FLOAT, sizeof(Vertex), &model[0].position);
		glTexCoordPointer(2, GL_FLOAT, sizeof(Vertex), &model[0].texture_coord);
		glNormalPointer(GL_FLOAT, sizeof(Vertex), &model[0].normal);
		glDrawArrays(GL_TRIANGLES, 0, model.size());
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);
	}
	glPopMatrix();
}

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

void display_input()
{
	glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	double w = glutGet(GLUT_WINDOW_WIDTH);
	double h = glutGet(GLUT_WINDOW_HEIGHT);
	double ar = w / h;
	glTranslatef(curTrans.x / w * 2, curTrans.y / h * 2, zoom);
	gluPerspective(60, ar, 0.1, 100);

	/*glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();*/
	//First input object:
	displayInputOBJ(255, 0, 0, 0, 0, -20, first_model);

	//Second input object:
	displayInputOBJ(0, 255, 0, 0, 0, -20, second_model);

	glutSwapBuffers();
}

void display_output()
{
	glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	double w = glutGet(GLUT_WINDOW_WIDTH);
	double h = glutGet(GLUT_WINDOW_HEIGHT);
	double ar = w / h;
	glTranslatef(curTrans.x / w * 2, curTrans.y / h * 2, zoom);
	gluPerspective(60, ar, 0.1, 100);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//First input object:
	displayOBJ(255, 0, 0, 0, 0, -20, first_modelVec);

	//Second input object:
	displayOBJ(0, 255, 0, 0, 0, -20, second_modelVec);

	glutSwapBuffers();
}

//void mouse_wheel(int button, int dir, int x, int y)
//{
//	if(dir > 0)
//		zoom += 1.0f;
//	if(dir < 0)
//		zoom -= 1.0f;
//}

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

int main(int argc, char **argv)
{
	std::ifstream inputfile1("lamp.obj");
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