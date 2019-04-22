#include <GL/glew.h>
#include <GL/freeglut.h>

#include <glm/glm.hpp>
#include <glm/gtx/component_wise.hpp>

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

template< typename Vec >
std::pair< Vec, Vec > GetExtents(const Vec* pts, size_t stride, size_t count)
{
	unsigned char* base = (unsigned char*)pts;
	Vec pmin(*(Vec*)base);
	Vec pmax(*(Vec*)base);
	for (size_t i = 0; i < count; ++i, base += stride)
	{
		const Vec& pt = *(Vec*)base;
		pmin = glm::min(pmin, pt);
		pmax = glm::max(pmax, pt);
		
	}

	return std::make_pair(pmin, pmax);
}

template<typename Vec>
void CenterAndScale(Vec *pts, size_t stride, size_t count, const typename Vec::value_type & size)
{
	typedef typename Vec::value_type Scalar;

	// get min/max extents
	std::pair< Vec, Vec > exts = GetExtents(pts, stride, count);

	// center and scale 
	const Vec center = (exts.first * Scalar(0.5)) + (exts.second * Scalar(0.5f));

	const Scalar factor = size / glm::compMax(exts.second - exts.first);
	unsigned char* base = (unsigned char*)pts;
	for (size_t i = 0; i < count; ++i, base += stride)
	{
		Vec& pt = *(Vec*)base;
		pt = ((pt - center) * factor);
	}
	
}

void display_first()
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
	glTranslatef(0, 0, -10);

	glPushMatrix();
	{
		glRotatef(curRot.x % 360, 0, 1, 0);
		glRotatef(-curRot.y % 360, 1, 0, 0);

		// object
		glColor3ub(255, 0, 0);
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);
		glVertexPointer(3, GL_FLOAT, sizeof(Vertex), &first_model[0].position);
		glTexCoordPointer(2, GL_FLOAT, sizeof(Vertex), &first_model[0].texture_coord);
		glNormalPointer(GL_FLOAT, sizeof(Vertex), &first_model[0].normal);
		glDrawArrays(GL_TRIANGLES, 0, first_model.size());
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);

		// bounding cube
		glDisable(GL_LIGHTING);
		/*glColor3ub(255, 255, 255);
		glutWireCube(7);*/
		glEnable(GL_LIGHTING);
	}
	glPopMatrix();

	glutSwapBuffers();
}

void display_second()
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
	glTranslatef(0, 0, -10);

	glPushMatrix();
	{
		glRotatef(curRot.x % 360, 0, 1, 0);
		glRotatef(-curRot.y % 360, 1, 0, 0);

		// object
		glColor3ub(255, 0, 0);
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);
		glVertexPointer(3, GL_FLOAT, sizeof(Vertex), &second_model[0].position);
		glTexCoordPointer(2, GL_FLOAT, sizeof(Vertex), &second_model[0].texture_coord);
		glNormalPointer(GL_FLOAT, sizeof(Vertex), &second_model[0].normal);
		glDrawArrays(GL_TRIANGLES, 0, second_model.size());
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);

		// bounding cube
		glDisable(GL_LIGHTING);
		/*glColor3ub(255, 255, 255);
		glutWireCube(7);*/
		glEnable(GL_LIGHTING);
	}
	glPopMatrix();

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
	CenterAndScale(&first_model[0].position, sizeof(Vertex), first_model.size(), 7);

	std::ifstream inputfile2("shuttle.obj");
	second_model = loadOBJ(inputfile2);
	CenterAndScale(&second_model[0].position, sizeof(Vertex), second_model.size(), 7);
	
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_SINGLE);
	glutInitWindowSize(1280, 720);

	glutCreateWindow("First Object");
	glutDisplayFunc(display_first);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	//glutMouseWheelFunc(mouse_wheel);

	glEnable(GL_DEPTH_TEST);

	// set up simple lighting:
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

	glutCreateWindow("Second Object");
	glutDisplayFunc(display_second);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	//glutMouseWheelFunc(mouse_wheel);

	glEnable(GL_DEPTH_TEST);

	// set up simple lighting:
	glShadeModel(GL_SMOOTH);
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//GLfloat position[] = { 0, 0, 1, 0 };
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glPolygonMode(GL_FRONT, GL_FILL);
	glPolygonMode(GL_BACK, GL_LINE);

	glutMainLoop();

	return 0;
}