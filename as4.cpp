
#include "includes.h"
#include "Bone.h"
#include "Kinematics.h"
#include "Cylinder.h"

#define PI 3.14159265

//****************************************************
// Some Classes
//****************************************************

class Viewport;

class Viewport {
public:
	int w, h; // width and height
};


//****************************************************
// Global Variables
//****************************************************
Viewport viewport;
int currColor=0;
bool shading=0, wireframe=1;
GLuint object;
GLfloat lights[5][4]= {{0.9, 0.9, 0.9, 0.8}, {0.4, 0.0, 0.8, 0.8}, {0.0, 0.6, 0.6, 0.8}, {0.5, 0.5, 0.1, 0.8}, {0.0, 0.0, 0.4, 0.8}};
std::vector<Bone> bones(7);
Kinematics test(0.1,0.01);
typedef Eigen::Vector3d (* ParametricFunctions) (float t);

Eigen::Vector3d heart(float t){return Eigen::Vector3d((16*pow(sin(t),3))/3, (13*cos(t)-5*cos(2*t)-2*cos(3*t)-cos(4*t))/3, -3);}

Eigen::Vector3d spiral(float t){return Eigen::Vector3d(2*cos(2*PI*t), 2*sin(2*PI*t), t);}

Eigen::Vector3d figureEight(float t){return Eigen::Vector3d(2*cos(t), 2*sin(2*t), 1);}

Eigen::Vector3d butterfly(float t){return Eigen::Vector3d(sin(t)*(exp(cos(t))-2*cos(4*t)-pow(sin(t/12),5)),
														  cos(t)*(exp(cos(t))-2*cos(4*t)-pow(sin(t/12),5)), 1);}


ParametricFunctions coolShapes[] = { spiral, heart, figureEight, butterfly
					};
int shape = 2; 

Eigen::Vector3d goal=coolShapes[shape](0);
//****************************************************
// reshape viewport if the window is resized
//****************************************************
void myReshape(int w, int h) {
	viewport.w = w;
	viewport.h = h;
	glViewport (0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(30, (GLfloat) w/(GLfloat) h, 1.0, 100.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(10, 10, 10, 0, 0, 0, 0, 0, 1);
}






//****************************************************
// Simple init function
//****************************************************
void initScene(){
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
	glEnable(GL_DEPTH_TEST);
    //glEnable(GL_LIGHTING);

    glLightfv(GL_LIGHT0, GL_AMBIENT, lights[0]);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lights[0]);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lights[0]);
    GLfloat pos[] = {2000,2000,2000};
    glLightfv(GL_LIGHT0, GL_POSITION, pos);
    glEnable(GL_LIGHT0);
    glShadeModel(GL_FLAT);

	object = glGenLists(1);


    std::cout<<test.solveFK(bones,0, 0.001, 0)<<std::endl<<std::endl;

    for (int i=0; i<bones.size();i++){
        std::cout << bones[i].currTheta << ", " << bones[i].currPhi << std::endl;
    }

     std::cout<<test.jacobian(bones, 0.1)<<std::endl;
    

	glNewList(object, GL_COMPILE);

	glLineWidth(1);
	glBegin(GL_LINES);
    
    glColor3f(1, 0, 0);
	glVertex3f(0,0,0);
	glVertex3f(100,0,0);

	glColor3f(0, 1, 0);
	glVertex3f(0,0,0);
	glVertex3f(0,100,0);

	glColor3f(0, 0, 1);
	glVertex3f(0,0,0);
	glVertex3f(0,0,100);
    
    glEnd();

    glBegin(GL_LINE_STRIP);
    float t = 0.0f;
    while (t<16) {
    	Eigen::Vector3d temp = coolShapes[shape](t);
    	glVertex3f(temp[0], temp[1], temp[2]);
        t += 0.01f;
    }
    glEnd();
    
    glEndList();

	glClearColor(0.0, 0.0, 0.0, 0.0);
}

void interpolateGoal(bool dir, Eigen::Vector3d & goal) {
    static float t = 0.0f;
    t += dir? 0.01f : -0.01f;
	goal = coolShapes[shape](t);
}

void renderIK() {
    //Eigen::Vector3d goal = interpolateGoal();
    test.solveIK(bones, goal);
    glColor3f(0, 1, 1);

    renderCylinder_convenient(0, 0, 0, bones[0].currPos[0], bones[0].currPos[1], bones[0].currPos[2], 0.04, 10);
    glPushMatrix();
    glTranslatef(bones[0].currPos[0], bones[0].currPos[1], bones[0].currPos[2]);
    glutSolidSphere(0.1, 10, 10);
    glPopMatrix();

	for (int i=1; i<bones.size(); i++) {
        if (i%2==0) {
            glColor3f(0, 1, 1);
        } else {
            glColor3f(1, 0, 1);
        }
        renderCylinder_convenient(bones[i-1].currPos[0], bones[i-1].currPos[1], bones[i-1].currPos[2], bones[i].currPos[0], bones[i].currPos[1], bones[i].currPos[2], 0.04, 5);
        
        glPushMatrix();
        glTranslatef(bones[i].currPos[0], bones[i].currPos[1], bones[i].currPos[2]);
        glutSolidSphere(0.1, 10, 10);
        glPopMatrix();
	}
    glBegin(GL_LINES);
    glColor3f(1, 1, 0);
    glVertex3f(0,0,0);
    glVertex3f(goal[0],goal[1],goal[2]);
    glEnd();
}

//****************************************************
// function that does the actual drawing of stuff
//***************************************************
void myDisplay() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glCallList(object);
    renderIK();
	glFlush();
	glutSwapBuffers();  
}

void keyboard(unsigned char key, int x, int y)
{
	switch (key) {
	case 'n':
		shape = (shape+1) % 4;
		initScene();
		glutPostRedisplay();
		break;
    case 'u':
        interpolateGoal(1, goal);
        glutPostRedisplay();
        break;
    case 'i':
        interpolateGoal(0, goal);
        glutPostRedisplay();
        break;
	case 's':
		if (shading) { 
			glShadeModel(GL_FLAT);
		} else {
			glShadeModel(GL_SMOOTH);
		}
		glutPostRedisplay();
		shading = !shading;
		break;
	case 'w':
		if (wireframe) { 
			glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
		} else {
			glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
		}
		glutPostRedisplay();
		wireframe = !wireframe;
		break;
	case '+':
		glScalef(1.1,1.1,1.1);
		glutPostRedisplay();
		break;
	case '-':
		glScalef(1/1.1,1/1.1,1/1.1);
		glutPostRedisplay();
		break;
	case 'c':
		currColor++;
		glLightfv(GL_LIGHT0, GL_AMBIENT, lights[currColor%5]);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, lights[currColor%5]);
		glLightfv(GL_LIGHT0, GL_SPECULAR, lights[currColor%5]);
		glutPostRedisplay();
		break;
	case 27:
		exit(0);
		break;
	}
}

void arrows(int key, int x, int y)
{
	switch (key) {
	case GLUT_KEY_UP:
		glRotatef(15.,1.0,0.0,0.0);
		glutPostRedisplay();
		break;
	case GLUT_KEY_DOWN:
		glRotatef(-15.,1.0,0.0,0.0);
		glutPostRedisplay();
		break;
	case GLUT_KEY_LEFT:
		glRotatef(15.,0.0,1.0,0.0);
		glutPostRedisplay();
		break;
	case GLUT_KEY_RIGHT:
		glRotatef(-15.,0.0,1.0,0.0);
		glutPostRedisplay();
		break;
	}
}

//****************************************************
// the usual stuff, nothing exciting here
//****************************************************
int main(int argc, char *argv[]) {
	bones[0]=Bone(2.0f);
    bones[1]=Bone(1.3f);
    bones[2]=Bone(0.5f);
    bones[3]=Bone(1.0f);
    bones[4]=Bone(1.0f);
    bones[5]=Bone(0.4f);
    bones[6]=Bone(0.8f);
	//This initializes glut
	glutInit(&argc, argv);

	//This tells glut to use a double-buffered window with red, green, and blue channels 
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);

	// Initalize theviewport size
	viewport.w = 640;
	viewport.h = 480;

	//The size and position of the window
	glutInitWindowSize(viewport.w, viewport.h);
	glutInitWindowPosition(0,0);
	glutCreateWindow(argv[0]);

	initScene();							// quick function to set up scene

	glutDisplayFunc(myDisplay);				// function to run when its time to draw something
	glutReshapeFunc(myReshape);				// function to run when the window gets resized
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(arrows);
	glutMainLoop();							// infinite loop that will keep drawing and resizing

	return 0;
}
