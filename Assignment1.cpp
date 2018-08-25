#include <math.h>
#include <GL/freeglut.h>
#include <iostream>
#include "loadTGA.h"
#include "loadBMP.h"

//--Globals-------------------------------------------------------------
GLuint textureID[11];
GLUquadric *q;    //Required for creating cylindrical objects
float size = 800; //size of sky-box
float theta = 0, toRad = 3.1415926 / 180, cam_hgt = 15, cam_speed = 5, theta_xz = 0, theta_yz = 0;//contral variable
int step = 0, rotation = 0, walk_angle = 0, walk_cycle = 2, walk_step = 1.5, walk_turn = 10;//contral variable
float ex = -80, ey = 0, ez = 110, lx = -80, ly = 15, lz = 0; //main camera position
float white[4] = {1., 1., 1., 1.}, black[4] = {0}, grey[4] = {0.6, 0.6, 0.6, 1.0}, yellow[4] = {1., 1., 0., 1.}, blue[4] = {0., 0., 1., 1.};//colours
float dir_x, dir_y, dir_z;// for main camera
int elevator_height1 = 0, elevator_height1_1 = 50, elevator_height2 = 50, elevator_height2_2 = 0;//wing-elevator, main-elevator
int theta_clk1 = 0, theta_clk2 = 0, theta_pdl = 0, swing_pdl = 2;//clock in main building
float x1 = -84, z1 = 75, z2 = 60, x2 = -40, x3 = 5, z3 = -5, x4 = 35, z4 = 65, x5 = 35, turn_theta = 180, turn_theta1 = 90, turn_theta2 = 180, turn_theta3 = 90, turn_theta4 = 0, turn_theta5 = -90;//path of visitor
bool path1 = true, path2 = false, path3 = false, path4 = false, path5 = false, path6 = false, path7 = false, path8 = false;//path of visitor
bool go_up = false, go_down = false, turn_right1 = false, turn_left1 = false, turn_right2 = false, turn_right3 = false, turn_right4 = false, turn_right5 = false, turn_right6 = false;//path of visitor
bool stay_elevator1 = true, stay_elevator2 = true, go_up1 = false, go_down1 = false;//path of elevator
float gravity = 1, fall_t = 0, fall_hgt = 145, rotation1 = 0, fly_z1 = -19, rotation2 = 90, fly_z2 = 46, fly_z3 = 46, text_x = 0;//path of ironman
bool fly_up = true, fly_straignt = false, fall = false, energy_on = true;//path of ironman
float lgt_pos1[4] = {-12.5f, 7.25f, -3.5f, 1.0f}, spot_light1[3] = {-15.5f, -7.25f, -.5f};//light1
float lgt_pos2[4] = {-12.5f, 7.25f, 3.5f, 1.0f}, spot_light2[3] = {-15.5f, -7.25f, .5f};//light2
float lgt_pos3[4] = {0.0f, 0.0f, 0.0f, 1.0f}, spot_light3[3] = {0.0f, 0.0f, 5.0f};//light3
float UFO_hgt = 9, UFO_dis = 1, rotation0 = 0, UFO_hgt1 = 5, UFO_dis1 = 1;//path of UFO
float lpos[4] = {0.f, 120.f, 0.f, 1.f};  //light's position
bool moveView = false, /*trigger of the first sight*/ shadow = false, pause = false;
float shadowMat[16] = {lpos[1], 0, 0, 0, -lpos[0], 0, -lpos[2], -1, 0, 0, lpos[1], 0, 0, 0, 0, lpos[1]};
float bx = 0, by = 20, ball_t, v0 = 10, theta_ball = 75;


//--Normal----------------------------------------------------
void normal(float x1, float y1, float z1,
            float x2, float y2, float z2,
            float x3, float y3, float z3 )
{
    float nx, ny, nz;
    nx = y1*(z2-z3)+ y2*(z3-z1)+ y3*(z1-z2);
    ny = z1*(x2-x3)+ z2*(x3-x1)+ z3*(x1-x2);
    nz = x1*(y2-y3)+ x2*(y3-y1)+ x3*(y1-y2);
    glNormal3f(nx, ny, nz);
}
//--load texture ---------------------------------------------
void loadTexture()
{
	glGenTextures(11, textureID);
	
	//load down texture
	glBindTexture(GL_TEXTURE_2D, textureID[0]);
	loadTGA("dn.tga");
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);	
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	//load back texture
	glBindTexture(GL_TEXTURE_2D, textureID[1]);
	loadTGA("ft.tga");
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	//load front texture
	glBindTexture(GL_TEXTURE_2D, textureID[2]);
	loadTGA("rt.tga");
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	//load left texture
	glBindTexture(GL_TEXTURE_2D, textureID[3]);
	loadTGA("bk.tga");
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	//load right texture
	glBindTexture(GL_TEXTURE_2D, textureID[4]);
	loadTGA("lf.tga");
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	//load up texture
	glBindTexture(GL_TEXTURE_2D, textureID[5]);
	loadTGA("up.tga");
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	
	//load iron man face texture
	glBindTexture(GL_TEXTURE_2D, textureID[6]);
	loadTGA("ironman_front_face.tga");
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	
	//load iron man body front texture
	glBindTexture(GL_TEXTURE_2D, textureID[7]);
	loadTGA("front_body.tga");
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);	
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	
	//load iron man body back-side texture
	glBindTexture(GL_TEXTURE_2D, textureID[8]);
	loadTGA("back_body.tga");
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);	
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	
	//load top of main building texture
	glBindTexture(GL_TEXTURE_2D, textureID[9]);
	loadTGA("mainup.tga");
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);	
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	
	//load UFO texture
	glBindTexture(GL_TEXTURE_2D, textureID[10]);
	loadTGA("UFO.tga");
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);	
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

}
//--Draw Skybox---------------------------------------------------------
void drawSkybox()
{
	glColor4f(0.0, 0.0, 0.1, 1);
	int skybox_height = size / 3;
	
	//draw bottom
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, textureID[0]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.f, 1.f);
	glVertex3f(size/2, 0, -size/2);
	glTexCoord2f(1.f, 1.f);
	glVertex3f(size/2, 0, size/2);
	glTexCoord2f(1.f, 0.f);
	glVertex3f(-size/2, 0, size/2);
	glTexCoord2f(0.f, 0.f);
	glVertex3f(-size/2, 0, -size/2);
	glEnd();
	glDisable(GL_TEXTURE_2D);
	
	
	//draw front
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, textureID[1]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.f, 0.f);
	glVertex3f(size/2, 0, size/2);
	glTexCoord2f(0.f, 1.f);
	glVertex3f(size/2, skybox_height, size/2);
	glTexCoord2f(1.f, 1.f);
	glVertex3f(-size/2, skybox_height, size/2);
	glTexCoord2f(1.f, 0.f);
	glVertex3f(-size/2, 0, size/2);
	glEnd();
	glDisable(GL_TEXTURE_2D);
		
	
	//draw right
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, textureID[2]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.f, 0.f);
	glVertex3f(-size/2, 0, size/2);
	glTexCoord2f(1.f, 0.f);
	glVertex3f(-size/2, 0, -size/2);
	glTexCoord2f(1.f, 1.f);
	glVertex3f(-size/2, skybox_height, -size/2);
	glTexCoord2f(0.f, 1.f);
	glVertex3f(-size/2, skybox_height, size/2);
	glEnd();
	glDisable(GL_TEXTURE_2D);
	
	
	//draw back
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, textureID[3]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.f, 0.f);
	glVertex3f(-size/2, 0, -size/2);
	glTexCoord2f(0.f, 1.f);
	glVertex3f(-size/2, skybox_height, -size/2);
	glTexCoord2f(1.f, 1.f);
	glVertex3f(size/2, skybox_height, -size/2);
	glTexCoord2f(1.f, 0.f);
	glVertex3f(size/2, 0, -size/2);
	glEnd();
	glDisable(GL_TEXTURE_2D);
	
	
	//draw left
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, textureID[4]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.f, 0.f);
	glVertex3f(size/2, 0, -size/2);
	glTexCoord2f(0.f, 1.f);
	glVertex3f(size/2, skybox_height, -size/2);
	glTexCoord2f(1.f, 1.f);
	glVertex3f(size/2, skybox_height, size/2);
	glTexCoord2f(1.f, 0.f);
	glVertex3f(size/2, 0, size/2);
	glEnd();
	glDisable(GL_TEXTURE_2D);
	

	//draw up
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, textureID[5]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.f, 0.f);
	glVertex3f(size/2, skybox_height, -size/2);
	glTexCoord2f(0.f, 1.f);
	glVertex3f(-size/2, skybox_height, -size/2);
	glTexCoord2f(1.f, 1.f);
	glVertex3f(-size/2, skybox_height, size/2);
	glTexCoord2f(1.f, 0.f);
	glVertex3f(size/2, skybox_height, size/2);
	glEnd();
	glDisable(GL_TEXTURE_2D);
}
//--Draw Floor----------------------------------------------------------
void drawFloor()
{
  glColor4f(0.82, 0.98, 0.58, 1);  //The floor is ligth yellow in colour
  glNormal3f(0.0, 1.0, 0.0);
  glMaterialfv(GL_FRONT, GL_SPECULAR, black);
  //The floor is made up of several tiny squares on a 200x200 grid. Each square has a unit size.
  glBegin(GL_QUADS);
  for(int i = -200; i < 200; i++)
  {
    for(int j = -200;  j < 200; j++)
    {
      glVertex3f(i, 1, j);
      glVertex3f(i, 1, j+1);
      glVertex3f(i+1, 1, j+1);
      glVertex3f(i+1, 1, j); 
    }
  }
  glEnd();
  glMaterialfv(GL_FRONT, GL_SPECULAR, white);
}
//--Draw Round-road-----------------------------------------------------
void Road(float radius, float height) //small changes from lab 3
{
	float angle1, angle2, ca1, sa1, ca2, sa2;
	float x1, z1, x2, z2, x3, z3, x4, z4;  //four points of a quad
    
	glBegin(GL_QUADS);
	for (int i = 0; i < 360; i += 2)    //5 deg intervals
	{
		angle1 = i * toRad;       //Computation of angles, cos, sin etc
		angle2 = (i + 5) * toRad;
		ca1 = cos(angle1); ca2 = cos(angle2);
		sa1 = sin(angle1); sa2 = sin(angle2);
		x1 = (radius - 0.5)*sa1; z1 = (radius - 0.5)*ca1;
		x2 = (radius + 0.5)*sa1; z2 = (radius + 0.5)*ca1;
		x3 = (radius + 0.5)*sa2; z3 = (radius + 0.5)*ca2;
		x4 = (radius - 0.5)*sa2; z4 = (radius - 0.5)*ca2;

		glNormal3f(0., 1., 0.);
		glVertex3f(x1, height, z1);
		glVertex3f(x2, height, z2);
		glVertex3f(x3, height, z3);
		glVertex3f(x4, height, z4);
		
		glNormal3f(-sa1, height, -ca1);
		glVertex3f(x1, height, z1);
		glVertex3f(x1, 1, z1);
		glNormal3f(-sa2, height, -ca2);
		glVertex3f(x4, 1, z4);
		glVertex3f(x4, height, z4);

		glNormal3f(sa1, height, ca1);
		glVertex3f(x2, 1, z2);
		glVertex3f(x2, height, z2);
		glNormal3f(sa2, height, ca2);
		glVertex3f(x3, height, z3);
		glVertex3f(x3, 1, z3);
	}
	glEnd();
}	
void drawCircularRoad()
{
	float radius;
	glMaterialfv(GL_FRONT, GL_SPECULAR, black);
	glColor4f(0.6, 0.6, 0.6, 1);
	for (radius = 160.00; radius < 190.00; radius++)
	{
		Road(radius, 1.5);
	}
	glMaterialfv(GL_FRONT, GL_SPECULAR, white);
	glMaterialfv(GL_FRONT, GL_SPECULAR, black);
	glColor4f(1, 1, 1, 1);
	for (radius = 174.0; radius <176; radius++)
	{
		Road(radius, 2);
	}
	glMaterialfv(GL_FRONT, GL_SPECULAR, white);
}
//--UFO's camera----------------------------------------------------
void cameraUFO()
{
	float vlx, vly, vlz, vir_x, vir_y, vir_z;
	float vex, vey, vez, vtheta;

	vlx = vex;
	vly = vey = 30;
	vlz = vez - 10;
	
	gluLookAt(vex, vey, vez, vlx, vly, vlz, 0, 1, 0);
}
//--Camera--------------------------------------------------------------
void camera(void)
{
	int distance = 3;
	dir_x = -sin(theta_xz), dir_y = tan(theta_yz),  dir_z = -cos(theta_xz);
	
	lx = ex + distance * dir_x;
	ly = cam_hgt + distance * dir_y;
	lz = ez + distance * dir_z;
	
	gluLookAt(ex, cam_hgt, ez, lx, ly, lz, 0., 1., 0.);	
}

//--Draw UFO-----------------------------------------------------------
void drawUFO(bool shadow)    
{
	const int N = 10;
	float vx[N] = {30, 30, 0, 10, 8.5, 7.5, 6, 4.5, 2, 0}; 
	float vy[N] = {5, 8, 0, 8, 10.5, 12.5, 14, 15, 15.5, 16};
	float vz[N] = {0};
	float wx[N], wy[N], wz[N];
	float angel = 2.0 * toRad;
	
	glRotatef(rotation0, 0, 1, 0);
	glRotatef(5, 0 ,0 ,1);
	glPushMatrix();
		for(int j = 0; j < 180; j++){
			for(int i = 0; i < N; i++){
				wx[i] = (vx[i] * cos(angel)) + (vz[i] * sin(angel));
				wy[i] = vy[i];
				wz[i] = vz[i] * cos(angel) - vx[i] * sin(angel) ;
			}
			if (shadow == true)
			glColor4f(0.2, 0.2, 0.2, 1.);
		    glBindTexture(GL_TEXTURE_2D, textureID[10]);
		    glEnable(GL_TEXTURE_2D);
			glBegin(GL_TRIANGLE_STRIP);
			for (int i = 0; i < N; i++)
				{
					if (i > 0) normal( vx[i - 1], vy[i - 1], vz[i - 1],
								wx[i - 1], wy[i - 1], wz[i - 1],
								vx[i], vy[i], vz[i]);		
					glVertex3f(vx[i], vy[i], vz[i]);
					glVertex3f(wx[i], wy[i], wz[i]);
					if (shadow == false)
					glTexCoord2f(j/180.0, i/(float)(N)); 
					
				}	
			glEnd();
			for (int i = 0; i < N; i++)
			{
				vx[i] = wx[i];
				vy[i] = wy[i];
				vz[i] = wz[i];
			}	
			glDisable(GL_TEXTURE_2D);
		}	

	glPopMatrix();
}
//--Draw cars-----------------------------------------------------------
void drawCars(bool shadow)
{
	//cab
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1.0);
	if (shadow == false)
    glColor4f(0.67, 0.78, 0.9, 1.0);
    glPushMatrix();
      glTranslatef(0.0, 8.5, 0.0);
      glScalef(15.0, 10.0, 10.0);
      glutSolidCube(1.0);
    glPopMatrix();
	
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1.0);
	if (shadow == false)
	glColor4f(0.67, 0.78, 0.9, 1.0);
    glPushMatrix();
      glTranslatef(-10.0, 6, 0.0);
      glScalef(5.0, 5.0, 10.0);
      glutSolidCube(1.0);
    glPopMatrix();
    
    //Wheels
    if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1.0);
	if (shadow == false)
    glColor4f(0.27, 0.22, 0.25, 1.0);
    glPushMatrix();                   //left-front wheel
      glTranslatef(-10.0, 3.5, 5.1);
      gluDisk(q, 0.0, 2.0, 20, 2);
    glPopMatrix();
    
    glPushMatrix();                  //left-back wheel
      glTranslatef(5.5, 3.5, 5.1);
      gluDisk(q, 0.0, 2.0, 20, 2);
    glPopMatrix();
    
    glPushMatrix();                  //right-front wheel
      glTranslatef(-10.0, 3.5, -5.1);
      glRotatef(180.0, 0., 1., 0.);
      gluDisk(q, 0.0, 2.0, 20, 2);
    glPopMatrix();
    
    glPushMatrix();                 //right-back wheel
      glTranslatef(5.5, 3.5, -5.1);
      glRotatef(180.0, 0., 1., 0.);
      gluDisk(q, 0.0, 2.0, 20, 2);
    glPopMatrix();
    
	//Windows
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1.0);
	if (shadow == false)
	glColor4f(0.47, 0.55, 0.71, 1.0);
	glPushMatrix();                 
	  glTranslatef(-5.5, 11, 0);
	  glScalef(4.1, 3.0, 10.1);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	//Car light
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1.0);
	if (shadow == false)
	glColor4f(1.0, 1.0, 0.0, 1.0);
	glPushMatrix();
      glTranslatef(-12.5, 7.25, -3.5);
      glutSolidSphere(1.0, 36, 36);  //left headlight
	glPopMatrix();
	
	glPushMatrix();
      glTranslatef(-12.5, 7.25, 3.5);
      glutSolidSphere(1.0, 36, 36);  //right headlight
	glPopMatrix();
	
	//front decoration
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1.0);
	if (shadow == false)
	glColor4f(1.0, 1.0, 1.0, 1.0);   //outward square
	glPushMatrix();  
	  glTranslatef(-12.0, 5.0, 0.0);
	  glScalef(1.1, 2.0, 5.0);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1.0);
	if (shadow == false)
    glColor4f(0.43, 0.55, 0.7, 1.0);   //inward square
	glPushMatrix();  
	  glTranslatef(-12.0, 5.0, 0.0);
	  glScalef(1.2, 1.5, 4.0);
	  glutSolidCube(1.0);
	glPopMatrix();	
}
 //--Draw Elevator
void drawElevator(float x, float y, float z, float h, bool shadow)
{
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1.0);
	if (shadow == false)
	//4-column
	glColor4f(0.78, 0.78, 0.55, 1);
	glPushMatrix();               //column-front-left
	  glTranslatef(x, y, z);
	  glScalef(0.9, h, 0.9);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix(); 
	  glTranslatef(x + 9, y, z);//column-front-right
	  glScalef(0.9, h, 0.9);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();               //column-back-left
	  glTranslatef(x, y, z - 9);
	  glScalef(0.9, h, 0.9);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix(); 
	  glTranslatef(x + 9, y, z - 9);//column-back-right
	  glScalef(0.9, h, 0.9);
	  glutSolidCube(1.0);
	glPopMatrix();
}
//--Draw Elevator-Up----------------------------------------------------
void drawElevatorUp()
{	
	glColor4f(0.98, 0.84, 0.22, 1);
    glPushMatrix();               //wing-elevator-up
	  glTranslatef(5, 63, -5);
	  glScalef(10, 1, 10);
	  glutSolidCube(1.0);
	glPopMatrix();
	
}
//--Draw Elevator-Bottom------------------------------------------------
void drawElevatorBottom(float x, float y, float z)
{
	glColor4f(0.23, 0.45, 0.94, 1);
	glPushMatrix();
	  glTranslatef(x, y, z);
	  glScalef(9, 1, 9);
	  glutSolidCube(1.0);
	glPopMatrix();
}

//--Draw Wing Building--------------------------------------------------
void drawWingBuilding(bool shadow)
{
	//wing-3
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1);
	if (shadow == false)
	glColor4f(1 ,0.98, 0.74, 1.);  
	glPushMatrix();                  //wing-3-front-1
	  glTranslatef(-55, 17.5, 70);
	  glScalef(50, 30, 1);
	  glutSolidCube(1.0);
	glPopMatrix();
	 
	glPushMatrix();                  //wing-3-front-2
	  glTranslatef(-85, 25, 70);
	  glScalef(10, 15, 1);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();                  //wing-3-left
	  glTranslatef(-89.5, 17.5, 60);
	  glScalef(1, 30, 20);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();                  //wing-3-back
	  glTranslatef(-70, 17.5, 50);
	  glScalef(40, 30, 1);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();                  //wing-3-right
	  glTranslatef(-30.5, 17.5, 60);
	  glScalef(1, 30, 20);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();                  //wing-3-up
	  glTranslatef(-60, 32.5, 60);
	  glScalef(60, 1, 20);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1);
	if (shadow == false)
	glColor4f(0.98, 0.8, 0.18, 1);
	glPushMatrix();                  //wing-3-bottom
	  glTranslatef(-60, 3, 60);
	  glScalef(58, 1, 18);
	  glutSolidCube(1.0);
	glPopMatrix();
	//wing-2
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1);
	if (shadow == false)
	glColor4f(0.75 ,0.72, 0.45, 1.); 
	glPushMatrix();                  //wing-2-left
	  glTranslatef(-49.5, 17.5, 30);
	  glScalef(1, 30, 40);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();                  //wing-2-right
	  glTranslatef(-30.5, 17.5, 30);
	  glScalef(1, 30, 40);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();                  //wing-2-up
	  glTranslatef(-40, 32.5, 30);
	  glScalef(20, 1, 40);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1);
	if (shadow == false)
	glColor4f(0.98, 0.8, 0.18, 1);
	glPushMatrix();                  //wing-2-bottom
	  glTranslatef(-40, 3, 30);
	  glScalef(18, 1, 42);
	  glutSolidCube(1.0);
	glPopMatrix();
	//wing-1
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1);
	if (shadow == false)
	glColor4f(1 ,0.98, 0.74, 1.);   //wing-1-front
	glPushMatrix();
	  glTranslatef(-10, 17.5, 9.5);
	  glScalef(40, 30, 1);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();                  //wing-1-left
	  glTranslatef(-49.5, 17.5, 0);
	  glScalef(1, 30, 20);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();                  //wing-1-back
	  glTranslatef(-20, 17.5, -9.5);
	  glScalef(60, 30, 1);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();                  //wing-1-right
	  glTranslatef(9.5, 17.5, 0);
	  glScalef(1, 30, 20);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();                  
	  glTranslatef(-25, 32.5, 0);   //wing-1-up-1
	  glScalef(50, 1, 20);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();
	  glTranslatef(5, 32.5, 5);     //wing-1-up-2
	  glScalef(10, 1, 10);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1);
	if (shadow == false)
	glColor4f(0.98, 0.8, 0.18, 1);
	glPushMatrix();                  //wing-1-bottom
	  glTranslatef(-20, 3, 0);
	  glScalef(58, 1, 18);
	  glutSolidCube(1.0);
	glPopMatrix();	
}
//--Draw Main Building--------------------------------------------------
void drawMainBuilding(bool shadow)
{
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1.0);
	if (shadow == false)
	glColor4f(0.26, 0.57, 0.3, 1);
	glPushMatrix();                  //main-front
	  glTranslatef(60, 31.5, 79.5);
	  glScalef(60, 62, 1);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();                  //main-back
	  glTranslatef(60, 31.5, -9.5);
	  glScalef(60, 62, 1);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();                  //main-right
	  glTranslatef(90, 31.5, 34.5);
	  glScalef(1, 62, 90);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();                  //main-left-1
	  glTranslatef(30.5, 26.5, -5);
	  glScalef(1, 52, 10);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();                  //main-left-2
	  glTranslatef(30.5, 31.5, 35);
	  glScalef(1, 62, 70);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();                  //main-left-3
	  glTranslatef(30.5, 40, 75);
	  glScalef(1, 45, 10);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();                  //main-up
	  glTranslatef(60, 62.5, 35);
	  glScalef(60, 1, 90);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1.0);
	if (shadow == false)
	glColor4f(0.98, 0.8, 0.18, 1);
	glPushMatrix();                  //main-bottom
	  glTranslatef(60, 2.5, 35);
	  glScalef(58, 1, 88);
	  glutSolidCube(1.0);
	glPopMatrix();
}
//--Draw Main Building Up-----------------------------------------------
void drawMainUp()
{
	glMaterialfv(GL_FRONT, GL_SPECULAR, black);
	
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, textureID[9]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.f, 0.f);  glVertex3f(30.0, 63.1, 80);
	glTexCoord2f(1.f, 0.f);  glVertex3f(90, 63.1, 80);
	glTexCoord2f(1.f, 1.f);  glVertex3f(90, 63.1, -10);
	glTexCoord2f(0.f, 1.f);  glVertex3f(30, 63.1, -10);
	glEnd();
	glDisable(GL_TEXTURE_2D);
	
	glMaterialfv(GL_FRONT, GL_SPECULAR, white);
}
//--Draw Tower----------------------------------------------------------
void drawTower(bool shadow)
{
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1.0);
	drawElevator(74.5, 104, 74.5, 80, true);
	if (shadow == false)
	glColor4f(0.95, 0.91, 0.68, 1);
	drawElevator(74.5, 104, 74.5, 80, false);
	
	glPushMatrix();
	  glTranslatef(79, 139, 70);
	  glScalef(10, 30, 10);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1.0);
	if (shadow == false)
	glColor4f(0.3, 0.3, 0.3, 1);
	glPushMatrix();
	  glTranslatef(79, 155, 70);
	  glScalef(7, 2, 7);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1.0);
	if (shadow == false)
	glColor4f(0.33, 0.06, 0.06, 1);
	glPushMatrix();
	  glTranslatef(79, 157, 70);
	  glScalef(10, 4, 10);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1.0);
	if (shadow == false)
	glColor4f(0.99, 0.92, 0.35, 1);
	glPushMatrix();
	  glTranslatef(79, 157, 70);
	  glScalef(14, 1.5, 14);
	  glutSolidCube(1.0);
	glPopMatrix();	
}
//--Draw Shooting ball--------------------------------------------------
void drawBall(bool shadow)
{
	//Shooting Ball base
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1);
	if (shadow == false)
	glColor4f(0.75 ,0.1, 0.15, 1.); 
	glPushMatrix();
	  glTranslatef(0, 10, -80);
	  glScalef(4, 15, 4);
	  glutSolidCube(1.0);
	  glTranslatef(0, 1, 0);
	glPopMatrix();
	
	glPushMatrix();
	  glTranslatef(0, 16, -80);
	  glutSolidSphere(3, 36, 20);
	glPopMatrix();
	
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1.0);
	if (shadow == false)
	glColor4f(0.12, 0.6, 0.3, 1);
	glPushMatrix();
	  glTranslatef(0, 18, -80);
	  glRotatef(90, 0, 1, 0);
	  glRotatef(-75, 1, 0, 0);
	  gluCylinder(q, 1, 5, 5, 180, 2);
      glTranslatef(0, 0, 5);
      gluDisk(q, 4, 5, 36, 20);
    glPopMatrix();
	
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1.0);
	if (shadow == false)
	glColor4f(0.85, 0.97, 0.34, 1);
	glPushMatrix();
	  glTranslatef(bx, by, -80);
	  glutSolidSphere(1, 36, 20);
	glPopMatrix();  
}
//--Draw Connection Bridge----------------------------------------------
void drawConnectionBridge(bool shadow)
{
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1.0);
	if (shadow == false)
	glColor4f(1 ,0.98, 0.74, 1.);   //bridge-bottom
	glPushMatrix();                 
	  glTranslatef(20, 53, -5);
	  glScalef(20, 1, 10);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	if (shadow == true)
	glColor4f(0.4, 0.4, 0.4, 1.0);
	if (shadow == false)
	glColor4f(0.67, 0.82, 0.92, 1);
	glPushMatrix();                  //rail-right
	  glTranslatef(20, 57, -0.5);
	  glScalef(20, 1, 1);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();                  //rail-left
	  glTranslatef(20, 57, -9.5);
	  glScalef(20, 1, 1);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	int i;
	for (i = 0; i < 16; i +=3)
	{
		if (shadow == true)
		glColor4f(0.4, 0.4, 0.4, 1.0);
		if (shadow == false)
		glColor4f(0.32, 0.39, 0.49, 1);
		glPushMatrix();             //rail-right-straight
		  glTranslatef(12.5 + i, 55, -0.5);
		  glScalef(1, 3, 1);
		  glutSolidCube(1.0);
		glPopMatrix();
		
		glPushMatrix();             //rail-left-straight
		  glTranslatef(12.5 + i, 55, -9.5);
		  glScalef(1, 3, 1);
		  glutSolidCube(1.0);
		glPopMatrix();
	}	
}
//--Draw the fifth floor------------------------------------------------
void drawFifthFloor()
{
	int i;
	glColor4f(0.2, 0.2, 0.2, 1);
	glPushMatrix();                 //fifth-floor-bottom
	  glTranslatef(36, 53, 25);
	  glScalef(10, 1, 70);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glColor4f(0.2, 0.13, 0.11, 1);
	glPushMatrix();                //rail-up
	  glTranslatef(39.5, 57, 25);
	  glScalef(1, 1, 70);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glColor4f(0.93, 0.93, 0.65, 1);//rail-straight
	for (i = -4.5; i < 61; i += 5)
	{
		glPushMatrix();     
		  glTranslatef(39.5, 55, i);
		  glScalef(1, 3, 1);
		  glutSolidCube(1.0);
		glPopMatrix();
	}       
}
//--Draw Windmill-------------------------------------------------------
void drawWindmill()
{
	glColor4f(0.8, 0, 0, 1);
	glPushMatrix();
	  glTranslatef(0, 1.5, 0);
	  glScalef(5, 3, 5);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glColor4f(0, 0.6, 0.99, 1);
	glPushMatrix();
	  glTranslatef(0, 3, 0);
	  glutSolidSphere(1, 20, 20);
	glPopMatrix();
	
	glColor4f(0.73, 0.52, 0.12, 1);
	for (int wind_theta = 0; wind_theta < 360; wind_theta += 90)
	{
		glPushMatrix();
		  glRotatef(wind_theta, 0, 1, 0);
		  glTranslatef(0, 1.5, 10);
		  glScalef(3, 1, 15);
		  glutSolidCube(1.0);
		glPopMatrix();
	}
}
//--Draw Clock
void Pendulum()
{
	glColor4f(0, 0.6, 0.99, 1);
	glPushMatrix();
	  glRotatef(90, 1, 0, 0);
	  glTranslatef(0, 0.5, 15);
	  glScalef(1, 1, 20);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glColor4f(0.8, 0.3, 0.2, 1);
	glPushMatrix();
      glTranslatef(0, -26.5, 0);
      gluCylinder(q, 1.5, 1.5, 1, 180, 2);
      glTranslatef(0, 0, 1);
      gluDisk(q, 0, 1.5, 36, 20);
    glPopMatrix(); 
}

void drawClock()
{
	//Clock's base
	glColor4f(0.9, 0.9, 0.9, 1);
    glPushMatrix();
      gluCylinder(q, 5, 5, 1, 180, 2);
      glTranslatef(0, 0, 1);
      gluDisk(q, 0, 5, 36, 20);
    glPopMatrix(); 
    //time of clock
	glColor4f(0.87, 0.6, 0.87, 1);
	glPushMatrix();
	glRotatef(90, 1, 0, 0);
    for (int theta = 0; theta < 360; theta += 30)
    {
		glColor4f(0, 0, 0, 1);
		glPushMatrix();
		  glRotatef(theta, 0, 1, 0);
		  glTranslatef(0, 1, -4.5);
		  glutSolidSphere(0.5, 36, 20);
		glPopMatrix();
	  }
	glPopMatrix();
	//indicator of clock
	glColor4f(0.83, 0.99, 0.34, 1);
	glPushMatrix();
	  glRotatef(90, 1, 0, 0);
	  glRotatef(-theta_clk2, 0, 1, 0);
	  glTranslatef(0, 1.3, -1.8);
	  glScalef(0.5, 0.5, 3.6);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();
	  glRotatef(90, 1, 0, 0);
	  glRotatef(-theta_clk1, 0, 1, 0);
	  glTranslatef(0, 1.6, -1.2);
	  glScalef(0.5, 0.5, 2.4);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();
	  glTranslatef(0, -5, 0);
	  glRotatef(theta_pdl, 0, 0, 1);
	  glTranslatef(0, 5, 0);
	  Pendulum();
	glPopMatrix();
}

//--Draw Visitor--------------------------------------------------------
void drawHat()
{
    glColor4f(0.88, 0.26, 0.38, 1);
	glPushMatrix();                  
	  glTranslatef(0, 9.3, 0);
      glScalef(1.6, 1, 1.6);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glColor4f(1, 1, 1, 1);
	glPushMatrix();
	  glTranslatef(0, 8.9, 1);
	  glScalef(1.6, 0.2, 0.4);
	  glutSolidCube(1.0);
	glPopMatrix();
}

void drawHead()
{
	glColor4f(0.95, 0.71, 0.25, 1);   //draw head
	glPushMatrix();             
	glTranslatef(0, 8.2, 0);
	glutSolidCube(1.4);
	glPopMatrix();
	
	glColor4f(1 ,1, 1, 1);            //draw left-eye round
	glPushMatrix();
	glTranslatef(-0.35, 8.35, 0.6);
	glutSolidSphere(.2, 20, 20);
	glPopMatrix();
	glPushMatrix();                   //draw right-eye round
	glTranslatef(0.35, 8.35, 0.6);
	glutSolidSphere(.2, 20, 20);
	glPopMatrix();

	glColor4f(0, 0, 0, 1);           
	glPushMatrix();                   //draw left-eye ball
	glTranslatef(-0.35, 8.35, 0.7);
	glutSolidSphere(.15, 20, 20);
	glPopMatrix();
	glPushMatrix();                   //draw right-eye ball
	glTranslatef(0.35, 8.35, 0.7);
	glutSolidSphere(.15, 20, 20);
	glPopMatrix();
	
	glColor4f(0.8, 0., 0, 1);         //draw mouth
	glPushMatrix();
	glTranslatef(0, 7.8, 0.7);
	glScalef(0.6, 0.2, 0.1);
	glutSolidCube(1.0);
	glPopMatrix();	
}
void drawVisitor()
{
	drawHat();
	drawHead();
	glColor4f(0, 0.55, 0.8, 1);            //draw body
	glPushMatrix();
	glTranslatef(0, 6.3, 0);
	glScalef(2.4, 2.6, 1.2);
	glutSolidCube(1.0);
	glPopMatrix();
	
	//draw front body
	glColor4f(0.48, 0.61, 0.87, 1);
	glPushMatrix();  
	  glTranslatef(-1.6, 7.5, 0);   //left-arm
	  glRotatef(-walk_angle, 1, 0, 0);
	  glTranslatef(1.6, -7.5, 0);                
	  glTranslatef(-1.6, 6, 0);
	  glScalef(0.8, 3, 0.8);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();                  //right-arm
	  glTranslatef(1.6, 7.5, 0);
	  glRotatef(walk_angle, 1, 0, 0);
	  glTranslatef(-1.6, -7.5, 0);
	  glTranslatef(1.6, 6, 0);
	  glScalef(0.8, 3, 0.8);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glColor4f(0.28, 0.71, 0.67, 1);
	glPushMatrix();                 //left-leg                
	  glTranslatef(-0.6, 4.9, 0);
	  glRotatef(walk_angle, 1, 0, 0);
	  glTranslatef(0.6, -4.9, 0);
	  glTranslatef(-0.6, 3.2, 0); 
	  glScalef(0.8, 3.8, 0.8);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();                  //right-leg
      glTranslatef(0.6, 4.9, 0);
	  glRotatef(-walk_angle, 1, 0, 0);
	  glTranslatef(-0.6, -4.9, 0); 
	  glTranslatef(0.6, 3.2, 0);
	  glScalef(0.8, 3.8, 0.8);
	  glutSolidCube(1.0);
	glPopMatrix();	
}
void drawBody()
{
	glPushMatrix();
	  glTranslatef(0, 2, 0);
	  drawVisitor();
	glPopMatrix();
}

//--Move visitor--------------------------------------------------------
void moveVisitor(float x, float y, float z, float turn_degree)
{
	glPushMatrix();                 
	  glTranslatef(x, y, z);
	  glRotatef(turn_degree, 0, 1, 0);
	  drawBody();
	glPopMatrix();
}

//--Draw Iron Man-------------------------------------------------------
void drawIronMan()
{
	//head
	glColor4f(0.51, 0.09, 0.11, 1);           
	glPushMatrix();
	  glTranslatef(0, 18, 0);
	  glScalef(1.4, 2, 1);
	  glutSolidCube(1.0);
	glPopMatrix();
	//texture of head
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, textureID[6]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.f, 0.f);  glVertex3f(-0.7, 17, 0.75);
	glTexCoord2f(1.f, 0.f);  glVertex3f(0.7, 17, 0.75);
	glTexCoord2f(1.f, 1.f);  glVertex3f(0.7, 19, 0.75);
	glTexCoord2f(0.f, 1.f);  glVertex3f(-0.7, 19, 0.75);
	glEnd();
	glDisable(GL_TEXTURE_2D);
	//body
	glPushMatrix();
	  glTranslatef(0, 14, 0);
	  glScalef(4, 6, 2);
	  glutSolidCube(1.0);
	glPopMatrix();
	//texture front of body
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, textureID[7]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.f, 0.f);  glVertex3f(-2, 11, 1.05);
	glTexCoord2f(1.f, 0.f);  glVertex3f(2, 11, 1.05);
	glTexCoord2f(1.f, 1.f);  glVertex3f(2, 17, 1.05);
	glTexCoord2f(0.f, 1.f);  glVertex3f(-2, 17, 1.05);
	glEnd();
	glDisable(GL_TEXTURE_2D);
	//texture back of body
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, textureID[8]);
	glBegin(GL_QUADS);
	  glTexCoord2f(0.f, 0.f);  glVertex3f(-2, 11, -1.05);
	  glTexCoord2f(1.f, 0.f);  glVertex3f(2, 11, -1.05);
	  glTexCoord2f(1.f, 1.f);  glVertex3f(2, 17, -1.05);
	  glTexCoord2f(0.f, 1.f);  glVertex3f(-2, 17, -1.05);
	glEnd();
	glDisable(GL_TEXTURE_2D);
	
	glPushMatrix();  
	  glTranslatef(2.4, 14, 0);   //left-arm
	  glRotatef(15, 0, 0, 1);
	  glTranslatef(-2.4, -14, 0);                
	  glTranslatef(3.05, 14, 0);
	  glScalef(0.8, 6, 0.8);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();  
	  glTranslatef(-2.4, 14, 0);   //right-arm
	  glRotatef(-15, 0, 0, 1);
	  glTranslatef(2.4, -14, 0);                
	  glTranslatef(-2.75, 14, 0);
	  glScalef(0.8, 6, 0.8);
	  glutSolidCube(1.0);
	glPopMatrix();

    glPushMatrix();                //left-leg
	  glTranslatef(-1, 7, 0);
	  glScalef(1.2, 8, 1.2);
	  glutSolidCube(1.0);
	glPopMatrix();
	
	glPushMatrix();                //right-leg
	  glTranslatef(1, 7, 0);
	  glScalef(1.2, 8, 1.2);
	  glutSolidCube(1.0);
	glPopMatrix();
	//light on front body
	if (energy_on == true) glColor4f(0, 0.95, 0.99, 1);
	else glColor4f(0.2, 0.2, 0.2, 1);
	glPushMatrix();                //energe-ball
	  glTranslatef(0, 15.5, 1);
	  glLightfv(GL_LIGHT3, GL_POSITION, lgt_pos3);   //set energy-ball's light
      glLightfv(GL_LIGHT3, GL_SPOT_DIRECTION, spot_light3);
      glLightf(GL_LIGHT3, GL_SPOT_CUTOFF, 15.0);
      glLightf(GL_LIGHT3, GL_SPOT_EXPONENT, 200);
	  glutSolidSphere(0.8, 20, 20);
	glPopMatrix();
}
//--Move Iron Man-------------------------------------------------------
void freefallIronMan() //free-fall motion of Iron Man
{
	glPushMatrix();
	  glTranslatef(80, fall_hgt, 46);//fall_hgt defined as distance formula of free-fall motion in time function
	  glRotatef(90, 1, 0, 0);
	  drawIronMan();
	glPopMatrix();
}
void flyupIronMan()
{
	glPushMatrix();
	  glTranslatef(80, 145, 46);
	  glRotatef(rotation1, 1, 0, 0);
	  glTranslatef(0, -80, 0);
	  glRotatef(180, 0, 1, 0);
	  glRotatef(60, 1, 0, 0);
      drawIronMan();
    glPopMatrix();
}
void flyIronMan()
{
	glPushMatrix();
	  glTranslatef(80, 145, fly_z1);
	  glRotatef(90, 1, 0, 0);
	  drawIronMan();
	glPopMatrix();
}

//--Keyboards-----------------------------------------------------------
void keyboard(unsigned char key, int x, int y)
{
	if (key == 'W' || key == 'w') theta_yz += 0.1;
	if (key == 'S' || key == 's') theta_yz -= 0.1;
	if (key == 'Q' || key == 'q') cam_hgt += 5;
	if (key == 'E' || key == 'e') cam_hgt -= 5;
	if (key == ' ' && pause == false) pause = true;
	else if (key == ' ' && pause == true) pause = false;
	glutPostRedisplay();
}

//--Special Keys--------------------------------------------------------
void special(int key, int x, int y)
{
	if (key == GLUT_KEY_UP) 
	{
		step = 1;
		ex = ex + step * dir_x * cam_speed;
		ez = ez + step * dir_z * cam_speed;
	}	
	else if (key == GLUT_KEY_DOWN) 
	{
		step = -1;
		ex = ex + step * dir_x * cam_speed;
		ez = ez + step * dir_z * cam_speed;	
	}
	else if (key == GLUT_KEY_LEFT) theta_xz += 0.1;
	else if (key == GLUT_KEY_RIGHT) theta_xz -= 0.1;
	else if (key == GLUT_KEY_F1 && moveView == false) moveView = true;
	else if (key == GLUT_KEY_F1 && moveView == true) moveView = false;
	glutPostRedisplay();
}
//--Display: -----------------------------------------------------------
void display(void) 
{ 
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	//Cameras
	if (moveView == false) camera();
	if (moveView == true) 
	{ 
		glRotatef(240, 0, 1, 0);
		glTranslatef(0, UFO_hgt, 140);
		glRotatef(-rotation0, 0, 1, 0);	
		cameraUFO();
	}
	glLightfv(GL_LIGHT0, GL_POSITION, lpos);   //Set outside-light position
	
	//Light3 for Iron Man
	if (energy_on == true)  glEnable(GL_LIGHT3);
	if (energy_on == false) glDisable(GL_LIGHT3);
	
	//Draw environment
    drawFloor();                           
    drawCircularRoad();
    drawSkybox();   
	
	//Car
	glDisable(GL_LIGHTING);
    glPushMatrix();  
	  glRotatef(rotation, 0, 1, 0);
	  glTranslatef(0, 3.0, -167);
	  glMultMatrixf(shadowMat);                          
	  glLightfv(GL_LIGHT1, GL_POSITION, lgt_pos1);   //set car's left-light
      glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, spot_light1);
      glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, 30.0);
      glLightf(GL_LIGHT1, GL_SPOT_EXPONENT, -0.1);
      glLightfv(GL_LIGHT2, GL_POSITION, lgt_pos2);   //set car's right-light
      glLightfv(GL_LIGHT2, GL_SPOT_DIRECTION, spot_light2);
      glLightf(GL_LIGHT2, GL_SPOT_CUTOFF, 30.0);
      glLightf(GL_LIGHT2, GL_SPOT_EXPONENT, -0.1);
	  drawCars(true);
	glPopMatrix();
	
	glEnable(GL_LIGHTING);
	glPushMatrix();                             //move car with lights
	glRotatef(rotation, 0, 1, 0);
	glTranslatef(0, 1.0, -167);
	glLightfv(GL_LIGHT1, GL_POSITION, lgt_pos1);   //set car's left-light
    glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, spot_light1);
    glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, 30.0);
    glLightf(GL_LIGHT1, GL_SPOT_EXPONENT, -0.1);
    glLightfv(GL_LIGHT2, GL_POSITION, lgt_pos2);   //set car's right-light
    glLightfv(GL_LIGHT2, GL_SPOT_DIRECTION, spot_light2);
    glLightf(GL_LIGHT2, GL_SPOT_CUTOFF, 30.0);
    glLightf(GL_LIGHT2, GL_SPOT_EXPONENT, -0.1);
	drawCars(false);
	glPopMatrix();
	
	//buildings's shadow
	glDisable(GL_LIGHTING);
    glPushMatrix();              
      glTranslatef(0, 1.5, 0);  
      glMultMatrixf(shadowMat);   
	  drawWingBuilding(true);
	glPopMatrix();
	
	glPushMatrix();
	  glTranslatef(-5, 1.5, 0);
	  glMultMatrixf(shadowMat);
	  drawConnectionBridge(true);
	glPopMatrix();
	
	glPushMatrix();
	  glTranslatef(10, 1.5, 0);
	  glScalef(0.5, 1, 1);
	  glMultMatrixf(shadowMat);
	  drawMainBuilding(true);
	  drawTower(true);	
	glPopMatrix();
	glEnable(GL_LIGHTING);
	
	//Draw buildings
	drawMainUp();
	drawMainBuilding(false);
	drawTower(false);	
    drawWingBuilding(false);	
	drawConnectionBridge(false);
	drawFifthFloor();
	if (stay_elevator1 == true)
	{
		drawElevator(0.5, 32.5, -0.5, 60, false);
		drawElevatorUp();
		drawElevatorBottom(5, 3.5, -5);
	}
	if (stay_elevator1 == false)
	{
		drawElevator(0.5, 32.5, -0.5, 60, false);
		drawElevatorUp();
	}
	if (stay_elevator2 == true)
	{
		drawElevator(30.5, 32.5, 69.5, 60, false);
		drawElevatorBottom(35, 53, 65);
	}
	if (stay_elevator2 == false)
	{
		drawElevator(30.5, 32.5, 69.5, 60, false);
	}	
	
	//Windmill in main building	
	glPushMatrix();
	  glTranslatef(90, 35, 40);
	  glRotatef(90, 0, 0, 1);
	  glRotatef(rotation0, 0, 1, 0);
	  drawWindmill();
	glPopMatrix();
	
	//Clock in Main building
	glPushMatrix();
	  glTranslatef(60, 35, -9);
	  drawClock();
	glPopMatrix();
	
	//Shooting Ball
	
	glDisable(GL_LIGHTING);
    glPushMatrix();              
      glTranslatef(0, 1.5, 0);  
      glMultMatrixf(shadowMat);   
	  drawBall(true);
	glPopMatrix();
	glEnable(GL_LIGHTING);
		
	drawBall(false);
	//Iron Man
	if (fly_up == true) flyupIronMan();
	if (fly_straignt == true) flyIronMan();
	if (fall == true)  freefallIronMan();
	
	//Visitor	
	if (path1 == true) moveVisitor(-84, 0, z1, 180);
	if (turn_right1 == true) moveVisitor(-84, 0, 60, turn_theta);
	if (path2 == true) moveVisitor(x1, 0, 60, 90);
	if (turn_left1 == true) moveVisitor(-40, 0, 60, turn_theta1);	
	if (path3 == true) moveVisitor(-40, 0, z2, 180);
	if (turn_right2 == true) moveVisitor(-40, 0, -5, turn_theta2);
	if (path4 == true) moveVisitor(x2, 0, -5, 90);
	if (go_up == true)
	{
		moveVisitor(5, elevator_height1, -5, 90);
		glPushMatrix();
	      glTranslatef(2.5, elevator_height1, -2.5);
	      drawElevatorBottom(2.5, 3, -2.5);
		glPopMatrix();
	}
	if (go_down1 == true)
	{
		glPushMatrix();
	      glTranslatef(2.5, elevator_height1_1, -2.5);
	      drawElevatorBottom(2.5, 3, -2.5);
		glPopMatrix();
	}
	if (path5 == true) moveVisitor(x3, 50, -5, 90);
	if (turn_right3 == true) moveVisitor(35, 50, -5, turn_theta3);
	if (path6 == true) moveVisitor(35, 50, z3, 0);
	if (go_down == true) 
	{
		moveVisitor(35, elevator_height2, 65, 0);
		glPushMatrix();
		  glTranslatef(17.5, elevator_height2, 32.5);
		  drawElevatorBottom(17.5, 3, 32.5);
		glPopMatrix();
	}
	if (go_up1 == true)
	{
		glPushMatrix();
		  glTranslatef(17.5, elevator_height2_2, 32.5);
		  drawElevatorBottom(17.5, 3, 32.5);
		glPopMatrix();
	}
	if (path7 == true) moveVisitor(35, 0, z4, 0);
	if (turn_right4 == true) moveVisitor(35, 0, 75, turn_theta4);
	if (path8 == true) moveVisitor(x5, 0, 75, -90);
	if (turn_right5 == true) moveVisitor(-84, 0, 75, turn_theta5);
	
	//UFO
	glDisable(GL_LIGHTING);    //shadow of UFO
	glPushMatrix();
	glRotatef(175, 0, 1, 0);
	glRotatef(rotation0, 0, 1, 0);
	glTranslatef(0, 3, 140);	
	glMultMatrixf(shadowMat);                          
	drawUFO(true);
	glPopMatrix();
	
	glEnable(GL_LIGHTING);    
	glPushMatrix();
	glRotatef(175, 0, 1, 0);
	glRotatef(rotation0, 0, 1, 0);	
	glTranslatef(0, UFO_hgt1, 140);
	drawUFO(false);
	glPopMatrix();
	
    glutSwapBuffers(); 
} 

//--My timer------------------------------------------------------------
void myTimer(int value)
{
	//UFO: go around the round road
	if (pause == false)
	{
	if (rotation0 < 360) rotation0 += 2;
	if (rotation0 >= 360) 
	{
		rotation0 = 0;
		rotation0++;
	}
	UFO_hgt += UFO_dis;
	UFO_hgt1 += UFO_dis1;
	if (UFO_hgt == 10 || UFO_hgt == -50) UFO_dis = -UFO_dis;
	if (UFO_hgt1 == 5 || UFO_hgt1 == 65) UFO_dis1 = -UFO_dis1; 
	
	//round-road
	if (rotation < 360) rotation++;
	if (rotation >= 360) 
	{
		rotation = 0;
		rotation++;
	}
	
	//Clock in the main building
	theta_pdl += 2 * swing_pdl;
	if (theta_pdl == 16 | theta_pdl == -16) swing_pdl = -swing_pdl;
	if (theta_pdl == 16) theta_clk2 += 30;	
	if (theta_clk2 >= 360 && theta_clk2 % 360 == 0 && theta_pdl == 16) theta_clk1 += 30; 
	
	//Shooting Ball
	if (by > 0)
	{
		ball_t += 0.5;
		by =20 + v0 * sin(theta_ball * toRad) * ball_t - 0.5 * gravity * ball_t * ball_t; 
		bx =v0 * ball_t * cos(theta_ball * toRad);
	}
    if (by < 0) 
    {
		by = 20;
		bx = 0;
		ball_t = 0;
	}
	
	//Iron Man
	if (fly_up == true && rotation1 < 90) rotation1 += 5; //fly up as a 1/4 circle path from the up of the main building 
	if (rotation1 == 90)
	{
		fly_straignt = true;
		fly_up = false;
		rotation1 = 0;
		fly_z1 = -19;
	}
	if (fly_straignt == true && fly_z1 < 46) fly_z1 += 5;  //fly straight to the tower
	if (fly_z1 == 46)
	{
		fall = true;
		fly_straignt = false;
		fly_z1 = 0;
		fall_hgt = 145;
		fall_t = 0;
	}
	if (fall == true && fall_hgt > 65) //free-fall motion
	{
		fall_t += 1;
		fall_hgt =145 - 0.5 * gravity * fall_t * fall_t; //formula: h = 1/2 * g * t ^ 2
	}
	if (fall_hgt == 60.5)
	{
		fly_up = true;
		fall = false;
		fall_hgt = 0;
		rotation1 = 0;
	}
	//Iron Man Light
	if (fall == true) energy_on = false; //When Iron Man hit the wall, light turned down
	else energy_on = true;	
	
	//walking motion of visitor
	walk_angle += 2 * walk_cycle;
	if (walk_angle == 20 | walk_angle == -20) walk_cycle = -walk_cycle;	
	
	//path of visitor
	if (z1 > 60) z1 -= walk_step;   //path1: go into the wing building
	if (z1 == 60)
	{
		path1 = false;
		turn_right1 = true;
		z1 = 0;
		turn_theta = 180;
	}
	if (turn_right1 == true && turn_theta > 90) turn_theta -= walk_turn; //turn_right1
	if (turn_theta == 90) 
	{
		path2 = true;
		turn_right1 = false;
		turn_theta = 0;
		x1 = -84;
	}
	if (path2 == true && x1 < -40) x1 += walk_step;  //path2
	if (x1 == -40)
	{
		path2 = false;
		turn_left1 = true;
		x1 = -39;
		turn_theta1 = 90;
	}
	if (turn_left1 == true && turn_theta1 < 180) turn_theta1 += walk_turn;  //trun_left1
	if (turn_theta1 == 180)
	{
		turn_left1 = false;
		path3 = true;
		turn_theta1 = 0;
		z2 = 60;
	}
	if (path3 == true && z2 > -5) z2 -= walk_step;  //path3
	if (z2 == -5)
	{
		path3 = false;
		turn_right2 = true;
		z2 = 0;
		turn_theta2 = 180;
	}
	if (turn_right2 == true && turn_theta2 > 90) turn_theta2 -= walk_turn;  //turn_right2
	if (turn_theta2 == 90)
	{
		path4 = true;
		turn_right2 = false;
		turn_theta2 = 0;
		x2 = -40;
	}
	if (path4 == true && x2 < 5) x2 += walk_step;  //path4
	if (x2 == 5)
	{
		path4 = false;
		go_up = true;
		x2 = 0;
		elevator_height1 = 0;
		x3 = 5;
	}
	if (go_up == true && elevator_height1 < 50) elevator_height1++;  //go up the elevator
	if (elevator_height1 == 50)
	{
		go_up = false;
		path5 = true;
		go_down1 = true;
		elevator_height1 = 0;
		elevator_height1_1 = 50;
	}
	if (go_down1 == true && elevator_height1_1 > 0) elevator_height1_1--;
	if (elevator_height1_1 == 0) go_down1 = false;
	if (path5 == true && x3 < 35) x3 += walk_step;  //path 5: cross the connecting bridge
	if (x3 == 35)
	{
		path5 = false;
		turn_right3 = true;
		x3 = 0;
		turn_theta3 = 90;
	}
	if (turn_right3 == true && turn_theta3 > 0) turn_theta3 -= walk_turn; //turn_right3
	if (turn_theta3 == 0)
	{
		turn_right3 = false;
		path6 = true;
		turn_theta3 = -1;
		z3 = -5;
	}
	if (path6 == true && z3 < 65) z3 += walk_step;  //path6:: go through the ale of fifth floor
	if (z3 == 65)
	{
		path6 = false;
		go_down = true;
		z3 = 0;
		elevator_height2 = 50;
		z4 = 65;
	}
	if (go_down == true && elevator_height2 > 0) elevator_height2--; //go down with the elevator
	if (elevator_height2 == 0)
	{
		path7 = true;
		go_down = false;
		go_up1 = true;
		elevator_height2 = 50;
		elevator_height2_2 = 0;
	}
	if (go_up1 == true && elevator_height2_2 < 50) elevator_height2_2++; //path7
	if (elevator_height2_2 == 50) go_up1 = false;
	if (path7 == true && z4 < 75) z4 += walk_step;
	if (z4 == 75)
	{
		path7 = false;
		turn_right4 = true;
		z4 = 0;
		turn_theta4 = 0;
	}
	if (turn_right4 == true && turn_theta4 > -90) turn_theta4 -= walk_turn; //turn_right4 
	if (turn_theta4 == -90)
	{
		turn_right4 = false;
		path8 = true;
		turn_theta4 = 0;
		x5 = 35;	
	}
	if (path8 == true && x5 > -84) //path8: go back to the start point
	{
		x5 -= walk_step;
	}
	if (x5 == -84)
	{
		path8 = false;
		turn_right5 = true;
		x5 = 0;
		turn_theta5 = -90;
	}
	if (turn_right5 == true && turn_theta5 > -180) turn_theta5 -= walk_turn; //turn_right5
	if (turn_theta5 == -180)
	{
		turn_right5 = false;
		turn_theta5 = 0;
		z1 = 75;
		path1 = true;
	}	
	if (go_up == true || go_down1 == true) stay_elevator1 = false;
	else stay_elevator1 = true;
	if (go_down == true || go_up1 == true) stay_elevator2 = false; 
	else stay_elevator2 = true;
	glutPostRedisplay();
	}
	
	glutTimerFunc(50, myTimer, 0);
	printf("%f\n", by);
	
}
//----------------------------------------------------------------------
void initialize(void)
{
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);  
	q = gluNewQuadric();	
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
	loadTexture();
	glEnable(GL_LIGHTING); 
    glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT2);
	glEnable(GL_COLOR_MATERIAL);
 	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glMaterialfv(GL_FRONT, GL_SPECULAR, white);
	glMaterialf(GL_FRONT, GL_SHININESS, 20);
	//Define light0's ambient, diffuse, specular properties
    glLightfv(GL_LIGHT0, GL_AMBIENT, grey);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, grey);
    glLightfv(GL_LIGHT0, GL_SPECULAR, grey);
    //Define other lights
    glLightfv(GL_LIGHT1, GL_SPECULAR, white);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, yellow);
	glLightfv(GL_LIGHT2, GL_SPECULAR, white);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, yellow);
    glLightfv(GL_LIGHT3, GL_SPECULAR, white);
	glLightfv(GL_LIGHT3, GL_DIFFUSE, blue);

 	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);
    gluQuadricDrawStyle(q, GLU_FILL);
	glClearColor (0.0, 0.0, 0.0, 0.0);  //Background colour
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    gluPerspective(60., 1.0, 10.0, 1000.0);
}
//  ------- Main: Initialize glut window and register call backs -------
int main(int argc, char **argv) 
{ 
	glutInit(&argc, argv);            
	glutInitDisplayMode(GLUT_SINGLE | GLUT_DEPTH);  
	glutInitWindowSize(600, 600);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("Assignment1");
	initialize();
	glutSpecialFunc(special); 
	glutDisplayFunc(display);
	glutKeyboardFunc(keyboard);
	glutTimerFunc(50, myTimer, 0);
	glutMainLoop();
	return 0; 
}
