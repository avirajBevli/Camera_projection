//In this program, we take actual values and predict the results
//Assumed we have been given the following information- 
		//The robot's pose in the threeD world(x0,y0,theeta)
		//The orientation of the camera wrto the robot
		//We have the threeD coordinates of the corners of the doors(all doors have the same height)
		//We want to find the outlines of all the doors in the image
		//We assume that the doors lie on two walls, one wall along y = 0 and the other along y = 10*1000mm

//Proposed Algorithm: Since it is relatively difficult to detect door in the image, we will instead try to detect doors in the threeD coordinates itself 
//Hence we sort the coordinates of the door according to their x coordinates and then according to their z coordinates
//We discard those 4 points of a door in which 2 points lie on one side and the other 2 points lie on the other side of the camera
//Then sets of 4 points will belong to the same door and we can hence draw the boundries of the door after projecting the points on the image

//To generate different views of the hallway, we can change the values of x0, y0, theeta present in line "190-192" of the code

#include<bits/stdc++.h> 
#include <iostream>
#include <fstream>//inlcude the fstream class
#include <math.h> 
#include <vector>
#include <stdlib.h>
#include <stdio.h> 
#include <sstream>
#include <algorithm>//for the sort function in

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"

using namespace std;
using namespace cv;

#define pi 3.141592653589793238;

typedef struct three_D_pt
{
	double x, y, z;
}three_D_pt;

typedef struct two_D_pt
{
	double x, y;
}two_D_pt;

typedef struct line_eqn
{
	double x0, y0, theeta_line;
}line_eqn;


//function to obtain the final projection matrix from K, Mrc, Mwr matrices
void matMul3(double K[3][4], double Mrc[4][4], double Mwr[4][4], double P[3][4])
{
	int i, j, k;
	double temp[4][4];
	
	cout<<"Mrc is:"<<endl;
	for(i=0;i<4;i++)
	{	
		for(j=0;j<4;j++)
			cout<<Mrc[i][j]<<" ";
		cout<<endl<<endl;
	}

	cout<<"Mwr is:"<<endl;
	for(i=0;i<4;i++)
	{	
		for(j=0;j<4;j++)
			cout<<Mwr[i][j]<<" ";
		cout<<endl<<endl;
	}

	cout<<"The product of Ms is:"<<endl;
	for(i=0;i<4;i++)
	{	
		for(j=0;j<4;j++)
		{
			temp[i][j] = 0;
			for(k=0;k<4;k++)
				temp[i][j] += Mrc[i][k]*Mwr[k][j];

			cout<<temp[i][j]<<" ";
		}
		cout<<endl;
	}

	cout<<endl<<endl<<"P is:"<<endl;
	for(i=0;i<3;i++)
	{	
		for(j=0;j<4;j++)
		{
			P[i][j] = 0;		
			for(k=0;k<4;k++)
				P[i][j] += K[i][k]*temp[k][j];

			cout<<P[i][j]<<" ";
		}
		cout<<endl;
	}

	cout<<endl<<endl;
}

two_D_pt project_to_image(double P[3][4], three_D_pt point)
{
	double temp[4];
	temp[0] = point.x;
	temp[1] = point.y;
	temp[2] = point.z;
	temp[3] = 1;

	double result[3];
	int i, j;
	for(i=0;i<3;i++)
	{
		result[i] = 0;
		for(j=0;j<4;j++)
			result[i] += P[i][j]*temp[j]; 
	}

	two_D_pt to_return;
	to_return.x = result[0]/result[2];	
	to_return.y = result[1]/result[2];	

	return to_return;
}

//pass by reference in C++
void swap(vector<three_D_pt> &threeD_world_corners, int i)
{
	three_D_pt temp;
	temp.x = threeD_world_corners[i].x; temp.y = threeD_world_corners[i].y; temp.z = threeD_world_corners[i].z;
	threeD_world_corners[i].x = threeD_world_corners[i+1].x; threeD_world_corners[i].y = threeD_world_corners[i+1].y; threeD_world_corners[i].z = threeD_world_corners[i+1].z;
	threeD_world_corners[i+1].x = temp.x; threeD_world_corners[i+1].y = temp.y; threeD_world_corners[i+1].z = temp.z;
}

//pass by reference in C++
void sort_function(vector<three_D_pt> &threeD_world_corners)
{
	int i,j;
	int n = threeD_world_corners.size();
	
	for(j=0;j<n-1;j++)
	{	
		for(i=0;i<n-j-1;i++)
		{
			if(threeD_world_corners[i].x > threeD_world_corners[i+1].x)
				swap(threeD_world_corners, i);

			else if(threeD_world_corners[i].x == threeD_world_corners[i+1].x)
			{
				if(threeD_world_corners[i].z > threeD_world_corners[i+1].z)
					swap(threeD_world_corners, i);
			}

		}
	}
}	

//function to check whether the 3d point under consideration lies infront of the camera or towards the back of the camera
//if the point lies to the back of the camera, we will ignore it as it may hamper our door prediction model
bool is_wrong_side(line_eqn our_line, three_D_pt point)
{
	if(sin(our_line.theeta_line)*(point.y - our_line.y0) + cos(our_line.theeta_line)*(point.x - our_line.x0) <= 0)
		return 1;
	else 
		return 0;
}

int main()
{
	//To get from the world to robot coordinate frame
	//The translation vector and Rotation matrix for world to Robot coordinate frame is - trw(3*1), Rrw(3*3)
	//The translation vector and Rotation matrix for Robot to Camera coordinate frame is - tcr(3*1), Rcr(3*3)

	//from Camera Calibration, we got the values of fx, fy, cx, cy(file with these values: "The_camera_calibration_matrix" )
	double fx = 735.677; double cx = 236.822; double fy = 733.533; double cy = 254.936;

	double K[3][4] = { {fx, 0, cx, 0}, {0, fy, cy, 0}, {0, 0, 1, 0} };//the Intrinsic matrix
	
	//double Twr[3][1] = { {-x0*cos(theeta) + y0*sin(theeta)}, {x0*sin(theeta)-y0*cos(theeta)}, {0} };
	double x0, y0;//we get the robot pose in the world coordinate frame as an input
	

//x0, y0, theeta are the parameters we can play with to obtain different views of the doors of the hallway 
///////////////////////////////////////
	x0 = 2*1000; y0 = 5*1000;	
	double pi_val = pi;
	double theeta = pi_val*0;
///////////////////////////////////////



	line_eqn our_line;
	our_line.x0 = x0;
	our_line.y0 = y0;
	our_line.theeta_line = theeta;
	//Hence now, Mwr is completely defined
    double Twr[3] = { -x0*cos(theeta) - y0*sin(theeta), x0*sin(theeta)-y0*cos(theeta), 0 };
	//coordinates of the world coordinate origin in terms of the robot coordinate frame
	double Mwr[4][4] = { {cos(theeta), sin(theeta), 0, Twr[0]} , {-sin(theeta), cos(theeta), 0, Twr[1]} , {0, 0, 1, Twr[2]} , {0, 0, 0, 1} };
	//keeps on changing with time

	
	double tx, ty, tz;
	tx = tz = 0;
	ty = 5*1000;
	//Hence now, Mrc is completely defined since we have assumed the orientation of the camera to the robot base frame in such a way
	//double Trc[3][1] = { {tx}, {ty}, {tz} };//this is fixed and known to us
	//this is the translation vector(coordinates of the robot coordinate origin in terms of the Camera coordinate frame)
	double Mrc[4][4] = { {0,-1,0,tx}, {0,0,-1,ty}, {1,0,0,tz}, {0,0,0,1} };

	double P[3][4];
	matMul3(K,Mrc,Mwr,P);
	
	vector<three_D_pt> threeD_world_corners;//the list of cornerss of the door is known
	//All the doors have the same height from the floor
	three_D_pt corner_temp;

	
	double half_door_width = 500;
	double half_door_height = 1000;
		










	//////////////////////////////////////////////////////SAMPLE DOOR POINTS START HERE/////////////////////////////////////////////////////////


	corner_temp = {0.5*1000-half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {0.5*1000+half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {0.5*1000-half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {0.5*1000+half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);


	corner_temp = {2*1000-half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {2*1000+half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {2*1000-half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {2*1000+half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {6*1000-half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {6*1000+half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {6*1000-half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {6*1000+half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {8*1000-half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {8*1000+half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {8*1000-half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {8*1000+half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {12*1000-half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {12*1000+half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {12*1000-half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {12*1000+half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);




	corner_temp = {15*1000-half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {15*1000+half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {15*1000-half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {15*1000+half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);





		corner_temp = {18*1000-half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {18*1000+half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {18*1000-half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {18*1000+half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);


	

		corner_temp = {20*1000-half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {20*1000+half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {20*1000-half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {20*1000+half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



		corner_temp = {25*1000-half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {25*1000+half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {25*1000-half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {25*1000+half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



		corner_temp = {30*1000-half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {30*1000+half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {30*1000-half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {30*1000+half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {35*1000-half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {35*1000+half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {35*1000-half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {35*1000+half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {40*1000-half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {40*1000+half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {40*1000-half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {40*1000+half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {45*1000-half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {45*1000+half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {45*1000-half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {45*1000+half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {50*1000-half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {50*1000+half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {50*1000-half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {50*1000+half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {58*1000-half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {58*1000+half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {58*1000-half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {58*1000+half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {65*1000-half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {65*1000+half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {65*1000-half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {65*1000+half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {72*1000-half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {72*1000+half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {72*1000-half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {72*1000+half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {80*1000-half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {80*1000+half_door_width, 10*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {80*1000-half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {80*1000+half_door_width, 10*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	corner_temp = {0.5*1000-half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {0.5*1000+half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {0.5*1000-half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {0.5*1000+half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {2*1000-half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {2*1000+half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {2*1000-half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {2*1000+half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {6*1000-half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {6*1000+half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {6*1000-half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {6*1000+half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {8*1000-half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {8*1000+half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {8*1000-half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {8*1000+half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {12*1000-half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {12*1000+half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {12*1000-half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {12*1000+half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);




	corner_temp = {15*1000-half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {15*1000+half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {15*1000-half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {15*1000+half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);




		corner_temp = {18*1000-half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {18*1000+half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {18*1000-half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {18*1000+half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);


	

		corner_temp = {20*1000-half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {20*1000+half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {20*1000-half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {20*1000+half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



		corner_temp = {25*1000-half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {25*1000+half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {25*1000-half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {25*1000+half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



		corner_temp = {30*1000-half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {30*1000+half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {30*1000-half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {30*1000+half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);


	corner_temp = {35*1000-half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {35*1000+half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {35*1000-half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {35*1000+half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {40*1000-half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {40*1000+half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {40*1000-half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {40*1000+half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {45*1000-half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {45*1000+half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {45*1000-half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {45*1000+half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {50*1000-half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {50*1000+half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {50*1000-half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {50*1000+half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {58*1000-half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {58*1000+half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {58*1000-half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {58*1000+half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {65*1000-half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {65*1000+half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {65*1000-half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {65*1000+half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {72*1000-half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {72*1000+half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {72*1000-half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {72*1000+half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



	corner_temp = {80*1000-half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {80*1000+half_door_width, 0*1000, 5*1000+half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {80*1000-half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);
	
	corner_temp = {80*1000+half_door_width, 0*1000, 5*1000-half_door_height};
	threeD_world_corners.push_back(corner_temp);



/////////////////////////////////////////////////////SAMPLE DOOR POINTS END HERE/////////////////////////////////////////////////////////












	//We need to detect rectangles from the list of threeD points "threeD_world_corners"
	int n = threeD_world_corners.size();//n is 4 for this particular case
	//Assumed that the doors lie in the plane of the wall and the wall lies on the X axis
	sort_function(threeD_world_corners);
	//Now the points in sets of 4 form doors in increasing order of their x axis
	//Hence, points with increasing	x coordinates(in sets of 4) will give the doors (from the 4 corners of the doors)
	//the points are sorted in inreasing order of x coordinates and in case of tie in increasing order of their z coordinates
	cout<<"THe corners of the door are: "<<endl;
	int i;
	for(i=0;i<n;i++)
		cout<<"("<<threeD_world_corners[i].x<<","<<threeD_world_corners[i].y<<","<<threeD_world_corners[i].z<<")"<<endl;
	
	vector<two_D_pt> corners_in_image1;//vector of the door corners as points in the image
	vector<two_D_pt> corners_in_image2;
	vector<bool> is_wrong_side_in_image1;
	vector<bool> is_wrong_side_in_image2;

	bool temp_bool = 0;

	cout<<endl<<endl<<endl<<"The corners in the image are at: "<<endl;
	for(i=0;i<n;i++)
	{
		cout<<"3D Point:("<<threeD_world_corners[i].x<<","<<threeD_world_corners[i].y<<","<<threeD_world_corners[i].z<<")  ";
		two_D_pt temp = project_to_image(P,threeD_world_corners[i]);
		
		if(threeD_world_corners[i].y == 0)
		{
			if( is_wrong_side(our_line,threeD_world_corners[i]) == 1 )
			{
				temp_bool = 1;
				is_wrong_side_in_image2.push_back(temp_bool);
			}
			else if(is_wrong_side(our_line,threeD_world_corners[i]) == 0)
			{
				temp_bool = 0;
				is_wrong_side_in_image2.push_back(temp_bool);
			}
			
			corners_in_image2.push_back(temp);
			cout<<"2D point("<<corners_in_image2[i].x<<","<<corners_in_image2[i].y<<")"<<endl;
		}
		
		else if(threeD_world_corners[i].y == 10*1000)
		{
			if( is_wrong_side(our_line,threeD_world_corners[i]) == 1 )
			{
				temp_bool = 1;
				is_wrong_side_in_image1.push_back(temp_bool);
			}
			else if(is_wrong_side(our_line,threeD_world_corners[i]) == 0)
			{
				temp_bool = 0;
				is_wrong_side_in_image1.push_back(temp_bool);
			}
			
			corners_in_image1.push_back(temp);
			cout<<"2D point("<<corners_in_image1[i].x<<","<<corners_in_image1[i].y<<")"<<endl;
		}	
	}

	i=0;
	int n1 = corners_in_image1.size();//points where y coordinate is 10,000mm
	/*for(i=0;i<n1;i++)
		cout<<is_wrong_side_in_image1[i]<<" ";
	cout<<endl;*/

	i=0;
	int n2 = corners_in_image2.size();//points where y coordinate is 0mm
	/*for(i=0;i<n2;i++)
		cout<<is_wrong_side_in_image2[i]<<" ";
	cout<<endl;*/

    int no_cols = 480;
    int no_rows = 640;
    Mat imgLines = Mat::zeros( no_cols, no_rows, CV_8UC3 );
    //Mat takes (xmax, ymax){ie (cols,rows)} as argument not (rows,cols)

    //To draw the blue rectangles in the image, bounding the corners of the doors
    i=0;
    cout<<"n1(the total number of door corners with y coordinate = 10m), shown in blue color: "<<n1<<endl;
    n1 = corners_in_image1.size();//points where y coordinate is 10,000mm
    while(i<n1)//for all the (n/4) number of doors
    {
    	if( !(is_wrong_side_in_image1[i] || is_wrong_side_in_image1[i+1] || is_wrong_side_in_image1[i+2] || is_wrong_side_in_image1[i+3]))
    	{
	    	line(imgLines, Point(corners_in_image1[i].x, corners_in_image1[i].y), Point(corners_in_image1[i+1].x, corners_in_image1[i+1].y), Scalar(255,0,0), 1);
	   		line(imgLines, Point(corners_in_image1[i+1].x, corners_in_image1[i+1].y), Point(corners_in_image1[i+3].x, corners_in_image1[i+3].y), Scalar(255,0,0), 1);
	   		line(imgLines, Point(corners_in_image1[i+3].x, corners_in_image1[i+3].y), Point(corners_in_image1[i+2].x, corners_in_image1[i+2].y), Scalar(255,0,0), 1);
	   		line(imgLines, Point(corners_in_image1[i].x, corners_in_image1[i].y), Point(corners_in_image1[i+2].x, corners_in_image1[i+2].y), Scalar(255,0,0), 1);	
   		}
   		i=i+4;
    }

    i=0;
    n2 = corners_in_image2.size();//points at y=0 coordninate have been given the color red
    cout<<"n2(the total number of door corners with y coordinate = 0m), shown in red color: "<<n2<<endl;
    while(i<n2)//for all the (n/4) number of doors
    {
    	if(!(is_wrong_side_in_image2[i] || is_wrong_side_in_image2[i+1] || is_wrong_side_in_image2[i+2] || is_wrong_side_in_image2[i+3]))
    	{
	    	line(imgLines, Point(corners_in_image2[i].x, corners_in_image2[i].y), Point(corners_in_image2[i+1].x, corners_in_image2[i+1].y), Scalar(0,0,255), 1);
	   		line(imgLines, Point(corners_in_image2[i+1].x, corners_in_image2[i+1].y), Point(corners_in_image2[i+3].x, corners_in_image2[i+3].y), Scalar(0,0,255), 1);
	   		line(imgLines, Point(corners_in_image2[i+3].x, corners_in_image2[i+3].y), Point(corners_in_image2[i+2].x, corners_in_image2[i+2].y), Scalar(0,0,255), 1);
	   		line(imgLines, Point(corners_in_image2[i].x, corners_in_image2[i].y), Point(corners_in_image2[i+2].x, corners_in_image2[i+2].y), Scalar(0,0,255), 1);	
   		}
   		i=i+4;
    }
   	
   	String windowName = "Detected doors in the image";
	namedWindow(windowName);
	imshow(windowName,imgLines);

	waitKey(0);

	destroyWindow(windowName);
    return 0;
}		
