#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <stdint.h>
#include <thread>
#include <cmath>
#include<string.h>    //strlen
#include<string>  //string
#include <sstream>

#include "math.h"
#include <stdio.h>
#include<iostream>
#include<fstream>

using namespace std;

int main() {


 FILE * pFile;

 pFile = fopen ("data.txt","w");
 ifstream myReadFile;
 myReadFile.open("Training1.txt");
 int line, xx1, yy1, zz1, xx2, yy2, zz2;
 double x1, y1, z1, x2, y2, z2;
 double vect_mag1, vect_mag2, theta1q,theta1f,theta2q,theta2f,thetaBetween12;
 double magX1Y1, magX2Y2;
 int lineCounter1 = 0;
 int lineCounter2 = 0;
 int lineCounter3 = 0;
 int lineCounter4 = 0;
 int lineCounter5 = 0;
 int lineCounter6 = 0;
 int lineCounter7 = 0;
 int lineCounter8 = 0;
 int i =0;
 if (myReadFile.is_open()) {
	 while (!myReadFile.eof()) {

		myReadFile >> line;
		myReadFile >> xx1;
		myReadFile >> yy1;
		myReadFile >> zz1;
		myReadFile >> xx2;
		myReadFile >> yy2;
		myReadFile >> zz2;
		//printf("%d \t %d \t %d \t %d \t %d \t %d \t %d\n", line, xx1, yy1, zz1, xx2, yy2, zz2);

		i++;
		if(line == 1)
		{
			lineCounter1 ++;
		}
		else if(line == 2)
		{
			lineCounter2 ++;
		}
		else if(line == 3)
		{
			lineCounter3 ++;
		}
		else if(line == 4)
		{
			lineCounter4 ++;
		}
		else if(line == 5)
		{
			lineCounter5 ++;
		}
		else if(line == 6)
		{
			lineCounter6 ++;
		}
    else if(line == 7)
    {
      lineCounter7 ++;
    }
    else if(line == 8)
    {
      lineCounter8 ++;
    }

		x1 = (double)xx1;
		y1 = (double)yy1;
		z1 = (double)zz1;
		x2 = (double)xx2;
		y2 = (double)yy2;
		z2 = (double)zz2;


		vect_mag1 = sqrt(x1 * x1 + y1 * y1 + z1 * z1);
		vect_mag2 = sqrt(x2 * x2 + y2 * y2 + z2 * z2);


		magX1Y1 = sqrt(x1 * x1 + y1 * y1);
		if(x1 != 0 && magX1Y1 != 0)
		{
			theta1q = atan2(z1,magX1Y1);
			theta1f = atan2(y1,x1);
		}
		else
		{
			theta1q = 0;
			theta1f = 0;
		}

		magX2Y2 = sqrt(x2 * x2 + y2 * y2);
		if (x2 != 0 && magX2Y2 != 0)
		{
			theta2q = atan(z2/magX2Y2);
			theta2f = atan(y2/x2);
		}
		else
		{
			theta2q = 0;
			theta2f = 0;
		}

		if(vect_mag1*vect_mag2 != 0)
		{
			thetaBetween12 = acos((x1*x2 + y1*y2 + z1*z2)/(vect_mag1*vect_mag2));
		}
		else
		{
			thetaBetween12 = 0;
		}
		fprintf (pFile, "%d \t %f \t %f \t %f \t %f \t %f \t %f \t %f\n",line, vect_mag1, vect_mag2, theta1q,theta1f,theta2q,theta2f,thetaBetween12);

 }
}
printf("%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n",lineCounter1, lineCounter2, lineCounter3, lineCounter4, lineCounter5, lineCounter6, lineCounter7, lineCounter8, i);
myReadFile.close();
return 0;
}
