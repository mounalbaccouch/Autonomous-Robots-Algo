#include <iostream>
#include <fstream>      
#include <math.h>
#include <vector>
#include <string>
#include <sstream>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace std;

// Sensor characteristic: Min and Max ranges of the beams
double Zmax = 5000, Zmin = 170;
// Defining free cells(lfree), occupied cells(locc), unknown cells(l0) log odds values
double l0 = 0, locc = 0.4, lfree = -0.4;
// Grid dimensions
double gridWidth = 100, gridHeight = 100;
// Map dimensions
double mapWidth = 30000, mapHeight = 15000;
// Robot size with respect to the map 
double robotXOffset = mapWidth / 5, robotYOffset = mapHeight / 3;
// Defining an l vector to store the log odds values of each cell
vector< vector<double> > l(mapWidth/gridWidth, vector<double>(mapHeight/gridHeight));

double inverseSensorModel(double x, double y, double theta, double xi, double yi, double sensorData[])
{
    //******************Code the Inverse Sensor Model Algorithm**********************//
    // Defining Sensor Characteristics
    double Zk, thetaK, sensorTheta;
    double minDelta = -1;
    double alpha = 200, beta = 20;
    double cell;

    //******************Compute r and phi**********************//
    double r = sqrt(pow(xi - x, 2) + pow(yi - y, 2));
    double phi = atan2(yi - y, xi - x) - theta;

    //Scaling Measurement to [-90 -37.5 -22.5 -7.5 7.5 22.5 37.5 90]
    for (int i = 0; i < 8; i++) {
        if (i == 0) {
            sensorTheta = -90 * (M_PI / 180);
        }
        else if (i == 1) {
            sensorTheta = -37.5 * (M_PI / 180);
        }
        else if (i == 6) {
            sensorTheta = 37.5 * (M_PI / 180);
        }
        else if (i == 7) {
            sensorTheta = 90 * (M_PI / 180);
        }
        else {
            sensorTheta = (-37.5 + (i - 1) * 15) * (M_PI / 180);
        }

        if (fabs(phi - sensorTheta) < minDelta || minDelta == -1) {
            Zk = sensorData[i];
            thetaK = sensorTheta;
            minDelta = fabs(phi - sensorTheta);
        }
    }

    //******************Evaluate the three cases**********************//
    if (r > min((double)Zmax, Zk + alpha / 2) || fabs(phi - thetaK) > beta / 2 || Zk > Zmax || Zk < Zmin) {
        cell = l0;
    }
    else if (Zk < Zmax && fabs(r - Zk) < alpha / 2) {
        cell = locc;
    }
    else if (r <= Zk) {
        cell = lfree;
    }
    
    return cell;
}

void occupancyGridMapping(double Robotx, double Roboty, double Robottheta, double sensorData[])
{
    //1- Generate a grid (size 300x150) and then loop through all the cells
    for(int x=0 ; x<mapWidth/gridWidth ; x++){
    	for(int y=0 ; y<mapHeight/gridHeight ; y++){
    		//2- Compute the center of mass of each cell xi and yi 
    		double xi = x * gridWidth + gridWidth / 2 - robotXOffset;
    		double yi = -(y * gridHeight + gridHeight / 2) + robotYOffset;
    		//3- Check if each cell falls under the perceptual field of the measurements
    		if (sqrt(pow(xi - Robotx, 2) + pow(yi - Roboty, 2)) <= Zmax){
               	l[x][y] = l[x][y] + inverseSensorModel(Robotx, Roboty, Robottheta, xi, yi, sensorData) - l0;
            	}
    	}
    }
}

void visualization()
{
    plt::title("Map");
    plt::xlim(0, (int)(mapWidth / gridWidth));
    plt::ylim(0, (int)(mapHeight / gridHeight));

    // Draw every grid of the map:
    for (double x = 0; x < mapWidth / gridWidth; x++) {
        cout << "Remaining Rows= " << mapWidth / gridWidth - x << endl;
        for (double y = 0; y < mapHeight / gridHeight; y++) {
            if (l[x][y] == 0) { //Green unkown state
                plt::plot({ x }, { y }, "g.");
            }
            else if (l[x][y] > 0) { //Black occupied state
                plt::plot({ x }, { y }, "k.");
            }
            else { //Red free state
                plt::plot({ x }, { y }, "r.");
            }
        }
    }
    plt::save("./Images/map.png");
    plt::clf();
}

int main()
{
    string line;
    double timeStamp;
    double measurementData[8];
    double robotX, robotY, robotTheta;
    ifstream posesFile("poses.txt");
    ifstream measurementFile("measurement.txt");

    // Scanning the files and retrieving measurement and poses at each timestamp
    if(posesFile.is_open() && measurementFile.is_open()){
        while(posesFile){
            // Reading poses
            getline(posesFile, line, '\n');
            istringstream ss1(line);
            ss1 >> timeStamp >> robotX >> robotY >> robotTheta;
            // Reading measurements data
            getline(measurementFile, line, '\n');
            istringstream ss2(line);
            ss2 >> timeStamp >> measurementData[0] >> measurementData[1] >> measurementData[2] >> measurementData[3] >> 				measurementData[4] >> measurementData[5] >> measurementData[6] >> measurementData[7] ;
	    occupancyGridMapping(robotX, robotY, (robotTheta / 10) * (M_PI / 180), measurementData);
    	}
    }else
    	cout << "Can't open files \n";
    	
    // Visualize the map
    visualization();
    
    return 0;
}

