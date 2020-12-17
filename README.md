# Autonomous Robots algorithms #
In this repository, you can find an implementation in C++ of the most common algorithms used in robotics.

## Building and running ##
```
git clone https://github.com/mounalbaccouch/Autonomous-Robots-Algo
cd Autonomous-Robots-Algo/build
cmake ../
make
```
In the build/ directory, you can find the generated binaries and files.

## 1- Occupancy Grid Mapping ##
This code generates a map under build/Images. This images represents the map of the environments depending on sensors measurements from measurements.txt file and robot poses from poses.txt.

Map Legend

    - Green: Unkown/Undiscovered zone
    - Red: Free zone
    - Black: Occupied zone


