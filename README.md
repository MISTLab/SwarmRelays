# SwarmRelays
This is the official code repository for project Swarm Relays.
## SwarmRelays
This is the official code repository for project Swarm Relays.
(This is only a restricted set of the code, we will keep updating the complete code). 
If you use the code from this project please cite: 

```
@ARTICLE{Varadharajan2020,
  author={V. S. {Varadharajan} and D. {St-Onge} and B. {Adams} and G. {Beltrame}},
  journal={IEEE Robotics and Automation Letters}, 
  title={Swarm Relays: Distributed Self-Healing Ground-and-Air Connectivity Chains}, 
  year={2020},
  volume={},
  number={},
  pages={1-1},}

```
The following instructions assume the user is running a Debian OS (e.g. Ubuntu)
# Dependencies 
The Open Motion Planning Library (OMPL)
OMPL has the following required dependencies:
Boost (version 1.58 or higher)
CMake (version 3.5 or higher)
Eigen (version 3.3 or higher)
# Installing OMPL
```
$ git clone https://github.com/ompl/ompl.git
$ mkdir -p build/Release
$ cd build/Release
$ cmake ../..
# next step is optional
$ make -j 4 update_bindings # if you want Python bindings
$ make -j 4 # replace "4" with the number of cores on your machine
```
ARGoS3 Simulator
ARGoS3 simulator can also be installed from binaries please refer to the official website for more information: https://www.argos-sim.info/
 
The instructions below are for installing ARGoS3 from its source.

Official code repository: https://github.com/ilpincy/argos3

Dependencies for ARGoS3 can be installed using the following command:
```
sudo apt-get install cmake libfreeimage-dev libfreeimageplus-dev \
  qt5-default freeglut3-dev libxi-dev libxmu-dev liblua5.3-dev \
  lua5.3 doxygen graphviz graphviz-dev asciidoc
```
# Installing ARGoS3
```
$ git clone https://github.com/ilpincy/argos3.git argos3
$ cd argos3
$ mkdir build_simulator
$ cd build_simulator
$ cmake ../src
$ make
$ sudo make install
```
This code repository has three libraries that have to be built and a Buzz virtual machine (BVM) to use the provided buzz source (buzz/connectivity_planning.bzz).
Building and installing BVM:
The official website for Buzz: https://the.swarming.buzz/ 
Official code repository: https://github.com/MISTLab/Buzz
Buzz is a programming language for heterogeneous robot swarms.
We recommend using the BVM sources located in this repository. 
To install Buzz (once inside the repository folder):
```
$ cd Buzz
$ mkdir build
$ cd build
$ cmake ../src
$ sudo make install
$ sudo ldconfig
```
Building and installing the KheperaIV plugin for ARGoS3
```
$ git clone https://github.com/ilpincy/argos3-kheperaiv.git
$ mkdir build_sim
$ cd build_sim
$ cmake -DCMAKE_BUILD_TYPE=Release ../src
$ make
$ sudo make install
``` 
Building the controller:
This controller contains additional buzz functions that call the rrt* planner from OMPL. 
```
# you should now be in the repository folder. 
$ cd Hooks_src
$ mkdir build
$ cmake ..
$ make 
```
Building the loop function that loads the map into ARGoS: 
```
$ cd ..
# you should now be in the repository folder. 
$ cd Loop_fun_src
$ mkdir build
$ cmake ..
$ make
```
Once all the previous steps have been done. 

Compile the Buzz script using the following command: 
```
$ cd .. 
# you should now be in the repository folder. 
$ cd buzz_scripts
$ bzzc Connectivity_planning.bzz
```
# Launching the experiment 
```
$ cd ..
# you should now be in the repository folder.
$ argos3 -c Swarm_relay_demo.argos
```
The experiment at time step 103 generates an SVG file in the root repository folder, from which the path generated for building the communication chain can be visualized. The SVG file writer was adapted from https://github.com/olegsinyavskiy/sparse_rrt

# Changing the map file 
There are several map files included in this repository obtained from https://www.movingai.com/benchmarks/ 

TODO
