# ALMG 
[ITSC 2024] Detection and Classification of Driving Maneuver Events for Automatic Lane Map Generation

This repository contains our implementation of **detection and classification of driving maneuver events** and its application to **Lanelet2-based ego-lane map generation**.

By utilizing vehicle sensor measurements (GNSS, INS, lane detection, video footage), we reconstruct vehicle states and segment the trajectory into multiple arc segments. These segments, which represent parts of the vehicle's path, are then classified into six distinct driving maneuver classes.

- Lane Keeping Maneuver (LKM)
- Right Turn (RT)
- Left Turn (LT)
- Right Lane Change (RLC)
- Left Lane Change (LLC)
- Roundabout (RA)

Based on the classified driving maneuvers, the vehicle data is automatically processed for Lanelet2 map generation.

For a detailed understanding of our work, please refer to our paper.
* Paper link will be provided after the final submission.


## Getting Started
Below are the required toolbox/libraries to run our simplified demos. If you just want to view the processed maps and figures, move to [pre-computed results](#results)
You have to install all of the following in order to run the main script.
* Tested on Ubuntu 20.04 (focal)

### MATLAB
Install MATLAB R2023a or higher with following toolboxes: [Optimization Toolbox](https://kr.mathworks.com/products/optimization.html), [Statistics and Machine Learning Toolbox](https://kr.mathworks.com/products/statistics.html), [Mapping Toolbox](https://kr.mathworks.com/products/mapping.html), [Deep Learning Toolbox](https://kr.mathworks.com/products/deep-learning.html).

### Python
Install basic packages
```
sudo apt-get install python3 python3-numpy python3-scipy python3-dev pybind11-dev
```
Then, install MATLAB Engine API for Python using [Python Package Index](https://pypi.org/project/matlabengine/)

> Architecture and training data of our implementation can be found in ```ALMG/model``` folder.

### C++ (Overall)
*Step 0* : Install ROS of appropriate version and follow manual installation guide provided in in Lanelet2
```
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo apt install ros-noetic-lanelet2
```

*Step 1* : Install dependencies
```
sudo apt-get install build-essential cmake-gui
```
*Step 2* : Clone the repository
```
git clone https://github.com/wjswlsghks98/ALMG.git ALMG
```
*Step 3* : Initialize Git Submodules
```
cd ALMG
git submodule init
git submodule update
```
*Step 4* : Create build directory
```
mkdir build
```
*Step 5* : Configure build and generate Makefiles
```
cd build && cmake ..
```
*Step 6* : Build Code
```
make -j
```

> If there are errors regarding MATLAB C++ API, refer to this [link](https://kr.mathworks.com/help/matlab/calling-matlab-engine-from-cpp-programs.html?s_tid=CRUX_lftnav).

## Results
The automatically generated map files are saved under the ```ALMG/OSM``` directory. There are two methods for viewing the mapping results. 

1. **JOSM**

   For the OpenStreetMap(OSM) files, download [JOSM](https://josm.openstreetmap.de/wiki/Download) and additional stylesheets provided in the Lanelet2 [repository](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master/lanelet2_maps). Then, open the ```.osm``` file you want and add satellite image layer for visualization. 

2. **MATLAB**

   This is a faster and easier way than using JOSM. Change the file path and run ```test.m```. The figure is automatically saved in the directory. Note that you must have Mapping Toolbox installed for using ```geoplot.m```. 


## References
If you use our implementation in your own work, please cite our [papers](https://www.sciencedirect.com/science/article/pii/S0167839624000979):
```
@inproceedings{Jeon2024ALMG,
title = {Detection and Classification of Driving Maneuver Events for Automatic Lane Map Generation},
author = {Jinhwan Jeon and Chaeho Lim and Yoonjin Hwang and Seibum B. Choi},
booktitle = {Proc.\ IEEE Intell.\ Trans.\ Syst.\ Conf.},
year      = {2024},
address   = {Edmonton, Canada},
owner     = {jeon},
month     = {September},
Url={**Will be added after proceedings**}
}

@article{Jeon2024ArcSplineApprox,
title = {Reliability-based G1 continuous arc spline approximation},
journal = {Computer Aided Geometric Design},
volume = {112},
pages = {102363},
year = {2024},
issn = {0167-8396},
doi = {https://doi.org/10.1016/j.cagd.2024.102363},
url = {https://www.sciencedirect.com/science/article/pii/S0167839624000979},
author = {Jinhwan Jeon and Yoonjin Hwang and Seibum B. Choi}
}
```
and the following paper of [Lanelet2](https://ieeexplore.ieee.org/document/8569929):
 
```
@inproceedings{poggenhans2018lanelet2,
  title     = {Lanelet2: A High-Definition Map Framework for the Future of Automated Driving},
  author    = {Poggenhans, Fabian and Pauls, Jan-Hendrik and Janosovits, Johannes and Orf, Stefan and Naumann, Maximilian and Kuhnt, Florian and Mayr, Matthias},
  booktitle = {Proc.\ IEEE Intell.\ Trans.\ Syst.\ Conf.},
  year      = {2018},
  address   = {Hawaii, USA},
  owner     = {poggenhans},
  month     = {November},
  Url={http://www.mrt.kit.edu/z/publ/download/2018/Poggenhans2018Lanelet2.pdf}
}
```
and also the work of [Tsintotas et al.](https://ieeexplore.ieee.org/abstract/document/8461146) for Loop Closure Detection.

```
@inproceedings{tsintotas2018places,
  title={Assigning Visual Words to Places for Loop Closure Detection},  
  author={K. A. Tsintotas and L. Bampis and A. Gasteratos},   
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  pages={5979 - 5985},
  year={2018},   
  month={May},
  address={Brisbane, QLD, Australia},
  doi={10.1109/ICRA.2018.8461146} 
}
```
