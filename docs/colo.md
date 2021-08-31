# LEMUR CoLo-AT

## Synopsis

CoLo-AT is the software analysis tool of CoLo using Python. It is a robotic testing environment for cooperative localization using real world dataset[1].  Compared with other popular robotic simulation environments[2], it has fewer dependencies and more convenient to add localization algorithms without worry much about the robot settings. Moreover, it is able to use both real world data to test the robustness and liabilities of these algorithms, whereas other simulation environments use only simulated data. Users can use CoLo_AT to evaluate their algorithms using the analysis results provided by CoLo-AT


## [CoLo-AT Manual](https://docs.google.com/document/d/1XOe8ZwLlM2DbLjeteLUff9cUuHIKdVW1M-U7jzvrrok/edit#)


## Features

Real world dataset 
Flexible control on the real world dataset
Easy to add and modify algorithms
Preloaded with several existing algorithms
Modularized simulation environment
Basic statistical analytic tools for algorithms



## Installation

1.  Clone this repository:

```
git clone git@git.uclalemur.com:billyskc/Simulation-Environment-for-Cooperative-Localization.git
```

## How to run the simulation environment

1. Install all dependencies

2. Create a python script for the environment(ex: test_simulation.py)

a. Create a dataset_manager with proper setting

b. Load a localization algorithm in to robots

c. Create simulation manager, recorder and analyzer in CoLo

d. Put all these part in simulation manager

3. Run CoLo
In MAC terminal:
```
python Localization_envir.py
```
![Sample Code for running CoLo]( https://git.uclalemur.com/billyskc/Simulation-Environment-for-Cooperative-Localization/raw/master/RunningCoLo.png)


## Available Algorithms:
1. LS-Cen
2. LS-CI
3. LS-BDA
4. GS-SCI
5. GS-CI



## Compatible Datasets:
CoLo Datasets: (in CoLo-D folder)
- official_dataset1
- official_dataset2
- official_dataset3
- official_dataset4


UTIAS dataset (created by the Autonomous Space Robotics Lab (ASRL) at the University of Toronto):
- MRCLAM_Dataset1
- MRCLAM_Dataset2
- MRCLAM_Dataset3
- MRCLAM_Dataset4
- MRCLAM_Dataset5
- MRCLAM_Dataset6
- MRCLAM_Dataset7
- MRCLAM_Dataset8
- MRCLAM_Dataset9

details info: [UTIAS Dataset](http://asrl.utias.utoronto.ca/datasets/mrclam/).

Note: to use UTIAS dataset, users need to create measurment_x files from measurement files using barcode_2_id.py 

## Authors
Shengkang Chen

Cade Mallett

Kyle Wong

Sagar Doshi

## Analytical Tool:
Plots:
1. Estimation deviation error vs. time
2. Trace of state variance vs. time
3. Landmark observation vs. time
4. Relative observation vs .time

Values:
1. Avg. location deviation errors
2. Avg. trace of state variance

## How to add new localization algorithm to the environment
1. Create the algorithm in the predefined algorithm framework
2. Load it to the robot before running

## Algorithm Framework
![algo_framework](https://git.uclalemur.com/billyskc/CoLo/raw/master/CoLo-AT/algo_framework.png)


[1] Leung K Y K, Halpern Y, Barfoot T D, and Liu H H T. “The UTIAS Multi-Robot Cooperative Localization and Mapping Dataset”. International Journal of Robotics Research, 30(8):969–974, July 2011

[2] Saeedi, Sajad, Michael Trentini, Mae Seto, and Howard Li. “Multiple-Robot Simultaneous Localization and Mapping: A Review.” Journal of Field Robotics 33, no. 1 (January 2016): 3–46. https://doi.org/10.1002/rob.21620.
