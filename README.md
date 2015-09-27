## 1. What is this?


This software package introduces the Recursive World Language (RWL), an interpreted language for constructing large virtual scenarios aimed at simulating datasets for mobile robotics SLAM. 

## 2. Build Dependencies
  * CMake (>=2.4)
  * MRPT (>=1.3.0)

In Ubuntu: 
`sudo apt-get install cmake libmrpt-dev`

##  3. Included programs
 * `rwl-compiler`: Parses and optionally visualize a virtual world.
 * `rwt-dataset-simulator`: Loads a virtual world, a robot trajectory description and a sensor type and generates a synthetic dataset.


## 4. Recursive World Language (RWL): Language definition 
 
See [RWL language definition](https://github.com/jlblancoc/recursive-world-toolkit/wiki/RWL-language-definition)

## 5. Available sensors 
 
See [Sensors](https://github.com/jlblancoc/recursive-world-toolkit/wiki/Sensors)


## 6. Some examples

RWL program: [world-demo1.wrl](https://github.com/jlblancoc/recursive-world-toolkit/blob/master/demos/world-demo1.rwl)

![screenshot](https://raw.githubusercontent.com/jlblancoc/recursive-world-toolkit/master/demos/world-demo1.png)

RWL program: [world-demo2.wrl](https://github.com/jlblancoc/recursive-world-toolkit/blob/master/demos/world-demo2.rwl)

![screenshot](https://raw.githubusercontent.com/jlblancoc/recursive-world-toolkit/master/demos/world-demo2.png)

RWL program: [world-demo3.wrl](https://github.com/jlblancoc/recursive-world-toolkit/blob/master/demos/world-demo3.rwl)

![screenshot](https://raw.githubusercontent.com/jlblancoc/recursive-world-toolkit/master/demos/world-demo3.png)
