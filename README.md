
<!-- image -->
<div align="center" id="top"> 
  <img src=images/ex1.png width="500" />
  &#xa0;
</div>

<h1 align="center"> occupancy-grid-mapping </h1>
<h2 align="center"> Occupancy grid mapping of the robot surrounding using the laser scans
 </h2>

<!-- https://shields.io/ -->
<p align="center">
  <img alt="Top language" src="https://img.shields.io/badge/Language-Python-yellow?style=for-the-badge&logo=python">
  <img alt="Status" src="https://img.shields.io/badge/Status-done-green?style=for-the-badge">
  <img alt="Repository size" src="https://img.shields.io/github/languages/code-size/KamilGos/occupancy-grid-mapping?style=for-the-badge">
</p>

<!-- table of contents -->
<p align="center">
  <a href="#dart-about">About</a> &#xa0; | &#xa0;
  <a href="#package-content">Content</a> &#xa0; | &#xa0;
  <a href="#microscope-tests">Tests</a> &#xa0; | &#xa0;
  <a href="#checkered_flag-starting">Starting</a> &#xa0; | &#xa0;
  <a href="#eyes-implementation">Implementation</a> &#xa0; | &#xa0;
  <a href="#memo-license">License</a> &#xa0; | &#xa0;
  <a href="#technologist-author">Author</a> &#xa0; | &#xa0;
</p>

<br>


## :dart: About ##
Presented algorithm execute the standard procedure for creating occupancy grid map of the robot surrounding using the laser (eg. 360deg lidar) scans. The map is represented as a two-dimensional array. Each call is representing a square area of a given size. The scans produced by laser were used to mark occupied cells on the map. The laser was mounted on the robot in a fixed place (not in the center). One was assumed that the laser beam has a zero width. The algorithm has been developed to get the robot position from odometry, so the program could draw a map during robot motion. Also, the algorithm has been extended with options to decrease the probability in the case of the beam passing through the cell without collisions. For determining the missed fields the Bresenham's algorithm was used. 

## :package: Content
 * [mapping.py](mapping.py) - mapping algorithm for the
 * [ex1.json](data/ex1.json), [ex2.json](data/ex2.json)  - files with example data
## :microscope: Tests ##
<h2 align="left">ex1.json file </h2>
<h3> roomSize=15, boxSize=0.1, probHit=0.9, probMiss=0.4</h3>
<div align="center" id="ex1"> 
  <img src=images/ex1.png width="350" />
  &#xa0;
</div>

---

<h2 align="left">ex2.json file </h2>
<h3> roomSize=15, boxSize=0.1, probHit=0.9, probMiss=0.4</h3>
<div align="center" id="ex2"> 
  <img src=images/ex2.png width="350" />
  &#xa0;
</div>


## :checkered_flag: Starting ##
```bash
# Clone this project
$ git clone https://github.com/KamilGos/occupancy-grid-mapping

# Access
$ cd occupancy-grid-mapping

# Run the project
$ sudo python3 mapping.py
```
## :eyes: Implementation ##
To create an occupancy grid map we need to acquire robot positions from the odometry. Then those positions are added to the relevant path (red line on figures) and together with the data from laser sensor they are used to update the map. Update process consists of kinematic transformations and updates of specific fields. Locations of detected objects are determined on the base of lidar measurements, transformed from polar to global coordinate system, and with help Hit&Miss algorithm with **probHit** parameter. All the locations between the object and the robot are calculated by Bresenhams algorithm. Then, the probability value of each field returned by the algorithm is updated by Hit&Miss algorithm but this time with **probMiss** parameter. This procedure is then repeated through all iterations.

The algorithm required several parameters to work: box size, room size, probability of hit and probability of miss. 

* The map is represented as two-dimensional square array with the size (xSize*ySize). roomSize represents size of the area in which the robot operates and the boxSize represents the density of squares in total area. The bigger the value, the lowerresolution of the room is generated and the algorithm require more time to execute.
<div align="center" id="put_id"> 
  <img src=images/eq1.png width="200" />
  &#xa0;
</div>

* Each value of the box on the map represents the probability of an obstacle. The value equal to 0.5 is related to lack of knowledge about the field. Every value higher than 0.5 means, that there is higher probability of obstacle in this filed, and if the value is lower than 0.5, the field is assumed empty. The algorithm update each filed when new sensor reading appears. If the obstacle is detected, then the value is increasing logarithmically:<center>value = value + log(probMiss / 1-probMiss)</center>Every field on the path between the robot and an obstacle is treated as empty so the probability is decreasing.<center>value = value + log(probMiss / 1-probMiss)</center>To prevent a situation that values are out of reasonable range a limit was included, setting the minimum value to 0 and maximum to 1. 

## :memo: License ##

This project is under license from MIT.

## :technologist: Author ##

Made with :heart: by <a href="https://github.com/KamilGos" target="_blank">Kamil Goś</a>

&#xa0;

<a href="#top">Back to top</a>