import matplotlib.pyplot as plt
import numpy as np
import math
import json
from bresenham import bresenham
import time

# CONTROL PARAMETERS
BOX_SIZE = 0.1
ROOM_SIZE = 15
PROB_HIT = 0.9
PROB_MISS = 0.4


class Cols:
    HEADER = '\033[34m'
    INFO = '\033[92m'
    FAILED = '\033[91m'
    ENDC = '\033[0m'
    
def print_header(message):
    print(f"{Cols.HEADER}"+message+f"{Cols.ENDC}")

def print_info(message):
    print(f"{Cols.INFO}"+message+f"{Cols.ENDC}")

def print_failed(message):
    print(f"{Cols.FAILED}"+message+f"{Cols.ENDC}")



class Converter:
    def __init__(self):
        self.pi = 3.141592653589793

    def polar2cart(self, rho, phi):
        x = rho * math.cos(phi)
        y = rho * math.sin(phi)
        return [x, y]
    

class Robot:
    def __init__(self):
        self.laserRange = 10
        self.laserShift = 0.1

    def transform_kinematic(self, globalPos, angle, dist):
        newPos = np.array([1,1,0], dtype=np.float)
        newPos[0] = dist * math.cos(globalPos[2] + angle) + globalPos[0]
        newPos[1] = dist * math.sin(globalPos[2] + angle) + globalPos[1]
        newPos[2] = globalPos[2]
        return newPos


class OccupancyGridMap(Robot):
    def __init__(self):
        super().__init__()
        self.boxSize = BOX_SIZE
        self.roomSize = ROOM_SIZE
        self.xSize = int(self.roomSize / self.boxSize)
        self.ySize = int(self.roomSize / self.boxSize)      
        self.probHit = PROB_HIT
        self.probMiss = PROB_MISS
        self.thHit = 1
        self.thMiss = -1
        self.fieldMap = np.zeros((self.xSize, self.ySize))
        self.robotPathX = []
        self.robotPathY = []
        self.fig = None
        self.mapFig = None

    def add_point_to_path(self, x, y):
        self.robotPathX.append(x)
        self.robotPathY.append(y)
    
    def return_robot_path(self):
        return [self.robotPathX, self.robotPathY]

    def glob2coord(self, x, y):
        coords = np.array([0,0], dtype=np.int)
        coords[0] = int( (x/self.boxSize) + self.xSize/2 )
        coords[1] = int( (y/self.boxSize) + self.ySize/2 )
        return coords

    def global_pos_of_box(self, box):
        pos = np.array([0,0], dtype=np.float)
        pos[0] = (box[0] - self.xSize/2) * self.boxSize
        pos[1] = (box[1] - self.ySize/2) * self.boxSize
        return pos
    
    def hit(self, field):
        field = field + np.log(self.probHit/(1-self.probHit))
        if field > self.thHit:
            field = self.thHit
        return field

    def miss(self, field):
        field = field + np.log(self.probMiss/(1-self.probMiss))
        if field < self.thMiss:
            field = self.thMiss
        return field

    def dist_point_to_line(self, A, B, C, xPoint, yPoint):
        return np.absolute(A * xPoint + B * yPoint + C) / np.sqrt(A*A + B*B)

            
    def get_missed_fields(self, startPoint, endPoint):
        startPoint = self.glob2coord(startPoint[0], startPoint[1])
        endPoint = self.glob2coord(endPoint[0], endPoint[1])
        x1 = startPoint[0]
        y1 = startPoint[1]
        x2 = endPoint[0]
        y2 = endPoint[1]

        dx = x2 - x1
        dy = y2 - y1

        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)
    
        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
    
        # Swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
    
        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1
    
        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1
    
        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
    
        # Reverse the list if the coordinates were swapped
        if swapped:
            points.reverse()
        return points


    def update_map(self, pose, scan):
        robotPose = np.array([0,0,0], dtype=np.float)
        for i in range(3):
            robotPose[i] = pose[i]
        robotPose[2] = robotPose[2] / 180 * np.pi
        sensorPose = Robot.transform_kinematic(robotPose, 0, Robot.laserShift)

        x = np.arange(0, len(scan))
        angles = (np.pi/len(scan))*x - np.pi/2

        for i in x:
            if np.isinf(scan[i]) or np.isnan(scan[i]):
                continue
            else:
                targetPoint = Robot.transform_kinematic(sensorPose, angles[i], scan[i])
                targetCoord = self.glob2coord(targetPoint[0], targetPoint[1])
                x = int(targetCoord[0])
                y = int(targetCoord[1])

                if x>=int(self.roomSize / self.boxSize) or y>=int(self.roomSize / self.boxSize):
                    print_failed("Error: Room size is too small to plot the element x: {} y: {}. \n Please, increate the room space...".format(x,y))
                else:
                    self.fieldMap[y][x] = self.hit(self.fieldMap[y][x])
                    missed1 = self.get_missed_fields(sensorPose, targetPoint)                

                    for box in missed1:
                        self.fieldMap[box[1]][box[0]] = self.miss(self.fieldMap[box[1]][box[0]])

        return self.fieldMap

    def compute_prob_map(self):
        return 1 - (1/(1 + np.exp(self.fieldMap)))	

    def initial_plot(self):
        self.fig = plt.figure()
        self.fieldMap[0][0] = 0.01
        # self.mapFig = plt.imshow(self.computeProbMap(), interpolation="nearest", cmap='Blues', vmin=0, vmax=1, origin='lower')
        self.mapFig = plt.imshow(self.compute_prob_map(), interpolation="nearest", cmap='Blues', origin='lower')
        plt.colorbar()

    def update_plot(self):
        self.mapFig.set_data(self.compute_prob_map())
        plt.plot(self.return_robot_path()[0], self.return_robot_path()[1], 'r-', linewidth=1)
        plt.show(block=False)
        plt.pause(0.1)
    
    def show_plot(self):
        plt.show()

if __name__ == '__main__':
    print_header("\n### PROGRAM STARTED ###")
    start_time = time.time()

    try:
        file_path = './data/ex1.json'
        json_file = open(file_path)
        data = json.load(json_file) 
        print_info(str(file_path) + " file loaded...")
        print_info("Data lenght: " + str(len(data)))
        FILE_LOADED = True
    except Exception as ex:
        print_failed(str(file_path) + " file loading FAILED")

    if FILE_LOADED:
        OGM = OccupancyGridMap()
        
        Robot = Robot()
        Conv = Converter()

        for i in range(0, len(data)):
            print_info("  Iteration: " + str(i))
            scan = data[i]['scan']
            pose = data[i]['pose']

            robotPose = OGM.glob2coord(pose[0], pose[1])
            OGM.add_point_to_path(robotPose[0], robotPose[1])
            fm = OGM.update_map(pose, scan)
            if i == 0:
                OGM.initial_plot()
            else:
                OGM.update_plot()

        print_info("--- {} s ---".format(round((time.time() - start_time), 3)))
        OGM.show_plot()

    print_header("### PROGRAM FINISHED ###")
