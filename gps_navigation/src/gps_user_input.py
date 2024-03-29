#!/usr/bin/python3
import rospy
import wget
from urllib.error import HTTPError
import sys
import os
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Quaternion, Twist, TwistStamped, PoseStamped, PointStamped, Vector3
import pandas as pd
import csv
from std_msgs.msg import String
from std_srvs.srv import SetBool
from sensor_msgs.msg import NavSatFix
from microstrain_inertial_msgs.msg import FilterHeading

import actionlib
from pxrf.msg import TakeMeasurementAction, TakeMeasurementGoal, TakeMeasurementResult

# add pxrf's plot script to lookup path
import rospkg
rospack = rospkg.RosPack()
pxrf_path = rospack.get_path('pxrf')
sys.path.insert(0, os.path.abspath(os.path.join(pxrf_path, "scripts")))

from plot import generate_plot
from gps_user_location import read_location

#testing
lat_set = 0
lon_set = 0
zoom_set = 0
width_set = 0
height_set = 0
#gps_fidelity = 1.0


class MeasurementMarker(pg.GraphicsObject):
    def __init__(self, x, y, parent=None):
        super().__init__(parent)
        size = 50
        self._rect = QtCore.QRectF(x - size // 2, y - size // 2, size, size)
        self.picture = QtGui.QPicture()
        self._generate_picture()

    @property
    def rect(self):
        return self._rect

    def _generate_picture(self):
        painter = QtGui.QPainter(self.picture)
        painter.setPen(pg.mkPen("w"))
        painter.setBrush(pg.mkBrush("g"))
        painter.drawRect(self.rect)
        painter.end()

    def paint(self, painter, option, widget=None):
        painter.drawPicture(0, 0, self.picture)

    def boundingRect(self):
        return QtCore.QRectF(self.picture.boundingRect())


def deg2num(lat_deg: float, lon_deg: float, zoom: int):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = (lon_deg + 180.0) / 360.0 * n
    ytile = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n
    return (xtile, ytile)

def num2deg(xtile: float, ytile: float, zoom: int):
    n = 2.0 ** zoom
    lon_deg = xtile / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
    lat_deg = math.degrees(lat_rad)
    return (lat_deg, lon_deg)

class TileMap:
    TILE_SIZE_PX = 512

    def __init__(self, coord=(0,0), dim=(0,0), zoom=0):
        self.load_tiles(coord, dim, zoom)

    def load_tiles(self, coord, dim, zoom: int):
        lon: float = coord[0]
        lat: float = coord[1]
        width: int = dim[0]
        height: int = dim[1] 
        print(f'Window at Lon: {lon}, Lat: {lat}, W: {width}, H: {height}')

        x, y = deg2num(lon, lat, zoom)
        x = int(x)
        y = int(y)
        print(f'Tile X,Y: {x}, {y}')

        x_start = x - width
        x_end = x + width + 1
        y_start = y - height
        y_end = y + height + 1

        print(f'X from {x_start} to {x_end}')
        print(f'Y from {y_start} to {y_end}')

        #xs = np.arange(-width, width+1) + x
        #ys = np.arange(-height, height+1) + y

        completeTile = None
        #for i in range(len(xs)):
        for i in range(x_start, x_end):
            verticalTile = None
            #for j in range(len(ys)):
            for j in range(y_start, y_end):
                tile = self.getMapTile(zoom, int(i), int(j))
                verticalTile = tile if verticalTile is None else np.concatenate((verticalTile, tile), axis=0)
            completeTile = verticalTile if completeTile is None else np.concatenate((completeTile, verticalTile), axis=1)

        print('\r')
        self.map_array = completeTile
        self.x = x_start
        self.y = y_start
        self.zoom = zoom

    def pixel2Coord(self, pix):
        x = float(pix[0]) / self.TILE_SIZE_PX + self.x
        y = float(pix[1]) / self.TILE_SIZE_PX + self.y
        return num2deg(x, y, self.zoom)

    def coord2Pixel(self, lat: float, lon: float):
        x,y = deg2num(lat, lon, self.zoom)
        x = round(self.TILE_SIZE_PX * (x-self.x))
        y = round(self.TILE_SIZE_PX * (y-self.y))
        return x, y
        
    def getMapTile(self, zoom: int, x: int, y: int):
        mapDirectory = os.path.join(
            os.path.abspath(os.path.dirname(__file__)),
            'gps_navigation_satellite_tiles')

        if not os.path.isdir(mapDirectory):
            os.mkdir(mapDirectory)

        mapFile = os.path.join(mapDirectory, f'satellite_{zoom}_{x}_{y}.jpeg')

        if not os.path.isfile(mapFile):
            print(f"\rDownloading missing map tile x:{x} y:{y} zoom:{zoom}", end='')
            access_token = "pk.eyJ1Ijoic2Vhbmp3YW5nIiwiYSI6ImNrb2c3d2M5bjBhcHcyb2xsd2VyNXdkNnEifQ.5uaSXmSX1HdSAlEf4LReNg"
            mapLink = f"https://api.mapbox.com/styles/v1/mapbox/satellite-v9/tiles/512/{zoom}/{x}/{y}?access_token={access_token}"

            try:
                wget.download(mapLink, out=mapFile)
            except HTTPError:
                print(f'Bad Url: {mapLink}')
        tile = np.asarray(mpimg.imread(mapFile))
        return tile


class PlotWithClick(pg.PlotItem):

    def mouseClickEvent(self, ev):
        xClick = self.getViewBox().mapSceneToView(ev.scenePos()).x()
        yClick = self.getViewBox().mapSceneToView(ev.scenePos()).y()
        for handler in self.click_handlers:
            handler([xClick, yClick])


class PolyLineROI_noHover(pg.PolyLineROI):
    def hoverEvent(self, ev):
        pass




class GpsNavigationGui:
    def __init__(self, lat, lon, zoom, width, height):
        self.prev_lat = 0
        self.prev_lon = 0
        self.prev_heading = 0

        #pxrf control
        self.pxrfRunning = False

        #speed display
        self.highSpeed = True

        #get the map
        self.satMap = TileMap(coord=(lat, lon), dim=(width, height), zoom=zoom)

        # init widget
        pg.setConfigOption('imageAxisOrder', 'row-major')
        self.widget = pg.LayoutWidget()

        # add plot for map
        satGUI = pg.GraphicsLayoutWidget()
        self.widget.addWidget(satGUI, row=0, col=0, colspan=8)
        satGUI.setBackground('w')
        self.click_plot = PlotWithClick()
        satGUI.addItem(self.click_plot)

        # show satellite image
        img = pg.ImageItem(self.satMap.map_array)
        img.setBorder({'color': 'b', 'width': 3})
        self.click_plot.addItem(img)
        self.click_plot.showAxes(True)
        self.click_plot.setAspectLocked()
        self.click_plot.invertY()

        with open('measurement_locations.csv') as f:
            reader = csv.reader(f)
            self.num_measurements = 0
            for row in reader:
                self.num_measurements += 1
                label = row[0]
                lat = float(row[1])
                lon = float(row[2])
                self.add_marker_at(lat, lon, label=label)

        #with open('coords_wgs84.csv') as f:
        #    reader = csv.reader(f)
        #    for row in reader:
        #        label = row[0]
        #        x = float(row[1])
        #        y = float(row[2])
        #        self.add_marker_at(x, y, label=label)

        # add arrow to show robot
        self.robotArrow = pg.ArrowItem(headLen=40, tipAngle=30, brush='r')
        self.click_plot.addItem(self.robotArrow)

        # init ROI for editing path
        self.pathRoi = PolyLineROI_noHover([], closed=False)

        # init waypoints display
        self.click_plot.addItem(self.pathRoi)

        # init plot for path
        self.pathPlotPoints: list[list] = []
        self.pathGPS = []
        self.heading = 0
        self.pathPlot = self.click_plot.plot(symbolBrush=(255, 0, 255))
        self.currentGoalMarker = self.click_plot.plot(symbolBrush=(0, 0, 255))
        self.pathIndex = 0

        # interaction for adding path points
        self.click_plot.click_handlers = [self.addROIPoint]

        # robot's history (path measured by gps)
        self.historyPoints: list[list[int]] = []
        self.historyPlot = self.click_plot.plot(pen=pg.mkPen('g', width=3))
        self.setHistory()

        # ros services
        self.parking_brake = rospy.ServiceProxy('/parking_brake', SetBool)

        # add buttons
        self.setup_widgets()

        # ros action clients
        self.pxrf_client = actionlib.SimpleActionClient('/take_measurement', TakeMeasurementAction)

        # ros sub pub
        #self.navigation_sub = rospy.Subscriber('/gps_navigation/current_goal', PoseStamped, self.readNavigation) # get status of navigation controller
        self.goal_pub = rospy.Publisher('/gps_navigation/goal', PoseStamped, queue_size=5)

        self.location_sub = rospy.Subscriber('/gnss1/fix', NavSatFix, self.on_gps_update)
        #self.heading_sub = rospy.Subscriber('/nav/heading', FilterHeading, self.on_heading_update)
        self.odom_sub = rospy.Subscriber('/nav/odom_throttle', Odometry, self.on_odom_update) # plotRobotPosition

    def add_marker_at(self, lat: float, lon: float, label=None, size=20):
        pos = self.satMap.coord2Pixel(lat, lon)

        marker = pg.TargetItem(
            pos=pos,
            movable=False,
            size=size,
            label=label)

        self.click_plot.addItem(marker)

    def setup_widgets(self):

        def prevGoal():
            self.changeGoal(changeDir=-1)

        def nextGoal():
            self.changeGoal(changeDir=1)

        def clearHistory():
            self.setHistory(clear = True)

        clearHistoryBtn = QtWidgets.QPushButton('Clear History')
        clearHistoryBtn.clicked.connect(clearHistory)
        clearPathBtn = QtWidgets.QPushButton('Clear Path')
        clearPathBtn.clicked.connect(self.clearPath)
        self.editPathBtn = QtWidgets.QPushButton('Edit Path')
        self.editPathMode = False
        self.editPathBtn.clicked.connect(self.toggleEditPathMode)
        loadPathFileBtn = QtWidgets.QPushButton('Load Path')
        loadPathFileBtn.clicked.connect(self.loadPathFile)
        savePathBtn = QtWidgets.QPushButton('Save Path')
        savePathBtn.clicked.connect(self.savePath)
        self.startPauseBtn = QtWidgets.QPushButton('Start Navigation')
        self.is_navigating = False
        self.startPauseBtn.clicked.connect(self.startPause)
        prevGoalBtn = QtWidgets.QPushButton('Prev Goal')
        prevGoalBtn.clicked.connect(prevGoal)
        nextGoalBtn = QtWidgets.QPushButton('Next Goal')
        nextGoalBtn.clicked.connect(nextGoal)
        
        self.stopStatus = True
        self.parking_brake(self.stopStatus)
        self.parkBtn = QtWidgets.QPushButton('PARK ON')
        self.parkBtn.setStyleSheet("background-color : red")
        self.parkBtn.clicked.connect(self.toggle_brake)
        self.widget.addWidget(self.parkBtn, row=3, col=6, colspan=2)

        self.pxrfStatus = False
        self.pxrfBtn = QtWidgets.QPushButton('Sample')
        self.pxrfBtn.clicked.connect(self.toggle_pxrf_collection)
        self.widget.addWidget(self.pxrfBtn, row=2, col=7, colspan=1)

        #text widget
        self.statusGPS = QtWidgets.QLineEdit()
        self.statusGPS.setText('GPS Connecting')
        self.statusGPS.setReadOnly(True)
        
        self.statusNav = QtWidgets.QLineEdit()
        self.statusNav.setText('Manual Mode')
        self.statusNav.setReadOnly(True)
        
        self.statusPxrf = QtWidgets.QLineEdit()
        self.statusPxrf.setText('Ready to collect')
        self.statusPxrf.setReadOnly(True)

        #self.status = pg.TextItem('')
        #self.status.setColor(pg.Qt.QtGui.QColor("red"))
        #self.status.setText("testing")
        #self.widget.addWidget(self.status, row = 1, col = 1)
        self.widget.addWidget(clearHistoryBtn,    row=1, col=3)
        self.widget.addWidget(clearPathBtn,       row=2, col=0)
        self.widget.addWidget(self.editPathBtn,   row=2, col=1)
        self.widget.addWidget(loadPathFileBtn,    row=2, col=2)
        self.widget.addWidget(savePathBtn,        row=2, col=3)
        self.widget.addWidget(self.startPauseBtn, row=2, col=5, colspan=1)
        self.widget.addWidget(prevGoalBtn,        row=1, col=6)
        self.widget.addWidget(nextGoalBtn,        row=2, col=6)
        self.widget.addWidget(self.statusGPS,     row=1, col=0, colspan=3)
        self.widget.addWidget(self.statusNav,     row=1, col=5, colspan=1)
        #self.widget.addWidget(self.statusSpeed, row=1, col=6, colspan=1)
        self.widget.addWidget(self.statusPxrf,    row=1, col=7, colspan=1)
        #self.widget.addLabel(text = "Mode: "+ self.statusNav, row = 1, col = 0, colspan = 2 )
        #self.widget.addLabel(text = "GPS: " + self.statusGPS, row = 1, col = 3, colspan = 2 )
        #self.widget.addLabel(text = "PXRF status: " + self.statusPxrf,  row = 1,  col = 5, colspan = 2)

    def on_pxrf_measurement_complete(self, status, result: TakeMeasurementResult):
        print(f'pxrf cb result: {result.result.data}')
        self.pxrfRunning = False
        self.pxrfBtn.setText('Sample')
        self.statusPxrf.setText("Ready to collect")
        self.pxrfStatus = False

        self.num_measurements += 1
        with open('measurement_locations.csv', 'a') as f:
            f.write(f'Sample{self.num_measurements},{self.prev_lat},{self.prev_lon}\n')

        self.add_marker_at(self.prev_lat, self.prev_lon, f'Sample#{self.num_measurements}')

        if result.result.data == "201":
            generate_plot()

    #This function sets the heading of the robot
    def on_heading_update(self, data: FilterHeading):
        #if abs(self.prev_heading - self.heading) < 0.5:
        #    self.heading = data.heading_rad
        #else:
        #    self.heading = self.prev_heading
        if(data.heading_rad == 0 and data.heading_deg == 0):
            return
        print('heading update')
        self.heading = data.heading_rad 

        #self.prev_heading = self. heading

    # This function adds points to roi (when user is editing path)
    def addROIPoint(self, point):
        if self.editPathMode:
            print('click: Add Point')
            points = [[handle['pos'].x(),handle['pos'].y()] for handle in self.pathRoi.handles]

            points.append(point)
            self.pathRoi.setPoints(points)
    
    # This function converts the current path from gps coordinates to pixels
    def gps_to_pixels(self):
        self.pathPlotPoints = []
        for gpsPoint in self.pathGPS:
            lat = gpsPoint[0]
            lon = gpsPoint[1]
            point = list(self.satMap.coord2Pixel(lat, lon))
            self.pathPlotPoints.append(point)

    # This fuction converts the current path from pixels to gps coordinates
    def pixels_to_gps(self, pathPlotPoints):
        pathGPS = []
        for point in pathPlotPoints:
            gpsPoint = list(self.satMap.pixel2Coord(point))
            gpsPoint.append(1)
            pathGPS.append(gpsPoint)
        
        return pathGPS
    
    # This function clears robot's history path
    def setHistory(self, clear=False):
        if clear:
            self.historyPoints=[]
        
        if len(self.historyPoints) == 0:
            x = []
            y = []
        else:
            x, y = zip(*self.historyPoints)

        self.historyPlot.setData(x=list(x), y=list(y))
    
    # This function clears the current path
    def clearPath(self):
        self.pathRoi.setPoints([])
        self.pathPlotPoints= []
        self.pathGPS = []
        self.pathPlot.setData(x=[], y=[])
        self.updateGoalMarker()
        self.is_navigating = False
        self.startPauseBtn.setText('start navigation')

    # This function turns on/off editing mode
    def toggleEditPathMode(self):
        self.editPathMode = not self.editPathMode
        if self.editPathMode:
            self.editPathBtn.setText('Apply')
            self.pathRoi.setPoints([])
            for point in self.pathPlotPoints:
                self.addROIPoint(point)
        else:
            self.editPathBtn.setText('Edit Path')
            self.pathPlotPoints = []
            for handle in self.pathRoi.handles:
                pos = handle['pos']
                self.pathPlotPoints.append([pos.x(), pos.y()])
            self.pathGPS = self.pixels_to_gps(self.pathPlotPoints)
            x, y = zip(*self.pathPlotPoints)
            self.pathPlot.setData(x=list(x), y=list(y))
            self.pathRoi.setPoints([])
            self.updateGoalMarker()
    
    # This function loads path from csv file chosen by user
    def loadPathFile(self):
        fn = str(QtGui.QFileDialog.getOpenFileName()[0])
        #pathFn = easygui.fileopenbox()
        if fn =='':
            return
        if self.editPathMode:
            self.toggleEditPathMode()
        self.clearPath()
        csvData = pd.read_csv(fn)
        csvData = np.array(csvData.iloc[:,:])
        for i in range(csvData.shape[0]):
            toStop = True
            if csvData.shape[1]>2:
                toStop = bool(csvData[i,2])
            gpsPoint = [csvData[i,0],csvData[i,1],toStop]
            self.pathGPS.append(gpsPoint)
        self.gps_to_pixels()
        self.pathPlot.setData(x=[point[0] for point in self.pathPlotPoints],y=[point[1] for point in self.pathPlotPoints])
        self.changeGoal(reset=True)

    # this function saves the current path as csv file
    def savePath(self):
        fn = str(QtGui.QFileDialog.getSaveFileName()[0])
        csvFile = open(fn,'w',newline='')
        csvWriter = csv.writer(csvFile,delimiter=',')
        csvWriter.writerow(['lat','lon','stop'])
        for gpsPoint in self.pathGPS:
            csvWriter.writerow(gpsPoint)

    # This function starts/ pauses the navigation
    def startPause(self):
        if (len(self.pathGPS) == 0):
            return
        self.is_navigating = not self.is_navigating
        if self.is_navigating:
            self.startPauseBtn.setText('Pause Navigation')
        elif self.pathIndex == 0:
            self.startPauseBtn.setText('Start Navigation')
        else:
            self.startPauseBtn.setText('Continue Navigation')

    # This function changes the goal to another waypoint on path
    def changeGoal(self,reset=False,changeDir=1):
        self.pathIndex = self.pathIndex + changeDir
        if self.pathIndex < 0:
            self.pathIndex = max(self.pathIndex+len(self.pathPlotPoints), 0)
        if reset or self.pathIndex >= len(self.pathPlotPoints):
            self.pathIndex = 0
            if self.is_navigating:
                self.startPause()
        self.updateGoalMarker()

    # This function updates the marked goal on the path
    def updateGoalMarker(self):
        if self.pathIndex < len(self.pathPlotPoints):
            point = self.pathPlotPoints[self.pathIndex]
            self.currentGoalMarker.setData(x=[point[0]], y=[point[1]])
        else:
            self.currentGoalMarker.setData(x=[], y=[])

    # This function is called by subscriber of gps sensor
    def on_odom_update(self,data: Odometry):
        print('Incoming Odom')
        lat = data.pose.pose.position.x
        lon = data.pose.pose.position.y

        #calculate heading based on gps coordinates 
        pixX, pixY = self.satMap.coord2Pixel(lat, lon)
        #print(f"GPS -> pixels ({lat}, {lon}) -> {pixX}, {pixY}")
        prevpixX, prevpixY = self.satMap.coord2Pixel(self.prev_lat, self.prev_lon)
        robotHeading = self.heading
        #print("robotHeading "+ str(robotHeading))
        quat = data.pose.pose.orientation
        r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        #xVec = r.as_dcm()[:,0]
        #robotHeading = np.mod(math.atan2(xVec[1],xVec[0])+np.pi/3.0,2*np.pi)
        #robotHeading = self.heading #math.atan2(xVec[0],xVec[1])

        if not self.robotArrow is None:
            self.robotArrow.setStyle(angle = robotHeading*180.0/np.pi + 90.0)
            self.robotArrow.setPos(pixX, pixY)
            self.robotArrow.update()

        self.historyPoints.append([pixX, pixY])
        self.setHistory()
        self.prev_lat = lat
        self.prev_lon = lon
        self.prev_heading = robotHeading
    
    # This function updates the value of longitude and latitude information
    def on_gps_update(self, data: NavSatFix):
        if(data.longitude != 0.0 and data.latitude != 0.0):
            self.longitude = data.longitude
            self.latitude = data.latitude
            self.statusGPS.setText("lon: " + str(round(self.longitude,4)) + " " +"lat: " + str(round(self.latitude, 4)) )
        else:
            self.statusGPS.setText("GPS connecting")

    #this function checks status of navigation controller
    def readNavigation(self,data: PoseStamped):
        print('Incoming nav update')
        if self.is_navigating:
            self.statusNav.setText("Automatic navigation")
            currNavGoal = np.array([data.pose.position.y, data.pose.position.x])
            desNavGoal = np.array(self.pathGPS[self.pathIndex][0:2])
            onCurrentGoal = np.linalg.norm(desNavGoal-currNavGoal) < 1e-4
            navFinished = data.pose.position.z < 0
            if onCurrentGoal and navFinished:
                if self.pathGPS[self.pathIndex][2]:
                    self.startPause()
                self.changeGoal()
            else:
                msg = PoseStamped()
                msg.pose.position.y = desNavGoal[0]
                msg.pose.position.x = desNavGoal[1]
                self.goal_pub.publish(msg)
        else:
            # stop the navigation
            self.statusNav.setText("Manual")
            msg = PoseStamped()
            msg.pose.position.x = float('nan')
            msg.pose.position.y = float('nan')
            msg.pose.position.z = -1
            self.goal_pub.publish(msg)

    def toggle_brake(self):
        self.stopStatus = not self.stopStatus
        self.is_navigating = 0

        self.parking_brake(self.stopStatus)
        if self.stopStatus:
            self.parkBtn.setText('PARK ON')
            self.parkBtn.setStyleSheet("background-color : red")
        else:
            self.parkBtn.setText('PARK OFF')
            self.parkBtn.setStyleSheet("background-color : green")

    def toggle_pxrf_collection(self):
        self.pxrfStatus = not self.pxrfStatus
        if self.pxrfStatus:
            self.statusPxrf.setText("Collecting")
            self.pxrfRunning = True
            self.pxrfBtn.setText("STOP pxrf")
            #self.pubCTRL.publish("start")
            goal = TakeMeasurementGoal()
            self.pxrf_client.send_goal(goal, done_cb=self.on_pxrf_measurement_complete)
        else:
            self.statusPxrf.setText("Ready to collect")
            self.pxrfRunning = False
            self.pxrfBtn.setText("Sample")
            #self.pubCTRL.publish("stop")
            self.pxrf_client.cancel_all_goals()


if __name__ == '__main__':
    rospy.init_node('gps_user_input',anonymous=True)

    try:
       lat = float(rospy.get_param('~lat'))
       lon = float(rospy.get_param('~lon'))
       zoom = int(rospy.get_param('~zoom'))
       height = int(rospy.get_param('~height'))
       width = int(rospy.get_param('~width'))
    except:
        location_input = read_location()

        lat, lon = [float(n) for n in location_input[1:3]]
        zoom = int(location_input[3])
        width, height = [int(n) for n in location_input[4:6]]

    app = QtWidgets.QApplication([])
    mw = QtWidgets.QMainWindow()
    print(f'{lat}, {lon}, {width}, {height}')
    gps_node = GpsNavigationGui(lat, lon, zoom, width, height)
    mw.setCentralWidget(gps_node.widget)
    mw.show()
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtWidgets.QApplication.instance().exec_()
