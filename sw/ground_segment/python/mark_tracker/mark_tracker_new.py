from tkinter import Y
from PyQt5 import QtCore, QtWidgets, QtGui
from collections import namedtuple
from ui.mainwindow_tracker_new.form import Ui_MarkTracker
import traceback
from time import sleep
import numpy as np
from math import cos, sqrt, pi

import sys
from os import path, getenv

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))

sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.message import PprzMessage
from pprz_connect import PprzConnect, PprzConfig

import lxml.etree as ET

#only two types of markers, one vector type with all the markers and one variable holding the correct tag value
#dynamic vector which appends new positions of ArUco tags

MARK_S1 = 1
MARK_S2 = 2
MARK_S3 = 3
MARK_S4 = 4
mark_types = {
    MARK_S1: {'id':1, 'name':'Known location'},
    MARK_S2: {'id':2, 'name':'100m GPS Spoof'},
    MARK_S3: {'id':3, 'name':'Unknown field location'},  
    MARK_S4: {'id':4, 'name':'Silent Deilivery'}
}

class Mark():
    '''
    Store mark information
    '''
    def __init__(self, _id, _name):
        self.id = _id
        self.name = _name
        self.lat = 0.
        self.lon = 0.
        self.alt = 0.
        self.lat_arr = np.array([])
        self.lon_arr = np.array([])

    def set_pos(self, lat, lon, alt):
        ''' set pos '''
        # filtering
        self.lat_arr = np.append(self.lat_arr, [lat])
        self.lon_arr = np.append(self.lon_arr, [lon])
        self.lat = np.mean(self.lat_arr)
        self.lon = np.mean(self.lon_arr)
        self.alt = alt

    def clear(self):
        self.lat_arr = np.array([])
        self.lon_arr = np.array([])

    def __str__(self):
        out = 'Mark {} with ID {} at pos {}, {}'.format(self.name, self.id, self.lat, self.lon)
        return out

class Tracker(Ui_MarkTracker):
    '''
    Main tracker class
    '''  
    def __init__(self, parent=None, verbose=False):
        Ui_MarkTracker.__init__(self)
        
        self.verbose = verbose
        self.marks_fpl = {}
        self.marks_by_name = {}
        
        for k, e in mark_types.items():
            self.marks_fpl[k] = Mark(k, e['name'])
            self.marks_by_name[e['name']] = k
        for i in self.marks_fpl:      
            print(i, " ", self.marks_fpl[i])
        
        self.id_list = []
        #print(mark_types)
        #print(self.marks)
        self.uavs = {}
        self.alt_ref = 0

    def built(self):
        global wps_counter
        wps_counter = 0
        for i in self.marks_fpl: 
            hist = str(i) + " " + str(self.marks_fpl[i])
            self.commands.append(hist)
        
        
        ''' HMI callbacks '''
        self.active_uav.currentIndexChanged.connect(self.uav_selected)
        self.clear_s1.clicked.connect(lambda:self.clear_mark(MARK_S1))
        self.clear_s2.clicked.connect(lambda:self.clear_mark(MARK_S2))
        self.clear_s3.clicked.connect(lambda:self.clear_mark(MARK_S3))
        self.clear_s4.clicked.connect(lambda:self.clear_mark(MARK_S4))
        self.send_s1.clicked.connect(lambda:self.send_mark(MARK_S1))
        self.send_s2.clicked.connect(lambda:self.send_mark(MARK_S2))
        self.send_s3.clicked.connect(lambda:self.send_mark(MARK_S3))
        self.send_s4.clicked.connect(lambda:self.send_mark(MARK_S4))
        
        ''' get aircraft config '''
        #POPULATE COMBO LIST WITH flight plan waypoints
        #USE THIS TO POPULATE LIST the same way for each mission
        def connect_cb(conf):
            global conf2 
            conf2 = conf

            active_wps = []
            try: 
                xml = ET.parse(conf.flight_plan)
                fp = xml.find('flight_plan')
                hist = str(fp.find('waypoints')) + "\n"
                self.commands.append(hist)
                print(fp.find('waypoints'))

                self.alt_ref = fp.get("alt")
                wps = fp.find('waypoints')
                if wps is not None:
                    self.uavs[conf.name] = []
                    for wp in wps.iter("waypoint"):
                        self.uavs[conf.name].append(wp.get("name"))
                        active_wps.append(wp.get("name"))
                        #print(wp.get("name"))
            except (IOError, ET.XMLSyntaxError) as e:
                hist = 'XML error' + e.__str__()
                self.commands.append(hist)
                print('XML error',e.__str__())
            if self.verbose:
                hist = str(conf)
                self.commands.append(hist)
                print(conf)

            self.active_uav.addItem(conf.name)

            hist = str(self.uavs[conf.name])
            self.commands.append(hist)
            print(self.uavs[conf.name], " ################################### ")

        ''' create connect object, it will start Ivy interface '''
        self.connect = PprzConnect(notify=connect_cb)

        self.search_s3.clicked.connect(self.search_tag_s3)
        self.input_tag_number.returnPressed.connect(self.search_tag_s3)
        #self.input_tag_number.enterEvent.connect(self.search_tag_s3)

        ''' bind to MARK message '''
        def mark_cb(ac_id, msg):
            global conf2
            global correct_id, wps_counter
            mark_id = int(msg['ac_id']) # abuse ac_id field
            if(mark_id is not None):
                if(mark_id not in self.id_list):
                    self.id_list.append(mark_id)
                    hist = "NEW MARKER DETECTED!"
                    self.commands.append(hist)

                    if self.verbose:
                        hist = 'From ' + str(ac_id) + ': ' + str(msg)
                        self.commands.append(hist)
                        print('from ',ac_id,':',msg)
                    # update mark and send shape
                    print(mark_id, "bind")
                    lat = float(msg['lat'])
                    lon = float(msg['long'])

                    if(int(mark_id) == int(correct_id)):
                        # print("WAYPOINT TO BE MOVED: ", self.uavs[conf2.name])
                        # new_mark = Mark(mark_id,"S3")
                        # new_mark.set_pos(lat,lon,self.alt_ref)
                        # self.update_pos_label(new_mark)
                        if self.combo_s3_wps.findText(str(Mark(mark_id,"Found "))):
                            self.combo_s3_wps.addItem("Found " + str(Mark(mark_id,"")))
                        mark2_name = self.uavs[conf2.name][wps_counter]
                        mark2 = Mark(mark_id,mark2_name)
                        mark2.set_pos(lat, lon, self.alt_ref)
                        print(mark2_name, " ", lat, " ", lon)
                        self.move_wp(ac_id, wps_counter,mark2)

                        wps_counter += 1    
                        hist = "THIS IS THE CORRECT MARKER MOVED AS: " + str(mark2_name)
                        self.commands.append(hist)
                        #send ivy message with correct mark_id ???
                        self.connect.ivy.send(str(self.marks_fpl[MARK_S3]))

                        # self.update_shape(self.marks_fpl[mark_id])
                        self.update_shape(mark2)

                        if self.checkBox_auto_send.checkState == True:
                            self.send_mark(mark_id)

                        # populate active waypoints combo box
                        # self.combo_s3_wps.addItem(mark_id)
                    else:
                        mark2_name = self.uavs[conf2.name][wps_counter]
                        mark2 = Mark(mark_id,mark2_name)
                        mark2.set_pos(lat, lon, self.alt_ref)
                        print(mark2_name, " ", lat, " ", lon)
                        self.move_wp(ac_id, wps_counter,mark2)
                        
                        wps_counter += 1
                        hist = "This is a wrong marker moved as: " + str(mark2_name)
                        self.commands.append(hist)
                        # CHECK IF POS = ONE of the MARKER's positions (to know if it's mission 1 or 4)
                        # else - check if pos is around 100m of a certain position defined to know if it's mission 2
                        #populate active waypoints combo box
                        # print("POTENTIAL WAYPOINT: ", self.uavs[conf2.name])
                        new_mark = Mark(mark_id,"")
                        new_mark.set_pos(lat,lon,self.alt_ref)
                        if self.combo_s3_wps.findText(str(new_mark)):
                            self.combo_s3_wps.addItem(str(new_mark))

                        self.update_shape(mark2)
                        # if mark_id not in self.marks_fpl.keys:
                        print(self.marks_fpl)
                        # self.combo_s3_wps.addItem(mark_id)
            else:
                hist = "Marker found is not a marker type"
                self.commands.append(hist)
                print("Marker found is not a marker type")
        print("IVY")
        self.connect.ivy.subscribe(mark_cb,PprzMessage("telemetry", "MARK"))

        # self.connect.ivy.subscribe(self.search_tag_s3)

        def telemetry(ac_id, msg):
            mark_id = int(msg['ac_id'])
            wp_id = int(msg['wp_id'])
            lat = float(msg['lat'])
            lon = float(msg['long'])
            alt = float(msg['alt'])
            print(mark_id, " ", wp_id, " ", lat, " ", lon, " ", alt)
        self.connect.ivy.subscribe(telemetry,PprzMessage("ground", "MOVE_WAYPOINT"))
    
    def closing(self):
        ''' shutdown Ivy and window '''
        self.connect.shutdown()

    def search_tag_s3(self):
        global correct_id
        correct_id = self.input_tag_number.text().__str__()
        hist = "Input id: " + correct_id  + "\n"
        self.commands.append(hist)
        print(correct_id)
        return correct_id 
    
    def uav_selected(self, i):
        ''' update WP list when changing the selected UAV '''
        if self.verbose:
            hist = 'Selected ' + str(i)
            self.commands.append(hist)
            print('selected',i)
        wps = self.uavs[self.active_uav.currentText()]

        self.combo_s1.clear()
        self.combo_s1.addItems(wps)
        self.combo_s2.clear()
        self.combo_s2.addItems(wps)
        self.combo_s3.clear()
        self.combo_s3.addItems(wps)
        self.combo_s4.clear()
        self.combo_s4.addItems(wps)
        #print(self.marks_fpl[MARK_S1].name)
        
        try:
            self.combo_s1.setCurrentIndex(wps.index(self.marks_fpl[MARK_S1].name))
        except:
            if self.verbose:
                hist = "Mission 1 waypoint not found"
                self.commands.append(hist)
                print("Mission 1 waypoint not found")

        try:
            self.combo_s2.setCurrentIndex(wps.index(self.marks_fpl[MARK_S2].name))
        except:
            if self.verbose:
                hist = "Mission 2 waypoint not found"
                self.commands.append(hist)
                print("Mission 2 waypoint not found")

        try:
            self.combo_s3.setCurrentIndex(wps.index(self.marks_fpl[MARK_S3].name))
        except:
            if self.verbose:
                hist = "Mission 3 waypoint not found"
                self.commands.append(hist)
                print("Mission 3 waypoint not found")
        
        try:
            self.combo_s4.setCurrentIndex(wps.index(self.marks_fpl[MARK_S4].name))
        except:
            if self.verbose:
                hist = "Mission 4 waypoint not found"
                self.commands.append(hist)
                print("Mission 4 waypoint not found")

    def update_pos_label(self, mark):
        hist = "Label update" + "\n"
        self.commands.append(hist)
        if(mark.id is not None):
            if(mark.name == "S3"):
                self.pos_s3.setText("{:.7f} / {:.7f}".format(mark.lat, mark.lon))
            elif(mark.name == "S1"):
                self.pos_s1.setText("{:.7f} / {:.7f}".format(mark.lat, mark.lon))
            elif(mark.name == "S2"):
                self.pos_s2.setText("{:.7f} / {:.7f}".format(mark.lat, mark.lon))
            elif(mark.name == "S4"):
                self.pos_s4.setText("{:.7f} / {:.7f}".format(mark.lat, mark.lon))
        else:
            hist = "Mark id error"
            self.commands.append(hist)

    def clear_pos_label(self, mark):
        if(mark.id is not None):
            if(mark.name == "S1"):
                self.pos_s1.setText("lat / lon")                
            if(mark.name == "S2"):
                self.pos_s2.setText("lat / lon")    
            if(mark.name == "S3"):
                self.pos_s3.setText("lat / lon")    
            if(mark.name == "S4"):
                self.pos_s4.setText("lat / lon")           
        else:
            hist = "Mark id error"
            self.commands.append(hist)

    def get_wp_id(self, mark_id):
        ''' get WP id from mark id '''
        for i, e in mark_types.items():
            if(mark_id is not None):
                if(int(e['id']) == int(mark_id)):
                    if( i == MARK_S1):
                        hist = "MARK_S1 = " + str(mark_id)
                        self.commands.append(hist)
                        print("MARK_S1", " = ", mark_id)
                        return self.combo_s1.currentIndex() + 1
                    elif (i == MARK_S2):
                        hist = "MARK_S2 = " + str(mark_id)
                        self.commands.append(hist)
                        print("MARK_S2", " = ", mark_id)
                        return self.combo_s2.currentIndex() + 1
                    elif (i == MARK_S3):
                        hist = "MARK_S3 = " + str(mark_id)
                        self.commands.append(hist)
                        print("MARK_S3", " = ", mark_id)
                        return self.combo_s3.currentIndex() + 1                        
                    elif (i == MARK_S4):
                        hist = "MARK_S4 = " + str(mark_id)
                        self.commands.append(hist)
                        print("MARK_S4", " = ", mark_id)
                        return self.combo_s4.currentIndex() + 1
            else:
                hist = "Mark id error!"
                self.commands.append(hist)
                return None

    def send_mark(self, mark_id):
        global correct_id
        ''' send mark to selected uab cb '''
        hist = "Send mark - " + str(mark_id)
        self.commands.append(hist)
        print("Send_mark - ",mark_id)

        mark = self.marks_fpl[mark_id]
        uav_name = self.active_uav.currentText()
        wp_id = self.get_wp_id(mark_id)
        if uav_name != '':
            try:
                uav_id = self.connect.conf_by_name(uav_name).id
                self.move_wp(uav_id, wp_id, mark)
                if self.verbose:
                    hist = 'Send mark {} to UAV {} ({}), for WP {}'.format(mark.name, uav_name, uav_id, wp_id)
                    self.commands.append(hist)
                    print('Send mark {} to UAV {} ({}), for WP {}'.format(mark.name, uav_name, uav_id, wp_id))
                
                hist = "Mark id " + mark_id.__str__() + " vs Correct id " + correct_id.__str__() + "\n"
                self.commands.append(hist)

                if(int(mark_id) == int(correct_id)):
                    self.checkBox_auto_send.setCheckState(1)
                    hist = "Auto send is on"
                    self.commands.append(hist)
                    ########################################
            except Exception as e:
                if self.verbose:
                    hist = 'Send_mark error:' + e.__str__()  + "\n"
                    self.commands.append(hist)
                    print('Send_mark error:', e.__str__())

    def clear_mark(self, mark_id):
        ''' clear mark cb '''
        # print(self.marks_fpl)
        mark = self.marks_fpl[mark_id]
        mark.clear()
        self.clear_shape(mark)
        if self.verbose:
            hist = 'Clear marker - ' + mark.name + "\n"
            self.commands.append(hist)
            print('Clear marker - ', mark.name,"\n");

    def move_wp(self, ac_id, wp_id, mark):
        ''' move waypoint corresponding to a selected aircraft and mark '''
        msg = PprzMessage("ground", "MOVE_WAYPOINT")
        msg['ac_id'] = ac_id
        msg['wp_id'] = wp_id
        msg['lat'] = mark.lat
        msg['long'] = mark.lon
        msg['alt'] = mark.alt
        print("MOVING WP ", wp_id, " AC ID ", ac_id, " NAME ", mark.name)
        self.connect.ivy.send(msg)

    def update_shape(self, mark):
        ''' create or update a shape on the GCS map '''
        self.update_pos_label(mark)
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = mark.id
        msg['opacity'] = 1 # fill color
        msg['shape'] = 0 # circle
        msg['status'] = 0 # create or update
        msg['latarr'] = [int(10**7 * mark.lat),0]
        msg['lonarr'] = [int(10**7 * mark.lon),0]
        msg['radius'] = 2.
        msg['text'] = mark.name
        self.connect.ivy.send(msg)

    def clear_shape(self, mark):
        ''' delete a shape on the GCS map '''
        self.clear_pos_label(mark)
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = mark.id
        msg['opacity'] = 0 # no fill color
        msg['shape'] = 0 # circle
        msg['status'] = 1 # delete
        msg['latarr'] = [0]
        msg['lonarr'] = [0]
        msg['radius'] = 0.
        msg['text'] = 'NULL'
        self.connect.ivy.send(msg)

    def find_closest_orange(self, lat, lon):
        ''' try to find the correct orange mark based on distances '''
        def dist_ll(mark):
            ''' distances between two lat/lon position using simple pythagore '''
            if mark.nb_sample > 0:
                x = (lon - mark.lon) * cos(pi * (lat + mark.lat) / (2. * 180.))
                y = lat - mark.lat
                z = sqrt(x*x + y*y) # "distance" in degree
                d = 1852 * 60 * z # convert to meters
                return d
            else:
                return None

        mark_id = None
        min_dist = float(self.orange_threshold.sliderPosition()) # max dist to consider same mark
        for i in mark_types.keys:
            dist = dist_ll(self.marks_fpl[i]) #CHECK HERE!!!!
            if dist is not None and dist < min_dist:
                min_dist = dist # better solution
                mark_id = i
            elif dist is None and mark_id is None:
                mark_id = i # first run and empty slot
        return mark_id

