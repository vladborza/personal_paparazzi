from PyQt5 import QtCore, QtWidgets, QtGui
from collections import namedtuple
from ui.mainwindow import Ui_MainWindow
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

# this could be defined in a JSON file
MARK_RED = 1
MARK_BLUE = 2
MARK_YELLOW = 3
MARK_ORANGE_1 = 4
MARK_ORANGE_2 = 5
MARK_ORANGE_3 = 6
mark_types = {
        MARK_RED: { 'name': 'RED', 'color': 'red' },
        MARK_BLUE: { 'name': 'BLUE', 'color': 'blue' },
        MARK_YELLOW: { 'name': 'YELLOW', 'color': 'yellow'},
        MARK_ORANGE_1: { 'name': 'ORANGE_1', 'color': 'orange'},
        MARK_ORANGE_2: { 'name': 'ORANGE_2', 'color': 'orange'},
        MARK_ORANGE_3: { 'name': 'ORANGE_3', 'color': 'orange'},
        }

class Mark():
    '''
    Store mark information
    '''
    def __init__(self, _id, _name, _color):
        self.id = _id
        self.name = _name
        self.color = _color
        self.nb_sample = 0
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
        self.nb_sample = self.nb_sample + 1

    def clear(self):
        self.lat_arr = np.array([])
        self.lon_arr = np.array([])
        self.nb_sample = 0

    def __str__(self):
        out = 'Mark {} with ID {} color {} at pos {}, {}'.format(self.name, self.id, self.color, self.lat, self.lon)
        return out

class Tracker(Ui_MainWindow):
    '''
    Main tracker class
    '''
  
    def __init__(self, parent=None, verbose=False):
        Ui_MainWindow.__init__(self)
        self.verbose = verbose

        self.marks = {}
        self.marks_by_name = {}
        for k, e in mark_types.items():
            self.marks[k] = Mark(k, e['name'], e['color'])
            self.marks_by_name[e['name']] = k
        self.uavs = {}
        self.alt_ref = 0

    def built(self):

        ''' HMI callbacks '''
        self.combo_uav.currentIndexChanged.connect(self.uav_selected)
        self.clear_red.clicked.connect(lambda:self.clear_mark(MARK_RED))
        self.clear_blue.clicked.connect(lambda:self.clear_mark(MARK_BLUE))
        self.clear_yellow.clicked.connect(lambda:self.clear_mark(MARK_YELLOW))
        self.clear_orange_1.clicked.connect(lambda:self.clear_mark(MARK_ORANGE_1))
        self.clear_orange_2.clicked.connect(lambda:self.clear_mark(MARK_ORANGE_2))
        self.clear_orange_3.clicked.connect(lambda:self.clear_mark(MARK_ORANGE_3))
        self.send_red.clicked.connect(lambda:self.send_mark(MARK_RED))
        self.send_blue.clicked.connect(lambda:self.send_mark(MARK_BLUE))
        self.send_yellow.clicked.connect(lambda:self.send_mark(MARK_YELLOW))

        ''' get aircraft config '''
        def connect_cb(conf):
            try:
                xml = ET.parse(conf.flight_plan)
                fp = xml.find('flight_plan')
                self.alt_ref = fp.get("alt")
                wps = fp.find('waypoints')
                if wps is not None:
                    self.uavs[conf.name] = []
                    for wp in wps.iter("waypoint"):
                        self.uavs[conf.name].append(wp.get("name"))
                        #print(wp.get("name"))
            except (IOError, ET.XMLSyntaxError) as e:
                print('XML error',e.__str__())
            if self.verbose:
                print(conf)
            self.combo_uav.addItem(conf.name)

        ''' create connect object, it will start Ivy interface '''
        self.connect = PprzConnect(notify=connect_cb)

        ''' bind to MARK message '''
        def mark_cb(ac_id, msg):
            if self.verbose:
                print('from ',ac_id,':',msg)
            # update mark and send shape
            mark_id = int(msg['ac_id']) # abuse ac_id field
            lat = float(msg['lat'])
            lon = float(msg['long'])
            # find which ORANGE mark we have found
            if mark_id in [MARK_ORANGE_1, MARK_ORANGE_2, MARK_ORANGE_3]:
                i = self.find_closest_orange(lat, lon)
                if i is not None:
                    mark_id = i
                else:
                    mark_id = None
                    if self.verbose:
                        print("Orange mark error")
            if mark_id is not None:
                # update if valid ID
                self.marks[mark_id].set_pos(lat, lon, self.alt_ref)
                self.update_shape(self.marks[mark_id])
                if self.auto_send_check.isChecked():
                    self.send_mark(mark_id)
        self.connect.ivy.subscribe(mark_cb,PprzMessage("telemetry", "MARK"))

    def closing(self):
        ''' shutdown Ivy and window '''
        self.connect.shutdown()

    def uav_selected(self, i):
        ''' update WP list when changing the selected UAV '''
        if self.verbose:
            print('selected',i)
        wps = self.uavs[self.combo_uav.currentText()]
        self.combo_wp_red.clear()
        self.combo_wp_red.addItems(wps)
        try:
            self.combo_wp_red.setCurrentIndex(wps.index(self.marks[MARK_RED].name))
        except:
            if self.verbose:
                print('WP RED not found')
        self.combo_wp_blue.clear()
        self.combo_wp_blue.addItems(wps)
        try:
            self.combo_wp_blue.setCurrentIndex(wps.index(self.marks[MARK_BLUE].name))
        except:
            if self.verbose:
                print('WP BLUE not found')
        self.combo_wp_yellow.clear()
        self.combo_wp_yellow.addItems(wps)
        try:
            self.combo_wp_yellow.setCurrentIndex(wps.index(self.marks[MARK_YELLOW].name))
        except:
            if self.verbose:
                print('WP YELLOW not found')

    def update_pos_label(self, mark):
        if mark.id == MARK_RED:
            self.pos_red.setText("{:.7f} / {:.7f}".format(mark.lat, mark.lon))
            self.nb_red.setText("{}".format(mark.nb_sample))
        elif mark.id == MARK_BLUE:
            self.pos_blue.setText("{:.7f} / {:.7f}".format(mark.lat, mark.lon))
            self.nb_blue.setText("{}".format(mark.nb_sample))
        elif mark.id == MARK_YELLOW:
            self.pos_yellow.setText("{:.7f} / {:.7f}".format(mark.lat, mark.lon))
            self.nb_yellow.setText("{}".format(mark.nb_sample))
        elif mark.id == MARK_ORANGE_1:
            self.pos_orange_1.setText("{:.7f} / {:.7f}".format(mark.lat, mark.lon))
            self.nb_orange_1.setText("{}".format(mark.nb_sample))
        elif mark.id == MARK_ORANGE_2:
            self.pos_orange_2.setText("{:.7f} / {:.7f}".format(mark.lat, mark.lon))
            self.nb_orange_2.setText("{}".format(mark.nb_sample))
        elif mark.id == MARK_ORANGE_3:
            self.pos_orange_3.setText("{:.7f} / {:.7f}".format(mark.lat, mark.lon))
            self.nb_orange_3.setText("{}".format(mark.nb_sample))

    def clear_pos_label(self, mark):
        if mark.id == MARK_RED:
            self.pos_red.setText("lat / lon")
            self.nb_red.setText("{}".format(mark.nb_sample))
        elif mark.id == MARK_BLUE:
            self.pos_blue.setText("lat / lon")
            self.nb_blue.setText("{}".format(mark.nb_sample))
        elif mark.id == MARK_YELLOW:
            self.pos_yellow.setText("lat / lon")
            self.nb_yellow.setText("{}".format(mark.nb_sample))
        elif mark.id == MARK_ORANGE_1:
            self.pos_orange_1.setText("lat / lon")
            self.nb_orange_1.setText("{}".format(mark.nb_sample))
        elif mark.id == MARK_ORANGE_2:
            self.pos_orange_2.setText("lat / lon")
            self.nb_orange_2.setText("{}".format(mark.nb_sample))
        elif mark.id == MARK_ORANGE_3:
            self.pos_orange_3.setText("lat / lon")
            self.nb_orange_3.setText("{}".format(mark.nb_sample))

    def get_wp_id(self, mark_id):
        ''' get WP id from mark id '''
        if mark_id == MARK_RED:
            return self.combo_wp_red.currentIndex()+1
        elif mark_id == MARK_BLUE:
            return self.combo_wp_blue.currentIndex()+1
        elif mark_id == MARK_YELLOW:
            return self.combo_wp_yellow.currentIndex()+1
        else:
            return None

    def send_mark(self, mark_id):
        ''' send mark to selected uab cb '''
        mark = self.marks[mark_id]
        uav_name = self.combo_uav.currentText()
        wp_id = self.get_wp_id(mark_id)
        if uav_name is not '':
            try:
                uav_id = self.connect.conf_by_name(uav_name).id
                self.move_wp(uav_id, wp_id, mark)
                if self.verbose:
                    print('send mark {} to uav {} ({}) for WP {}'.format(mark.name, uav_name, uav_id, wp_id))
            except Exception as e:
                if self.verbose:
                    print('send_mark error:', e.__str__())

    def clear_mark(self, mark_id):
        ''' clear mark cb '''
        mark = self.marks[mark_id]
        mark.clear()
        self.clear_shape(mark)
        if self.verbose:
            print('clear', mark.name);

    def move_wp(self, ac_id, wp_id, mark):
        ''' move waypoint corresponding to a selected aircraft and mark '''
        msg = PprzMessage("ground", "MOVE_WAYPOINT")
        msg['ac_id'] = ac_id
        msg['wp_id'] = wp_id
        msg['lat'] = mark.lat
        msg['long'] = mark.lon
        msg['alt'] = mark.alt
        self.connect.ivy.send(msg)

    def update_shape(self, mark):
        ''' create or update a shape on the GCS map '''
        self.update_pos_label(mark)
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = mark.id
        msg['linecolor'] = mark.color
        msg['fillcolor'] = mark.color
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
        msg['linecolor'] = mark.color
        msg['fillcolor'] = mark.color
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
        for i in [MARK_ORANGE_1, MARK_ORANGE_2, MARK_ORANGE_3]:
            dist = dist_ll(self.marks[i])
            if dist is not None and dist < min_dist:
                min_dist = dist # better solution
                mark_id = i
            elif dist is None and mark_id is None:
                mark_id = i # first run and empty slot
        return mark_id

