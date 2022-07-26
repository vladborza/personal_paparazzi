#!/usr/bin/python3
import sys
from PyQt5 import QtCore, QtWidgets, QtGui
import mark_tracker_core
import mark_tracker_new
sys.path.append("/home/vladborza/paparazzi/sw/ground_segment/python/mark_tracker")
import argparse

def main():
  app = QtWidgets.QApplication(sys.argv)
  MainWindow = QtWidgets.QMainWindow()
  tracker = mark_tracker_new.Tracker(verbose=True)
  app.aboutToQuit.connect(tracker.closing)
  tracker.setupUi(MainWindow)
  tracker.built()
  MainWindow.show()
  sys.exit(app.exec_())

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description="Interface Mark tracking at IMAV2019")
  main()
