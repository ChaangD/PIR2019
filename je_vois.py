#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
    jeVois module
    --------------

    Use it to connect to the camera and get information while handling errors
    Needed packages: sudo apt install python-serial

    .. see also::   droneListener which will get these JeVois's informations and process it in order to know the position of the platform in the pseudo-body frame

""" 

"""

:param arg1: description
:param arg2: description
:type arg1: type
:type arg1: type
:return: description de la valeur de retour
:rtype: type de la valeur de retour

:Example:

Un exemple écrit après un saut de ligne.

.. seealso:: Référence à une autre partie du code
.. warning:: Avertissement
.. note:: Note
.. todo:: A faire

"""
 
import sys
import serial
import time
import numpy as np

# TODO Changer apres que le probleme de base soit fixe
# ~/paparazzi_bug/var/lib/python
# paparazzi/sw/ext/pprzlink/lib/v2.0/python 
# rotorcraftfp pour phi teta psi mais latence 

sys.path.append("/home/delairra/paparazzi/sw/ext/pprzlink/lib/v2.0/python")

# sys.path.append("../../codeDeRefDansPprz/")

from pprzlink.message import PprzMessage
from pprzlink.ivy import IvyMessagesInterface

from math import *

# JeVois cam sends information (when placed with A33 on the top)
# D3 id x y z w h d q1 q2 q3 q4 extra
# with  - x looking on the right
#       - y             bottom
#       - z is position in depth


class ArUcoTag:
    """ Define a tag object """
    def __init__(self, id):
        self.initialized = False         # To know if it has seen at least once the tag
        self.id = id                    # U43 == id of ArUco tag
        self.X = np.zeros(3)            # State vector
        self.quaternions = np.zeros(4)  
        self.whd = np.zeros(3)          # Width height depth
        self.timeLastUpdate = -1            # time in s of last updating time 



class JeVoisListenerIvy(object):

    def __init__(self, my_ac_id, tag_id, interface_):

        # Start a ivy messages listener named PIR DRONE
        self._interface = interface_
        self.tag = ArUcoTag(tag_id)
        self.ac_id = my_ac_id

        # bind to jevois message
        def jevois_cb(jevois_id, msg):
        
            # print("entree dans callback : {} {}".format(self.tag.id , str(msg['id'])))
            if self.tag.id == str(msg['id']).replace('"', ""):
                self.tag.X[0] = float(msg['coord'][0]) / 1000 # data is sent in milimeter , stocked in meter
                self.tag.X[1] = float(msg['coord'][1]) / 1000
                self.tag.X[2] = float(msg['coord'][2]) / 1000
                self.tag.quaternions[0] = float(msg['quat'][0])
                self.tag.quaternions[1] = float(msg['quat'][1])
                self.tag.quaternions[2] = float(msg['quat'][2])
                self.tag.quaternions[3] = float(msg['quat'][3])

                self.tag.whd[0] = float(msg['dim'][0])
                self.tag.whd[1] = float(msg['dim'][1])
                self.tag.whd[2] = float(msg['dim'][2])

                self.tag.initialized = True
                self.tag.timeLastUpdate = time.time()
                
        self._interface.subscribe(jevois_cb, PprzMessage("telemetry", "JEVOIS"))



class JeVoisListenerGround:
    """
        Handle communication with jeVois.
        Use it to get the position of the platform as seen by the camera
    """
    def __init__(self, ):
        self.tag = ArUcoTag(42)
        self.droneUpdater = DroneControler()
        self.serdev = '/dev/ttyACM0' # serial device of JeVois

        self.ser = None

        # time_before = time.time()
        
        # timeout while time.time() - time_before < 5:

        self.start_serial()

        # Send info : out = USB and we want normal infors
        self.send_command('setpar serout USB')       # Hard to use 4pin series

        self.send_command('setmapping2 YUYV 320 240 30.0 JeVois DemoArUco')
        self.send_command('streamon')

        # Send info : out = USB and we want normal infors
        self.send_command('setpar serstyle Detail')  # Terse for less inforamtion
        self.send_command('setpar serprec 2')
        self.send_command('setpar markerlen 60')
        self.send_command('setpar dopose true')
        
    def start_serial(self):
        """
            try to reconnect to the device

            .. todo:: Ajouter un timer pour ne pas attendre jusquau bout de la nuit
        """
        connection_established = False
        while not connection_established:
            print "Trying to restart"
            try:
                self.ser = serial.Serial(self.serdev, 115200, timeout=1)
            except IOError:
                print "error in serial connection"
                time.sleep(0.2)
                connection_established = False
                break
            connection_established = True
            print "Restarted"

    def send_command(self, cmd):
        """ Send a command to the device using the serial port """
        if self.ser is not None:
            print "HOST>> " + cmd
            self.ser.write(cmd + '\n')
            out = ''
            time.sleep(0.1)
            while self.ser.inWaiting() > 0:
               out += self.ser.read(1)
            if out != '':
                print "JEVOIS>> " + out, # the final comma suppresses extra newline, since JeVois already sends one
        else:
            self.start_serial()
            self.send_command(cmd)


    def update(self):

        try:
            # Read a whole line and strip any trailing line ending character:
            line = self.ser.readline().rstrip()
        except:
            print "PB in line reading"
            self.start_serial()
            return

        # Split the line into tokens:
        tok = line.split()

        # From now on, we hence expect: D3 id x y z w h d q1 q2 q3 q4 extra
        if len(tok) < 1: 
            print "TIMEOUT"
            return

        # Skip if not a standardized "Detail 3D" message:
        if tok[0] != 'D3': 
            print "pas un message standardised D3"
            return
 
        # From now on, we hence expect: D3 id x y z w h d q1 q2 q3 q4 extra
        if len(tok) < 12: 
            print "Pas assez d'infos"
            return

        # Assign some named Python variables to the tokens:
        # Without key (because we dont need it)
        key = tok[0]
        id = tok[1]
        x_cam, y_cam, z_cam, w_cam, h_cam, d_cam, q1_cam, q2_cam, q3_cam, q4_cam = tuple([float(tok[i]) for i in range(len(tok)) if i > 1])
 
        # quat = np.quaternion(q1_cam, q2_cam, q3_cam, q4_cam)
        # phi, teta, psi = quaternion.as_euler_angles(quat)

        self.tag.X[0] = x_cam
        self.tag.X[1] = y_cam
        self.tag.X[2] = z_cam

        self.tag.quaternions[0] = q1_cam
        self.tag.quaternions[1] = q2_cam
        self.tag.quaternions[2] = q3_cam
        self.tag.quaternions[3] = q4_cam

        self.tag.whd[0] = w_cam
        self.tag.whd[1] = h_cam
        self.tag.whd[2] = d_cam

        self.tag.initialized = True
        self.tag.time_since_seen = time.time()








