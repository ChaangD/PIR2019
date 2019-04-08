import sys
import time
import datetime
import numpy as np
import matplotlib.pyplot as plt
import quaternion
import math
import csv


sys.path.append("/home/delairra/paparazzi/sw/ext/pprzlink/lib/v2.0/python")
sys.path.append("~/paparazzi/var/lib/python")

from pprzlink.message import PprzMessage
from pprzlink.ivy import IvyMessagesInterface

import changement_repere as cr

from je_vois import *


#Parametres PIDs
#x
Px = 1
Ix = 0
Dx = 0
#y
Py = 1
Iy = 0
Dy = 0




class FC_Rotorcraft:
    def __init__(self, ac_id, tag_id, _interface):
        self.initialized = False
        self.id = ac_id
        self.X_opti = np.zeros(3)       # (x,y,z) dans le repere pseudo drone, source optitrack
        self.X_uwb = np.zeros(3)        # (x,y,z) dans le repere pseudo drone, source uwb
        self.X_cam = np.zeros(3)        # (x,y,z) dans le repere pseudo drone, source je vois
        self.U = np.zeros(4)            # (phi,theta,psi,V_z) dans le repere pseudo drone
        self.V_opti = np.zeros(3)       # (xdot,ydot,zdot) dans le repere pseudo drone, source optitrack
        self.V_uwb = np.zeros(3)        # (xdot,ydot,zdot) dans le repere pseudo drone, source uwb
        self.V_cam = np.zeros(3)        # (xdot,ydot,zdot) dans le repere pseudo drone, source je vois
        self.quaternions = np.zeros(4)  # (q1,q2,q3,q4) quaternions dans le repere inertiel
        self.cam_angle = -math.pi / 2   # Gives camera angle, angle between  where the camera is pointing and vector x_body, in rad
        self.timeLastUpdate = 0                # time in ms since last update of state vectors

        self._interface = _interface    # Interface Ivy given by the controller

        self.cam = JeVoisListenerIvy(ac_id, tag_id, self._interface) # Jevois listener
        # self.uwb = UWBListener(ac_id,self._interface)
     

class Platform:
    def __init__(self, id):
        self.initialized = False
        self.id = id
        self.X = np.zeros(3)
        self.angleWithInertialFrame = 0
        self.V = np.zeros(3)
        self.timeLastUpdate = 0


class DroneControler(object):

    def __init__(self, ac_id, plateform_id, tag_id):
        # Start a ivy messages listener named PIR DRONE
        self._interface = IvyMessagesInterface("PIR", ivy_bus="192.168.1:2010")

        self.drone = FC_Rotorcraft(ac_id, tag_id, self._interface)
        self.plateform = Platform(plateform_id)

        # bind to GROUND_REF message
        def ground_ref_cb(ground_id, msg):
            ac_id = int(msg['ac_id'])
            if ac_id == self.drone.id:
                # X and V in NED
                self.drone.X_opti[0] = float(msg['pos'][1])
                self.drone.X_opti[1] = float(msg['pos'][0])
                self.drone.X_opti[2] = -float(msg['pos'][2])
                self.drone.V_opti[0] = float(msg['speed'][1])
                self.drone.V_opti[1] = float(msg['speed'][0])
                self.drone.V_opti[2] = -float(msg['speed'][2])
                self.drone.quaternions[0] = float(msg['quat'][0])
                self.drone.quaternions[1] = float(msg['quat'][1])
                self.drone.quaternions[2] = float(msg['quat'][2])
                self.drone.quaternions[3] = float(msg['quat'][3])
                self.drone.timeLastUpdate = time.time()
                self.drone.initialized = True
            if ac_id == self.plateform.id:
                # X and V in NED
                self.plateform.X[0] = float(msg['pos'][1])
                self.plateform.X[1] = float(msg['pos'][0])
                self.plateform.X[2] = -float(msg['pos'][2])
                self.plateform.V[0] = float(msg['speed'][1])
                self.plateform.V[1] = float(msg['speed'][0])
                self.plateform.V[2] = -float(msg['speed'][2])
                self.plateform.quaternions[0] = float(msg['quat'][0])
                self.plateform.quaternions[1] = float(msg['quat'][1])
                self.plateform.quaternions[2] = float(msg['quat'][2])
                self.plateform.quaternions[3] = float(msg['quat'][3])
                self.plateform.timeLastUpdate = time.time()
                self.plateform.initialized = True

        self._interface.subscribe(ground_ref_cb, PprzMessage("ground", "GROUND_REF"))
        
        # Recuperation du x et y via l'UWB
        def get_xy_uwb(uwb_id, msg):
            # X and V in NED
            #msg est un tableau contenant : d1corr, d2corr, d3corr, d1vrai, d2vrai, d3vrai, x, y, z, vx, vy, vz
            x = float(msg['values'][6])
            y = float(msg['values'][7])
            z = self.drone.X_opti[3]
            psi = cr.getEulerAngles(self.drone.quaternions)[2] - cr.getEulerAngles(self.plateform.quaternions)[2]
            #self.drone.V_uwb[0] = float(msg['values'][9])
            #self.drone.V_uwb[1] = float(msg['values'][10])
            self.drone.X_uwb = cr.uwb2pseudoBody(x,y,z)

        self._interface.subscribe(get_xy_uwb, PprzMessage("telemetry", "PAYLOAD_FLOAT"))

        
    def stop(self):
        # Stop IVY interface
        print("Stopping Ivy interface")
        if self._interface is not None:
            self._interface.shutdown()
            
    def envoi_cmd(self,phi,theta,psi,throttle):
	
        msg = PprzMessage("datalink","JOYSTICK_RAW")
        msg['ac_id'] = self.drone.id
        msg['roll'] = phi
        msg['pitch'] = theta
        msg['yaw'] = psi
        msg['throttle'] = throttle
        
        self._interface.send_raw_datalink(msg)

    def envoi_cmd_guidedSetPointNED(self):
    
        msg = PprzMessage("datalink","GUIDED_SETPOINT_NED")
        msg['ac_id'] = self.drone.id
        msg['flags'] = 0b100011 # x and y (position) as offset coordinate in pseudoBody frame 

        phi, theta, psi = cr.getEulerAngles(self.drone.quaternions)
        Xtag_pseudoBody = cr.cam2pseudoBody(self.drone.cam.tag.X, phi, theta)

        msg['x'] = Xtag_pseudoBody[0]/2.3
        msg['y'] = Xtag_pseudoBody[1]/2.3
        msg['z'] = -1.3                     # COmmande en NED so the z axis is oriented facing down
        msg['yaw'] = 0
        
        print(msg)

        self._interface.send_raw_datalink(msg)


    def gen_commande(self,x,y,z,psi):
        """Entree : coordonnes x,y,z du drone dans le repere pseudo drone
            Sortie : Commande phi, theta, psi,V_z"""
        pidx = pid.PID(Px,Ix,Dx)
        pidy = pid.PID(Py,Iy,Dy)

        phi = pidx.update(x)
        theta = pidy.update(y)
        
        psi = getEulerAngles(self.drone.quaternions)[2] - getEulerAngles(self.plateform.quaternions)[2]
        V_z = pidz.update(z)

        return phi, theta, psi, V_z
		
    def update(self):

        [ self.drone.U[0],self.drone.U[1],self.drone.U[2],self.drone.U[3] ] = self.gen_commande(self.drone.X[0],self.drone.X[1],self.drone.X[2])
        self.envoi_cmd(phi,theta,psi,throttle)
        
    def run(self):    
        while(True) :
    	   self.update()

def main():
    
    l = []

    try:
        dc = DroneControler(112,52,"U42")
        """
        while True:
            phi, theta, psi = cr.getEulerAngles(dc.drone.quaternions)
            pos = cr.cam2pseudoBody(dc.drone.cam.tag.X, phi, theta)
            if dc.drone.cam.tag.initialized:
                print("{:.2f} | {:.2f} \t {:.2f} | {:.2f} \t {:.2f} | {:.2f} ".format(dc.drone.cam.tag.X[0], pos[0], dc.drone.cam.tag.X[1], pos[1], dc.drone.cam.tag.X[2], pos[2]))
                l.append( (dc.drone.cam.tag.X[0], dc.drone.cam.tag.X[1], dc.drone.cam.tag.X[2], pos[0], pos[1], pos[2], phi, theta, psi) )
            else:
                print("Pas encore vu l'aruco")
            time.sleep(0.5)
        """

        while True:

            posBody = cr.cam2body(dc.drone.cam.tag.X)

            phi, theta, psi = cr.getEulerAngles(dc.drone.quaternions)
            pos = cr.cam2pseudoBody(dc.drone.cam.tag.X, phi, theta)

            timeNow = time.time()
            print("time now = {}, tag time last update = {}, delta = {}".format(timeNow, dc.drone.cam.tag.timeLastUpdate, timeNow - dc.drone.cam.tag.timeLastUpdate))

            if timeNow - dc.drone.cam.tag.timeLastUpdate < 3:
                dc.envoi_cmd_guidedSetPointNED()
            else:
                print("Tag perdu de vue depuis {} secondes".format(timeNow - dc.drone.cam.tag.timeLastUpdate))

            print("Xtag_drone = {:.2f} | {:.2f} | {:.2f}".format(dc.drone.cam.tag.X[0], dc.drone.cam.tag.X[1], dc.drone.cam.tag.X[2]))
            time.sleep(0.05)


            if dc.drone.cam.tag.initialized:
                #print("{:.2f} | {:.2f} | {:.2f} \t {:.2f} | {:.2f} | {:.2f}  \t {:.2f} | {:.2f} | {:.2f} ".format(dc.drone.cam.tag.X[0],posBody[0], pos[0], dc.drone.cam.tag.X[1], posBody[1], pos[1], dc.drone.cam.tag.X[2], posBody[2], pos[2]))
                #print("Xtag_cam = \t\t{:.2f} \t| {:.2f} \t| {:.2f}".format(dc.drone.cam.tag.X[0], dc.drone.cam.tag.X[1], dc.drone.cam.tag.X[2]))
                #print("Xtag_body = \t\t{:.2f} \t| {:.2f} \t| {:.2f}".format(posBody[0], posBody[1], posBody[2]))
                #print("Xtag_PSEUDObody = \t{:.2f} \t| {:.2f} \t| {:.2f}".format(pos[0], pos[1], pos[2]))
                l.append( (dc.drone.cam.tag.X[0], dc.drone.cam.tag.X[1], dc.drone.cam.tag.X[2], pos[0], pos[1], pos[2], phi, theta, psi) )
            else:
                print("Pas encore vu l'aruco")
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Keyboard interrupt !")
        dc.stop()
    else:
        with open("data/posAndCameraRotation.csv", 'wb') as myfile:
            wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
            for d in l:
                wr.writerow(d)



if __name__ == '__main__':
    main()

