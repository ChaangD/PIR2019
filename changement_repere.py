import numpy as np
import math
import pdb

def getEulerAngles(quaternions):
    """
        convert quaternions into eulerangles, phi teta psy <=>  roll pitch yaw (angles)
		:param quaternions: array numpy de quaternions
		:type quaternions: array numpy
		:return: phi teta psi in rad
		:rtype: (float, float, float)
    """
    # quat = np.quaternion(self.quaternions[0], self.quaternions[1], self.quaternions[2], self.quaternions[3])
    # phi, teta, psi = quaternion.as_euler_angles(quat)
    qi = quaternions[0]
    qx = quaternions[1]
    qy = quaternions[2]
    qz = quaternions[3]
    phi = math.atan2(2.*(qy * qz + qi * qx), 1.0 - 2.*(qx**2 +  qy**2))
    theta = -math.asin(2.*(qx*qz - qi*qy))
    psi = math.atan2(2.*(qx*qy + qi*qz), 1.0 - 2.*(qy**2 +  qz**2))

    return theta, phi, -psi # passage de enu vers ned ! (cause: optitrack n'a pas les memes reperes)

def cam2body(Xtag_cam):
    """
        convert position from Camera Frame to Body Frame 
        :param Xtag_cam: the position of the tag in camera frame
        :type Xtag_cam: array numpy[3]
        :return: Xtag_body
        :rtype: array numpy[3]
    
    """
    # cameraX <=> bodyY | cameraY <=> -bodyX | cameraZ <=> bodyZ
    mat_cam2body = np.transpose(np.array([[0., 1., 0.], [-1, 0, 0], [0, 0, 1]]))
    return np.dot(mat_cam2body, Xtag_cam)

def body2pseudoBody(Xtag_body, phi, theta):
    """
                                                    1       0       0
    P_body2vdeux(phi) = transpose P_vdeux2body =    0       cPhi    -sPhi
                                                    0       sPhi    cPhi        
                                                                                
                                                    cTheta  0       sTheta      
    P_vdeux2vun(teta) = transpose P_vdeux2vun =     0       1       0
                                                    -sTheta 0       cTheta

    voir chrobotics.com/library/understanding-euler-angles
                                    
    """
    c = math.cos
    s = math.sin
    # l1 = [c(psi)*c(theta),    c(psi)*s(phi)*s(theta) - c(phi)*s(psi),         s(phi)*s(psi) + c(phi)*c(psi)*s(theta)]
    # l2 = [c(theta)*s(psi),    c(theta)*c(psi) + s(phi)*s(psi)*s(theta),       c(phi)*s(psi)*s(theta) - c(psi)*s(phi)]
    # l3 = [-s(theta),      c(theta)*s(phi),                                c(phi)*c(theta)]
    
    l1 = [c(theta),     s(theta)*s(phi),    c(phi)*s(theta)]
    l2 = [0,            c(phi),             -s(phi)]
    l3 = [-s(theta),    s(phi)*c(theta),    c(theta)*c(phi)]


    P_body2pseudoBody = np.array([l1, l2, l3])

    return np.dot(P_body2pseudoBody, Xtag_body)

def cam2pseudoBody(Xtag_cam, phi, theta):
    """
        convert position from Camera Frame to pseudoBody frame 
        :param Xtag_cam: the position of the tag in camera frame
        :param phi: roll angle from inertial frame to body frame
        :param teta: pitch angle from inertial frame to body frame
        :return: Xtag_pseudoBody 
    
    """
    Xtag_body = cam2body(Xtag_cam)
    return body2pseudoBody(Xtag_body, phi, theta)

    
def uwb2pseudoBody(x,y,z) :
    	"""
    	Renvoit le vecteur position du la plateforme dans le repere pseudo body a partir des coordonnees du drone dans le repere de la platefore
    	
    	"""
    	X = np.zeros(3)
    	phi,theta,psi = getEulerAngles()
    	mat_rot = np.array([[math.cos(psi),math.sin(psi),0],[-math.sin(psi),math.cos(psi),0],[0,0,-1]])
    	X = np.dot(mat_rot,[x,y,z])
    	return X



def main():
#	Xtag_cam = np.array([0, 0, 2])
	
#	Xtag_body = cam2body(Xtag_cam)

	Xtag_body = np.array([0,0,2])

	phi = -math.pi/6
	theta = 0

	Xtag_pseudoBody = cam2pseudoBody(Xtag_body, phi, theta)
	
	pdb.set_trace()


if __name__ == '__main__':
	main()

	"""
>>> phi = -math.pi/6
>>> zbody = 2
>>> ybody = 0
>>> import numpy as np
>>> zpb = zbody * math.cos(phi)
>>> ypb = ybody * math.sin(phi)
>>> print(zpb, ypb)
(1.7320508075688774, -0.0)
>>> 


	"""
