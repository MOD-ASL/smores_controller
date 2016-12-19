import rospy
import time
from smores_library.SmoresModule import SmoresCluster
from numpy import sin, cos, sign

class SMORESController():
    def __init__(self, module_id_list):
        self._max_torque = 40
        self.scale = 1.0
        self._forward_directon = 1
        self.max_v = 0.02
        self.max_w = 0.3
        self.v_scale = 1.0/self.max_v * 70
        self.w_scale = 1.0/self.max_w * 30

        self._max_torque = 40
        self.v_scale = 10000/5.0
        self.w_scale = 1000/6.0

        if module_id_list == None or len(module_id_list) == 0:
            rospy.logerr("No module ID list provided.")
            return
        self.c = SmoresCluster.SmoresCluster(module_id_list)

    def _checkAllBattery(self):
        for m_id, m_obj in self.c.mods.iteritems():
            rospy.loginfo("Module {} with battery level {} ...".format(m_id, m_obj.req.battery()))
            rospy.sleep(0.05)

    def getModuleObjectFromID(self, id_num):
        if id_num not in self.c.mods.keys():
            rospy.logerr("Cannot find module with ID {} ...".format(id_num))
            return None
        return self.c.mods[id_num]

    def driveForward(self, module_obj):
        if module_obj is None: return None
        self._sendTorque(module_obj, 90.0, 90.0)

    def spin(self, module_obj, direction="cw"):
        if module_obj is None: return None
        if direction == "ccw":
            self._sendTorque(module_obj, 20.0, -20.0)
        elif direction == "cw":
            self._sendTorque(module_obj, -20.0, 20.0)

    def driveBackward(self, module_obj):
        if module_obj is None: return None
        self._sendTorque(module_obj, -90.0, -90.0)

    def driveWithLocal(self, module_obj, v, w):
        if isinstance(module_obj, int):
            module_obj = self.getModuleObjectFromID(module_obj)
        if module_obj is None: return None

        #if abs(v) > self.max_v:
        #    v = sign(v)*self.max_v
        #if abs(w) > self.max_w:
        #    w = sign(w)*self.max_w
        left_vel = v * self.v_scale + w * self.w_scale
        right_vel = v * self.v_scale - w * self.w_scale

        self._sendTorque(module_obj, left_vel, right_vel)

    def stopAllMotors(self, module_obj):
        if isinstance(module_obj, int):
            module_obj = self.getModuleObjectFromID(module_obj)
        if module_obj is None: return None
        rospy.logdebug("Stoping all motors ...")
        module_obj.move.send_torque('left', 0)
        rospy.sleep(0.05)
        module_obj.move.send_torque('right', 0)
        rospy.sleep(0.05)
        module_obj.move.send_torque('pan', 0)
        rospy.sleep(0.05)
        module_obj.move.send_torque('tilt', 0)
        rospy.sleep(0.05)

    def _sendTorque(self,module_obj, left_t, right_t):
        if abs(left_t) > self._max_torque:
            scale = abs(left_t)/self._max_torque
            left_t = sign(left_t) * self._max_torque
            right_t = right_t / scale
        if abs(right_t) > self._max_torque:
            scale = abs(right_t)/self._max_torque
            right_t = sign(right_t) * self._max_torque
            left_t = left_t / scale
        print "before: l: {}, r: {}".format(left_t,right_t)

        rospy.logdebug("Sending left: {}    right: {}".format(
            left_t*self._forward_directon, right_t*-self._forward_directon))
        module_obj.move.send_torque('left', left_t*self._forward_directon)
        rospy.sleep(0.05)
        module_obj.move.send_torque('right', right_t*-self._forward_directon)
        rospy.sleep(0.05)

    def global2Local(self, x, y, theta):
        vx = 1.1*x
        vy = 1.1*y
        w = (1/0.10)*(-sin(theta)*vx + cos(theta)*vy)
        v = cos(theta)*vx + sin(theta)*vy
        print "v: {}, w: {}".format(v,w)
        return [v,w]

if __name__ == '__main__':
    r = SMORESController()
