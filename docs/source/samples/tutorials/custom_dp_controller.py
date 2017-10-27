#!/usr/bin/env python
import rospy
import numpy as np
from uuv_control_interfaces import DPControllerBase

class TutorialDPController(DPControllerBase):
  def __init__(self):
      super(TutorialDPController, self).__init__(self)

      self._Kp = np.zeros(shape=(6, 6))
      self._Kd = np.zeros(shape=(6, 6))
      self._Ki = np.zeros(shape=(6, 6))

      self._int = np.zeros(shape=(6,))

      self._error_pose = np.zeros(shape=(6,))

       # Do the same for the other two matrices
      if rospy.get_param('~Kp'):
          diag = rospy.get_param('~Kp')
          if len(diag) == 6:
              self._Kp = np.diag(diag)
              print 'Kp=\n', self._Kp
          else:
              # If the vector provided has the wrong dimension, raise an exception
              raise rospy.ROSException('For the Kp diagonal matrix, 6 coefficients are needed')

      if rospy.get_param('~Kd'):
          diag = rospy.get_param('~Kd')
          if len(diag) == 6:
              self._Kd = np.diag(diag)
              print 'Kd=\n', self._Kd
          else:
              # If the vector provided has the wrong dimension, raise an exception
              raise rospy.ROSException('For the Kd diagonal matrix, 6 coefficients are needed')

      if rospy.get_param('~Ki'):
          diag = rospy.get_param('~Ki')
          if len(diag) == 6:
              self._Ki = np.diag(diag)
              print 'Ki=\n', self._Ki
          else:
              # If the vector provided has the wrong dimension, raise an exception
              raise rospy.ROSException('For the Ki diagonal matrix, 6 coefficients are needed')
      self._is_init = True

  def _reset_controller(self):
      super(TutorialDPController, self)._reset_controller()
      self._error_pose = np.zeros(shape=(6,))
      self._int = np.zeros(shape=(6,))

  def update_controller(self):
      if not self._is_init:
          return False

      if not self.odom_is_init:
          return
      self._int = self._int + 0.5 * (self.error_pose_euler + self._error_pose) * self._dt
      self._error_pose = self.error_pose_euler
      tau = np.dot(self._Kp, self.error_pose_euler) + np.dot(self._Kd, self._errors['vel']) + np.dot(self._Ki, self._int)
      self.publish_control_wrench(tau)

if __name__ == '__main__':
  print('Tutorial - DP Controller')
  rospy.init_node('tutorial_dp_controller')

  try:
      node = TutorialDPController()
      rospy.spin()
  except rospy.ROSInterruptException:
      print('caught exception')
  print('exiting')
