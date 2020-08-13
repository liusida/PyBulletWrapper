#
# A convenient, gym-like wrapper for pybullet
# https://github.com/liusida/PyBulletWrapper
#
# Author of this file:
#   2020 <Your name> (<your email>)
# License:
#   MIT
# Description:
#   <Why do you make this wrapper?>
#   <What does this wrapper do?>
# Support Version:
#   Tested under pybullet 2.8.5 (installed via `pip install pybullet`)
#   Python 3.6
# Reference:
#   (1) PyBullet Quickstart Guide
#   (2) pybullet.c

from .base import BaseWrapperPyBullet


class TempPyBullet(BaseWrapperPyBullet):
    def my_func(self):
        """<What does this function do?>"""
        self.connect(self.GUI)
