#
# A convenient, gym-like wrapper for pybullet
# https://github.com/liusida/PyBulletWrapper
#
# Author of this file:
#   2020 Sida Liu (learner.sida.liu@gmail.com)
# License:
#   MIT
# Description:
#   Float numbers with too many digits are hard to read on the screen.
#   This wrapper provides a prettier print function so the returns from pybullet can be more readable.
# Support Version:
#   Tested under pybullet 2.8.5 (installed via `pip install pybullet`)
#   Python 3.6
# Reference:
#   (1) PyBullet Quickstart Guide
#   (2) pybullet.c

from .base import BaseWrapperPyBullet
from pprint import PrettyPrinter
import collections
import numbers


def formatfloat(x):
    ret = "%.3f" % float(x)
    if float(x) >= 0.0:
        return f" {ret}"
    return ret


def pformat(dictionary, function):
    if isinstance(dictionary, dict):
        return type(dictionary)((key, pformat(value, function)) for key, value in dictionary.items())
    # Warning: bytes and str are two kinds of collections.Container, but we don't want to go inside it, so it should be pick out here.
    if isinstance(dictionary, bytes) or isinstance(dictionary, str):
        return dictionary
    if isinstance(dictionary, collections.Container):
        return type(dictionary)(pformat(value, function) for value in dictionary)
    if isinstance(dictionary, float):
        return function(dictionary)
    return dictionary


class PrettyFloatPyBullet(BaseWrapperPyBullet):
    def prettyPrint(self, x):
        PrettyPrinter().pprint(pformat(x, formatfloat))
