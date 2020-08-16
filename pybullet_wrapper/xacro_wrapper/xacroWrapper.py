#
# A convenient, gym-like wrapper for pybullet
# https://github.com/liusida/PyBulletWrapper
#
# Author of this file:
#   2020 Sida Liu (learner.sida.liu@gmail.com)
#
# If you don't have ROS installed, please first install a version of xacro from here: https://github.com/ros-gbp/xacro-release/releases
#
from xacro import *

def xacro_to_urdf(input_file_name):
    """
    Read a .xacro file, return the content of corresponding .urdf.
    This function is modified from `main()` in xacro package.
    """
    try:
        # open and process file
        doc = process_file(input_file_name)

    # error handling
    except xml.parsers.expat.ExpatError as e:
        error("XML parsing error: %s" % unicode(e), alt_text=None)
        if verbosity > 0:
            print_location(filestack, e)
            print(file=sys.stderr)  # add empty separator line before error
            print("Check that:", file=sys.stderr)
            print(" - Your XML is well-formed", file=sys.stderr)
            print(" - You have the xacro xmlns declaration:",
                  "xmlns:xacro=\"http://www.ros.org/wiki/xacro\"", file=sys.stderr)
        sys.exit(2)  # indicate failure, but don't print stack trace on XML errors

    except Exception as e:
        msg = unicode(e)
        if not msg:
            msg = repr(e)
        error(msg)
        if verbosity > 0:
            print_location(filestack, e)
        if verbosity > 1:
            print(file=sys.stderr)  # add empty separator line before error
            raise  # create stack trace
        else:
            sys.exit(2)  # gracefully exit with error condition
    
    return doc.toprettyxml(indent='  ', **encoding)