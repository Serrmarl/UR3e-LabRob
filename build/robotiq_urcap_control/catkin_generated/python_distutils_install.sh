#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/jcab/ros_ws/src/robotiq_urcap_control"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/jcab/ros_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/jcab/ros_ws/install/lib/python2.7/dist-packages:/home/jcab/ros_ws/build/robotiq_urcap_control/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/jcab/ros_ws/build/robotiq_urcap_control" \
    "/usr/bin/python2" \
    "/home/jcab/ros_ws/src/robotiq_urcap_control/setup.py" \
     \
    build --build-base "/home/jcab/ros_ws/build/robotiq_urcap_control" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/jcab/ros_ws/install" --install-scripts="/home/jcab/ros_ws/install/bin"
