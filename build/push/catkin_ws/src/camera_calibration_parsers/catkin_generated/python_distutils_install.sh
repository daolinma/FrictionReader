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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/mcube-daolin/catkin_ws/src/push/catkin_ws/src/camera_calibration_parsers"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/mcube-daolin/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/mcube-daolin/catkin_ws/install/lib/python2.7/dist-packages:/home/mcube-daolin/catkin_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/mcube-daolin/catkin_ws/build" \
    "/usr/bin/python" \
    "/home/mcube-daolin/catkin_ws/src/push/catkin_ws/src/camera_calibration_parsers/setup.py" \
    build --build-base "/home/mcube-daolin/catkin_ws/build/push/catkin_ws/src/camera_calibration_parsers" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/mcube-daolin/catkin_ws/install" --install-scripts="/home/mcube-daolin/catkin_ws/install/bin"
