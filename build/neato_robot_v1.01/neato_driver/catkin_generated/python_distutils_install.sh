#!/bin/sh -x

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

cd "/home/serveur/catkin_ws/src/neato_robot_v1.01/neato_driver"

# todo --install-layout=deb per platform
# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/home/serveur/catkin_ws/install/lib/python2.7/dist-packages:/home/serveur/catkin_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/serveur/catkin_ws/build" \
    "/usr/bin/python" \
    "/home/serveur/catkin_ws/src/neato_robot_v1.01/neato_driver/setup.py" \
    build --build-base "/home/serveur/catkin_ws/build/neato_robot_v1.01/neato_driver" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/serveur/catkin_ws/install" --install-scripts="/home/serveur/catkin_ws/install/bin"
