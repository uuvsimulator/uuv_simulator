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

echo_and_run cd "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_control"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/amishsqrrob/uuv_simulator/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/amishsqrrob/uuv_simulator/install/lib/python2.7/dist-packages:/home/amishsqrrob/uuv_simulator/build/uuv_manipulators_control/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/amishsqrrob/uuv_simulator/build/uuv_manipulators_control" \
    "/usr/bin/python" \
    "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_control/setup.py" \
    build --build-base "/home/amishsqrrob/uuv_simulator/build/uuv_manipulators_control" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/amishsqrrob/uuv_simulator/install" --install-scripts="/home/amishsqrrob/uuv_simulator/install/bin"
