#!/bin/bash

# get directory of script
DIR="$( cd "$( dirname "$0" )" && pwd )"

# if not, this path will be used
GUACAMOLE=/opt/guacamole/master
AVANGO=/opt/avango/master

# third party libs
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/boost/current/lib:/opt/zmq/current/lib:/opt/Awesomium/lib

# schism
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/schism/current/lib/linux_x86

# avango
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$AVANGO/lib
export PYTHONPATH=$AVANGO/lib/python3.4

# guacamole
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$GUACAMOLE/lib


# run daemon
python3 ./daemon.py > /dev/null &
#python3 daemon.py

# run program
cd "$DIR" && DISPLAY=:0.0 python3.4 ./main.py


# kill daemon
kill %1
