#!/bin/bash
if [ ! "$BASH_VERSION" ] ; then
    exec /bin/bash "$0" "$@"
fi

# launch simulation first
./simviz &
SIMVIZ_PID=$!

# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

function ctrl_c() {
    kill -2 $SIMVIZ_PID  
}

sleep 2

# try run py file
python3 interface.py &
SERVER_PID=$!

# launch controller
./controller &
CONTROLLER_PID=$!

sleep 1

# launch interfaces server
#python3 interface/server.py HoopHero.html &
#SERVER_PID=$!

# wait for simviz to quit
wait $SIMVIZ_PID

# once simviz dies, kill controller & interfaces server
kill $CONTROLLER_PID
#for pid in $(ps -ef | grep interface/server.py | awk '{print $2}'); do kill -9 $pid; done
for pid in $(ps -ef | grep interface.py | awk '{print $2}'); do kill -9 $pid; done