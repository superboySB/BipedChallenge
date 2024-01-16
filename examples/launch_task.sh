# kill potential launched controller server
pkill task
sleep 1
cd /leju_controller;
./task$1_controller &
# wait for server
sleep 1
cd /BipedChallenge
/isaac-sim/python.sh examples/task_launcher.py $1 --no-window
