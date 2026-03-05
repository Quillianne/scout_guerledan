#!/bin/bash
# Script pour lancer 3 jumeaux numériques Blueboat + MAVROS

###############################################
# BLUEBOAT 1 — instance 0 → ports 5760 / 5762 #
###############################################

gnome-terminal --tab --title "Ardurover1" -- bash -c "
  cd ./Ardurover1 &&
  ./ardurover --home 48.199,-3.015,122,0 --model rover-skid --instance 0 --defaults blueboat1.parm"

sleep 3

gnome-terminal --tab --title "Mavros1" -- bash -c "
  ros2 run mavros mavros_node \
    --ros-args -r __ns:=/blueboat1 \
    --param fcu_url:='tcp://127.0.0.1:5762' \
    --param tgt_system:=1 \
    --param tgt_component:=1 \
    --param system_id:=255 \
    --param component_id:=191"
  

###############################################
# BLUEBOAT 2 — instance 1 → ports 5770 / 5772 #
###############################################

gnome-terminal --tab --title "Ardurover2" -- bash -c "
  cd ./Ardurover2 &&
  ./ardurover --home 48.1991,-3.0149,122,0 --model rover-skid --instance 1 --defaults blueboat2.parm"

sleep 3

gnome-terminal --tab --title "Mavros2" -- bash -c "
  ros2 run mavros mavros_node \
    --ros-args -r __ns:=/blueboat2 \
    --param fcu_url:='tcp://127.0.0.1:5772' \
    --param tgt_system:=2 \
    --param tgt_component:=1 \
    --param system_id:=255 \
    --param component_id:=192"


###############################################
# BLUEBOAT 3 — instance 2 → ports 5780 / 5782 #
###############################################

gnome-terminal --tab --title "Ardurover3" -- bash -c "
  cd ./Ardurover3 &&
  ./ardurover --home 48.1991,-3.0151,122,0 --model rover-skid --instance 2 --defaults blueboat3.parm"

sleep 3

gnome-terminal --tab --title "Mavros3" -- bash -c "
  ros2 run mavros mavros_node \
    --ros-args -r __ns:=/blueboat3 \
    --param fcu_url:='tcp://127.0.0.1:5782' \
    --param tgt_system:=3 \
    --param tgt_component:=1 \
    --param system_id:=255 \
    --param component_id:=193"

sleep 3

gnome-terminal --tab --title "QGC" -- bash -c "
  ./QGroundControl-x86_64.AppImage"
