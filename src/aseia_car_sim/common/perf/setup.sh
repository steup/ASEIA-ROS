#!/bin/sh

sudo mount -o remount,mode=755 /sys/kernel/debug
sudo mount -o remount,mode=755 /sys/kernel/debug/tracing
sudo sh -c "echo -1 > /proc/sys/kernel/perf_event_paranoid"
sudo perf probe -d kalman
sudo perf probe -d acc
sudo perf probe -d utm
sudo perf probe -x ${HOME}/eos/research/ros/devel/lib/libCarSimTransformations.so kalman=_ZN13aseia_car_sim24VirtACCKalmanTransformerclERK9MetaEvent
sudo perf probe -x ${HOME}/eos/research/ros/devel/lib/libCarSimTransformations.so acc=_ZN13aseia_car_sim18VirtACCTransformerclERK9MetaEvent
sudo perf probe -x ${HOME}/eos/research/ros/devel/lib/libCarSimTransformations.so utm=_ZN13aseia_car_sim20UTMToRoadTransformerclERK9MetaEvent
