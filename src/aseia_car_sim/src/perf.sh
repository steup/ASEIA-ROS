#!/bin/sh

exec perf record -e "{cycles,probe_libCarSimTransformations:kalman,probe_libCarSimTransformations:acc,probe_libCarSimTransformations:utm}" -F 249 -s -T --call-graph dwarf -a -o ${HOME}/.ros/perf
