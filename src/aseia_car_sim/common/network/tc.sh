#!/bin/sh

sudo tc qdisc add dev lo root handle 1: htb default 1
sudo tc class add dev lo parent 1: classid 1:2 htb rate 1Mbit
sudo tc class add dev lo parent 1: classid 1:3 htb rate 1Mbit
sudo tc class add dev lo parent 1: classid 1:4 htb rate 1Mbit
sudo tc qdisc add dev lo parent 1:2 handle 2: netem loss 10
sudo tc qdisc add dev lo parent 1:3 handle 3: netem delay 100ms 10ms
sudo tc qdisc add dev lo parent 1:4 handle 4: netem loss 10 delay 100ms 10ms
sudo tc filter add dev lo parent 1: protocol ip handle 2 fw classid 1:2
sudo tc filter add dev lo parent 1: protocol ip handle 3 fw classid 1:3
sudo tc filter add dev lo parent 1: protocol ip handle 4 fw classid 1:4
sudo tc filter add dev lo parent 1: protocol ipv6 handle 2 fw classid 1:2
sudo tc filter add dev lo parent 1: protocol ipv6 handle 3 fw classid 1:3
sudo tc filter add dev lo parent 1: protocol ipv6 handle 4 fw classid 1:4
