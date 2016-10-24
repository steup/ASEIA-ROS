#!/bin/sh

sudo tc -s qdisc show dev lo
sudo tc -g class show dev lo
sudo tc filter show dev lo
