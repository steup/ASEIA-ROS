#!/bin/sh

exec sudo tc qdisc del dev lo root
