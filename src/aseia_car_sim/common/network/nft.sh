#!/bin/sh

sudo nft delete table inet ros
exec sudo nft -f nft.rules
