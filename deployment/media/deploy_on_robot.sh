#!/bin/bash

robot=$1

rsync -r -avz --delete-after \
	~/.notrebonplaisir/videos odroid@192.168.150.1$robot:~/.notrebonplaisir

echo "deployment done on robot $robot"

