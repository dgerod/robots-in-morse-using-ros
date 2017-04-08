#!/bin/bash
rostopic echo -b $1 -n 1 /metadata | grep "data"
