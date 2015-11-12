#!/bin/bash


#TEST 1
speed="1.0"
step="0.05"

echo "Test with speed_max = $speed  and step = $step"
rosrun speed_test linear_speed _speed_max:=$speed _speed_step:=$step
echo "Done."

#TEST 2
speed="0.15"
step="0.01"

echo "Test with speed_max = $speed  and step = $step"
rosrun speed_test linear_speed _speed_max:=$speed _speed_step:=$step
echo "Done."

#TEST 1
speed="0.10"
step="0.005"

echo "Test with speed_max = $speed  and step = $step"
rosrun speed_test linear_speed _speed_max:=$speed _speed_step:=$step
echo "Done."

