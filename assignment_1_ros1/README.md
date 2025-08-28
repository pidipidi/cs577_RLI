# Quick Start 

Before running followings, you must load environment variables on each terminal using baxter.sh:
~~~~bash
./baxter.sh sim
~~~~

## Problem 1.A
After implementing the original DMPs on the problem_1.py, run it as follows:
~~~~bash
roscd assignment_1/src/assignment_1
python3 problem_1.py -s a
~~~~

## Problem 1.B
After implementing the plan() function in dmp.py, run it as follows:
~~~~bash
roscd assignment_1/src/assignment_1
python3 problem_1.py -s b
~~~~

## Problem 1.C
After implementing the learn() function in dmp.py, run it as follows:
~~~~bash
roscd assignment_1/src/assignment_1
python3 problem_1.py -s c
~~~~

## Problem 1.D
After implementing the plan() function in dmp.py, run it as follows:
~~~~bash
roscd assignment_1/src/assignment_1
python3 problem_1.py -s d
~~~~

#Problem 2
## Problem 2.A
Train an original DMP with a 2-dimensional demonstration on the problem_2.py. You can run the code as follows:
~~~~bash
roscd assignment_1/src/assignment_1
python3 problem_2.py -s a1
~~~~

After implementing the improved DMPs on the problem_2.py, you can run the same problem as follows:
~~~~bash
roscd assignment_1/src/assignment_1
python3 problem_2.py -s a2
~~~~

## Problem 2.B
After implementing the plan() function dmp.py for the improved DMP, run it as follows:
~~~~bash
roscd assignment_1/src/assignment_1
python3 problem_2.py -s b
~~~~


# Problem 3
You first need to launch the Baxter robot:
~~~~bash
roslaunch baxter_gazebo baxter_world.launch
~~~~

You can test the robot by running the joint trajectory example program. Please, open another terminal session. After sourcing the environment variables as before, run following:
~~~~bash
rosrun assignment_1 joint_trajectory_client.py -l right
~~~~
Note that you need to specify the left or right arm using the '-l' tag. 


## Problem 3.A
Train the improved DMPs with joint-space demonstrations via problem_3.py:
~~~~bash
roscd assignment_1/src/assignment_1
python3 problem_3.py -s a1
~~~~

Reproduce it with the Baxter robot
~~~~bash
roscd assignment_1/src/assignment_1
python3 problem_3.py -s a2
~~~~


## Problem 3.B
Train the improved DMPs with Cartesian-space demonstrations and reproduce it via problem_3.py:
~~~~bash
roscd assignment_1/src/assignment_1
python3 problem_3.py -s b
~~~~


## Problem 3.C
Train the improved DMPs with Cartesian-space demonstrations, adapt the goal to another pose, and reproduce it via problem_3.py:
~~~~bash
roscd assignment_1/src/assignment_1
python3 problem_3.py -s c
~~~~
