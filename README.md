# PIK
## What is this?
This is a simple python IK solver with a simple JSON config file. 

## Requirements
- Numpy
- Scipy
- Matplotlib
- Python 2.7

## How does it works?
conf.json is the configuration file, inside this file you will need to define the list of joints. A joint is composed of the following 
```
parameters:
{
		"name": The name of the joint
		"xyz": The resting position of the joint
		"rpy": The resting angles of the joint
		"type": The type of the joint (currently supporting revolute and end)
		"child": The child's name of the joint (Invalid for type end)
		"axis":  The rotation or translation axis of the joint
		"isRoot": True if the joint is the root
}
```

## How to use it?
'''
pik.py usage:
python pik.py "<target position array>" -v
<target position array>: An array representing the position of the target to reach
-v: Outputs a visualisation graph of the robot
Example: python pik.py "[0, 0, 2]"
'''