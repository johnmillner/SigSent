## NEAT AI Code

### Background

#### NEAT
To read more about NEAT by Dr. Stanley, check out his [first paper introducing the framework here.](NEAT.pdf)

#### SigSent Code
There are comments throughout the main.py file explaining what the code should be doing. Mostly scaffolding right now.

It is my understanding that we should employ 10 input nodes. 9 of them come from the IMU's raw data. 1 of them comes from the "Terrain Classifier" that will output some scaling float value that rates the nearby environment as being smooth/easy to navigate -> rough/difficult.

The terrain classifier can be as advanced as we want. Honestly thinking it should just be some thresholds we set based on obstacles detected in the camera with OpenCV and how uneven the LIDAR has mapped the surroundings.

The output will be contained in one single node. We will be using a Sigmoid activation function on the network. A value between 0 and 1 is output. Rather than a simple step function, there is a concave ascent and descent, centered at x = 0, that allows for more complex floating values between the two activation levels. The curve can be seen [here](http://neat-python.readthedocs.io/en/latest/_images/activation-sigmoid.png). Other activation functions built in can be seen [here](http://neat-python.readthedocs.io/en/latest/activation.html).

The inputs for the network should be gathered by subscribing to whatever ROS topic they are published to. The ANN will output the True/False for whatever mobility mechanism to enter and publish it to its publisher node that it creates.

When in training mode, it should measure stability and distance traveled (using IMU and GPS) to measure fitness for how well the robot was able to maneuver while in its given travel modes. That fitness will decide what genomes were successful for future generations. When not training, the most fit genome will be used.


### Setting up

Gather the python dependencies
```sh
pip install -r requirements.txt
```

Configure the ANN settings that NEAT should use in config-ann
