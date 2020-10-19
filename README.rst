This repository contains the main part of the information distribution
optimization middleware.


ROS node
========

The whole repository has a structure of ROS2 node. The node can be used to 
optimize information distribution in ROS2. An example usage of this node is
available at: https://gitlab.aau.at/aau-nav/development/camera_streaming .

Simulations
===========

Directory infdist/simulator contains a framework to run simulated experiments in
NS3. We also provide a docker image to make it easier to install and run
simulations.

Using the docker container
--------------------------

Using the provided Docker container is the easiest way to re-produce our
results. First, install Docker and make sure that Docker deamon is running.
Then:

.. code-block:: bash

  $ cd docker
  $ ./build.sh  
  (...)  # takes around 40 minutes on my laptop
  $ ./start.sh
  # This will start the docker container and switch us to the shell opened
  # inside this container
  # Read the instructions to find out the supported commands.
  $ run_infdist_experiments
  (...) # This might take a couple of hours.
  # After the command is finished the resulting plots should appear in /tmp


Manual installation
-------------------

1. Install NS3
2. Install all requirements from requirements.txt
3. Run main_ns3.py from NS3

In order to generate plots you also need plotly and plotly-orca. The Dockerfile
[https://github.com/zeroos/infdist/blob/master/docker/Dockerfile] can serve as
a working example of what needs to be done.


Code structure
==============

If you are just interesting in having a look at the code in order to better
understand how it is working, below we provided descriptions of the most
interesting files:

- https://github.com/zeroos/infdist/blob/master/infdist/optimization/agent.py
  this file contains definitions of multiple "Agents", which manage the main
  flow of messages: i.e., what happens before a message is sent, how received
  messages are incorporated, what goes into the decision tree, etc. In most
  experiments EstimatingAgent is used, FullKnowledgeAgent is a good starting
  point while developing new methods.

- https://github.com/zeroos/infdist/blob/master/infdist/optimization/dynamic_message_tree.py
  the decision tree is managed in this file. MCTS algorithm is implemented in a
  separate repository (https://github.com/zeroos/monte-carlo-tree-search), but
  this file configures and manages it for the purpose of information
  distribution.
