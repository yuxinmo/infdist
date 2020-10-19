This repository contains the main part of the information distribution
optimization middleware.


Using the ROS node
==================

The whole repository has a structure of ROS2 node. The node can be used to 
optimize information distribution in ROS. An example usage of this node is
available at: https://gitlab.aau.at/aau-nav/development/camera_streaming .

Simulations
===========

Directory infdist/simulator contains a framework to run simulated experiments in
NS3. We also provide a docker image to make it easier to install and run
simulations.

Using the docker container
--------------------------



Manual installation
-------------------

1. Install NS3
2. Install all requirements from requirements.txt
3. Run main_ns3.py from NS3

In order to generate plots you also need plotly and plotly-orca. 


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
