# SEE: Social Evaluation Environment

The Social Evaluation Environment (SEE) is a high fidelity, extensible, and open source simulation platform designed for the fair evaluation of social navigation algorithms. It leverages advances in graphics and physics modeling to build high fidelity scenes. We include in the platform many scenes with social agents that navigate according to standard pedestrian models. Integration with the Robot Operating System (ROS) allows for compatibility with existing navigation software stacks.

To get started, got to: [Setup](setup.html#setup)

## Features

For systematic evaluation, we provide the [trial runner](trials) which allows a user to run their navigation algorithms against a set of common tasks in both indoor and outdoor environments.

## Modifying the Environment

SEE is a flexible platform. We provide documentation for adding or editing [Environments](editing.html#Environments), [Robots](editing.html#Robots), [Sensors](editing.html#Sensors), and [Pedestrian Navigation Models](editing.html#pedestrian-navigation-models).


```eval_rst
.. toctree::
    :hidden:
    :glob:

    system
    setup
    trials
    editing
```
