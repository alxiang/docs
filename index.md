# SEAN: Social Environment for Autonomous Navigation

The Social Environment for Autonomous Navigation (SEAN) is a high fidelity, extensible, and open source simulation platform designed for the fair evaluation of social navigation algorithms. It leverages advances in graphics and physics modeling to build high fidelity scenes. We include in the platform many scenes with social agents that navigate according to standard pedestrian models. Integration with the Robot Operating System (ROS) allows for compatibility with existing navigation software stacks.

```eval_rst
.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="https://www.youtube.com/embed/nD3-zz3moUM" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>
```

If you use this work in your research, please consider citing:

```
@inproceedings{Tsoi_2020_HAI,
  author    = {Tsoi, Nathan and Hussein, Mohamed and Espinoza, Jeacy and Ruiz, Xavier and V\'{a}zquez, Marynel},
  title     = {SEAN: Social Environment for Autonomous Navigation},
  booktitle = {Proceedings of the 8th International Conference on Human-Agent Interaction (HAI)},
  month     = {November},
  year      = {2020},
}
```

To get started, go to: [Setup](setup.html#setup).

## Features

For systematic evaluation, we provide the [trial runner](trials) which allows a user to run their navigation algorithms against a set of common tasks in both indoor and outdoor environments.

## Modifying the Environment

SEAN is a flexible platform. We provide documentation for adding or editing [Environments](editing.html#Environments), [Robots](editing.html#Robots), [Sensors](editing.html#Sensors), and [Pedestrian Navigation Models](editing.html#pedestrian-navigation-models).


```eval_rst
.. toctree::
    :hidden:
    :glob:

    system
    setup
    running
    trials
    people
    editing
    environment
    licences
```
