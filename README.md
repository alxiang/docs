# SEAN: Social Environment for Autonomous Navigation

Documentation site: http://sean.interactive-machines.com/

Pushing to the master branch of this repository will update the preview site.

## Source Repositories

ROS project: https://github.com/yale-img/social_sim_ros

Unity Project: https://github.com/yale-img/social_sim_unity

Documentation (this project): https://github.com/yale-img/social-sim-docs

Dockerized Catkin Workspace: https://github.com/yale-img/sim_ws

## Setup

Install [pipenv](https://pypi.org/project/pipenv).

Clone this project, `cd` into the directory, and then run `pipenv install`.

## Live-reload Editing

In the project directory, run `pipenv run sphinx-reload .`

Your browser should open and update as you make edits and save.

## Rebuilding all HTML

In case the TOC is updated, not all pages my reflect the change until built. Rebuilt all pages with

    pipenv run make html
