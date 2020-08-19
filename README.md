# SEAN: Social Environment for Autonomous Navigation

Preview site: https://romantic-curie-8ece18.netlify.app/

Pushing to master will update the preview site.

## Setup

Install [pipenv](https://pypi.org/project/pipenv).

Clone this project, `cd` into the directory, and then run `pipenv install`.

## Live-reload Editing

In the project directory, run `pipenv run sphinx-reload .`

Your browser should open and update as you make edits and save.

## Rebuilding all HTML

In case the TOC is updated, not all pages my reflect the change until built. Rebuilt all pages with

    pipenv run make html
