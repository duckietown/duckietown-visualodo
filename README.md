[![CircleCI](https://circleci.com/gh/duckietown/duckietown-visualodo.svg?style=shield)](https://circleci.com/gh/duckietown/duckietown-visualodo)

[![Coverage Status](https://coveralls.io/repos/github/duckietown/duckietown-visualodo/badge.svg?branch=master&service=github)](https://coveralls.io/github/duckietown/duckietown-visualodo?branch=master)

[![PyPI status](https://img.shields.io/pypi/status/duckietown_visualodo.svg)](https://pypi.python.org/pypi/duckietown_visualodo/)


[![PyPI pyversions](https://img.shields.io/pypi/pyversions/duckietown_visualodo.svg)](https://pypi.python.org/pypi/duckietown_visualodo/)


# Visual Odometry

A pipeline for monocular visual odometry


## Installation from source

This is the way to install within a virtual environment created by
using `pipenv`:

    $ pipenv install
    $ pipenv shell
    $ cd lib-visualodo
    $ pip install -r requirements.txt
    $ python setup.py develop --no-deps


## Unit tests

To perform the unit tests present run:

    $ make -C lib-visualodo tests-clean tests

The output is generated in the folder in `lib-visualodo/out-comptests/`.

## Visualization of the results

In case you want to enable the plotting features (i.e. image containing the used mask, the feature extraction and matching or the outlier filtering), set the relevant variable to `True` in `lib-visualodo/src/duckietown-visualodo/data/default.yaml`

## Documentation

Detailed documentation for running the demo and on the implementation of the visual odometry pipeline can be found on the [Duckiebook](http://docs.duckietown.org/DT18/opmanual_duckiebot/out/demo_visualodometry.html)

## Code structure

The package is structured the following way: in `ros-visualodo`, the code responsible for the node instantiation and the communication with other nodes is placed. In `lib-visualodo`, all the actual logic is implemented, the parameters are contained and the unit tests are present.
