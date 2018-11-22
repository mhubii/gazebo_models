# Gazebo Models

This repository holds models for gazebo. Note that some of the models only functionally work with their plugins that need to be built first. Therefore, please have a look at the respective [model sections](#models) below. To prepare the models for the use with Gazebo, please have a look at the [build section](#build).

## Build

You can either install the models to a location that Gazebo knows, or update the path, where Gazebo is looking for models. Updating the path will usually keep your system cleaner.

### Update Path

Add this line to your `~/.bashrc`

```
export GAZEBO_MODEL_PATH=<location to which you cloned this repository>/gazebo_models:$GAZEBO_MODEL_PATH
```

### Install

You can also install the models. Note that this is no necessary if you already updated your path. To install the models do

```
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=~/.gazebo ..
make install
```

To uninstall the models do

```
cd build
make uninstall
```

## Models

The repository holds following models.

<br>
<figure>
  <p align="center"><img src="img/goal.jpg" width="20%" height="20%" hspace="40"><img src="img/obstacle.jpg" width="20%" height="20%" hspace="40"><img src="img/vehicle.jpg" width="20%" height="20%" hspace="40"></p>
  <figcaption>Fig. 1: Models from left to right: Goal, obstacle, vehicle. </figcaption>
</figure>
<br><br>

## Vehicle

The vehicle supports autonomous navigation, and keyboard controlled navigation. This only works if you built the plugins properly. Please follow the instructions in [gazebo_plugins](https://github.com/mhubii/gazebo_plugins).
