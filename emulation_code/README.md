# Automatic Dog Feeder Emulation

This repository contains a Dockerfile and associated ROS 2 workspace for an automatic dog feeder system. The application is designed to run within a Docker container using ROS 2 Iron and includes a web bridge for interfacing with the system through a web application.

## Prerequisites

Before you can build and run the Docker container, ensure that you have Docker installed on your machine. For detailed instructions, see the [Docker Installation Guide](https://docs.docker.com/get-docker/).

## Directory Structure

- `./emulation_code`: Contains the Dockerfile and ROS scripts needed for emulation of sensors.
- `./sensor_code`: ROS2 workspace containing code to interface directly with sensors.

## Building the Docker Image

To build the Docker image, navigate to the directory containing the Dockerfile (`./emulation`) and run the following command:

```bash
cd emulation
docker build -t adf_package .
```

This command builds a Docker image named `adf_package` based on the instructions in your Dockerfile.

## Running the Docker Container

After building the image, you can run the Docker container using:

```bash
docker run adf_package
```
