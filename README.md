# ROS 2 Docker Environment

## Overview

This repository contains a Dockerfile for setting up a ROS 2 environment with the `adf_package`.

## Prerequisites

- Docker installed on your machine. See [Docker Installation Guide](https://docs.docker.com/get-docker/).

## Building the Docker Image

To build the Docker image, run the following command from the root of this repository:

```bash
cd emulation_code
docker build -t adf_package .
```

## Running the Docker Image

To run the Docker container, use:

```bash
docker run adf_package
```
