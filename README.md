# self-racing-rc-platform

WIP

## Setup

### Catkin workspace

* Clone this repo under `~/workspace/self_racing_rc_platform_ws`, as `src`

### Docker environment

All the code runs in a Docker, built and started with Docker compose.

* Add the following line to your `~/.bashrc`: `export CURRENT_UID=$(id -u):(id -g)`
* Build the Docker: `cd ~/workspace/self_racing_rc_platform_ws/src && docker compose build`
* Start the Docker: `docker compose up`
* Terminals in the Docker can be entered with `docker exec -it src-app-1 /bin/bash`

### Install pre-commit hooks

To install the pre-commit hooks, run: `pre-commit install`

It is possible to commit without running the commit hooks by running `git commit --no-verify`

## Building the catkin packages

Run `catkin build` from the top of the repo

## Building the `arduino_pkg` package

Follow the instruction in `arduino_pkg/README.md`
