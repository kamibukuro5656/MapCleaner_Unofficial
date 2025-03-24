# MapCleaner Unofficial Docker

## Dependencies
- Use [rocker](https://github.com/osrf/rocker) for execution.
- Please install it according to the README of [rocker](https://github.com/osrf/rocker).

## How to use
- The [data](./data) directory will be mounted to `/data` inside the container.
- Place the dataset in the [data](./data) directory and edit [data/config/config.yaml](data/config/config.yaml) to set parameters.
- [data/config/config.yaml](data/config/config.yaml) will be automatically copied to `/MapCleaner/src/config/config.yaml`.
- When you run [run.bash](./run.bash), the Docker image will be built and MapCleaner will be started.