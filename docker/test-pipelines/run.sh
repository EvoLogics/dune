#!/bin/bash

source "${BASH_SOURCE%/*}"/lib/execution
source "${BASH_SOURCE%/*}"/lib/logging

CONT_NAME="test-pipelines"

usage()
{
  log  ""
  bold "SYNOPSIS"
  log  "  $(basename $0) [-hbr]"
  log  ""
  bold "DESCRIPTION"
  log  "  Run the Docker image for receiving the WIC video stream."
  log  ""
  bold "OPTIONS"
  log  "  -h  Show this help text."
  log  "  -b  Start bash session in container (useful for looking around / debugging)."
  log  "  -r  Run receive pipeline."
}

connect_x_server()
{
  xhost +local:docker >&/dev/null
}

disconnect_x_server()
{
  xhost -local:docker >&/dev/null
}

run_docker()
{
  local command="${1}"
  connect_x_server
  docker run -it \
    --rm \
    --name "${CONT_NAME}" \
    --network host \
    -e DISPLAY=${DISPLAY} \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    "${CONT_NAME}" \
    "${command}"
  disconnect_x_server
  }

while getopts 'hbrs' flag; do
  case "${flag}" in
    h)
      usage && exit 0;;
    b)
      run_docker "bash";;
    r)
      run_docker "/receive-pipeline.sh";;
    *)
      error "Invalid option: -$OPTARG" && exit 1;;
  esac
done
