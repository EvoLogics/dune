#!/bin/bash

source "${BASH_SOURCE%/*}"/lib/execution
source "${BASH_SOURCE%/*}"/lib/logging
source "${BASH_SOURCE%/*}"/lib/validation


usage()
{
  log  ""
  bold "SYNOPSIS"
  log  "    $0 [-hnv] <JETSON_USER> <JETSON_IP> <DUNE_LOCATION>"
  log  ""
  bold "DESCRIPTION"
  log  "    This script pushes/updates DUNE on the Jetson board at location ~/dev/dune/."
  log  ""
  bold "ARGUMENTS"
  log  "    JETSON_USER    : Name of the user on the Jetson board."
  log  "    JETSON_IP      : IP address of the Jetson board."
  log  "    DUNE_LOCATION  : Location of the DUNE repository on the local machine."
  log  ""
  bold "OPTIONS"
  log  "    -h : Show this help message."
  log  "    -n : Dry run (do not execute any commands)."
  log  "    -v : Verbose mode (show commands that are being executed)."
}

# Get command line arguments
while getopts 'hnv' opt; do
  case $opt in
    h)
      usage && exit 0;;
    n)
      warn "Running in dry-run mode..." && set_dry_run && set_logging_debug;;
    v)
      set_logging_debug;;
    *)
      error "Invalid option: -$OPTARG";;
  esac
done

# Get and check arguments
JETSON_USER=${@:$OPTIND:1}
JETSON_IP=${@:$OPTIND+1:1}
DUNE_LOCATION=${@:$OPTIND+2:1}


info "Updating DUNE on jetson..."
execute "rsync \
-av \
--exclude=build/* \
--exclude=compile_commands.json \
--exclude=.cache/* \
$DUNE_LOCATION \
$JETSON_USER@$JETSON_IP:/home/jetson/dev/dune"

success
