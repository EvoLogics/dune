#!/bin/bash

source "${BASH_SOURCE%/*}"/lib/execution
source "${BASH_SOURCE%/*}"/lib/logging
source "${BASH_SOURCE%/*}"/lib/validation


# =========
# Constants
# =========

# JETSON_IP_DEFAULT="192.168.3.17"
# WIC_SDK_LICENSE_LOCATION="~/nas/home/resources/WIC/Workswell_WIC/306d2211/license\ 306D2211.wlic"
# WIC_SDK_ZIP_LOCATION="~/nas/home/resources/WIC/Workswell_WIC/WIC_SDK_ARM_1.1.0.zip"


# =================
# Utility functions
# =================

usage()
{
  log  ""
  bold "SYNOPSIS"
  log  "    $0 [-hnv] <JETSON_USER> <JETSON_IP> <WIC_SDK_ZIP_LOCATION> <WIC_LICENSE_LOCATION>"
  log  ""
  bold "DESCRIPTION"
  log  "    This script configures the Jetson board so that it can be used to compile and run the DUNE WIC Task."
  log  "    It installs the necessary packages, the WIC SDK and the DUNE source code."
  log  "    It also adds a line to ~/.bashrc to source the WIC SDK environment variables."
  log  ""
  bold "ARGUMENTS"
  log  "    JETSON_USER              : Name of the user on the Jetson board."
  log  "    JETSON_IP                : IP address of the Jetson board."
  log  "    WIC_SDK_ZIP_LOCATION     : Location of the WIC SDK zip file on the local machine."
  log  "    WIC_LICENSE_LOCATION     : Location of the WIC license file on the local machine."
  log  ""
  bold "OPTIONS"
  log  "    -h : Show this help message."
  log  "    -n : Dry run (do not execute any commands)."
  log  "    -v : Verbose mode (show commands that are being executed)."
}

execute_on_jetson()
{
  cmd="$1"
  execute "ssh -t $JETSON_USER@$JETSON_IP $cmd"
}


# ======
# Script
# ======

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

JETSON_USER=${@:$OPTIND:1}
JETSON_IP=${@:$OPTIND+1:1}
WIC_SDK_ZIP_LOCATION=${@:$OPTIND+2:1}
WIC_LICENSE_LOCATION=${@:$OPTIND+3:1}

check_argument_not_empty "JETSON_USER" "$JETSON_USER"
check_argument_not_empty "JETSON_IP" "$JETSON_IP"
check_is_valid_ip "$JETSON_IP"
check_argument_not_empty "WIC_SDK_ZIP_LOCATION" "$WIC_SDK_ZIP_LOCATION"
check_file_exists "$WIC_SDK_ZIP_LOCATION"
check_argument_not_empty "WIC_LICENSE_LOCATION" "$WIC_LICENSE_LOCATION"
check_file_exists "$WIC_LICENSE_LOCATION"

PACKAGES=( \
  cmake \
  htop \
  ninja-build \
  valgrind \
  libjpeg-dev \
)

GSTREAMER_PACKAGES=( \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-bad1.0-dev \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gstreamer1.0-doc \
  gstreamer1.0-tools \
  gstreamer1.0-x \
  gstreamer1.0-alsa \
  gstreamer1.0-pulseaudio \
)

info "Installing necessary packages..."
execute_on_jetson "sudo apt-get update"
execute_on_jetson "sudo apt-get install -y ${PACKAGES[*]}"

info "Installing GStreamer 1.0..."
execute_on_jetson "sudo apt-get install -y ${GSTREAMER_PACKAGES[*]}"

info "Installing WIC SDK..."
execute_on_jetson "mkdir -p /tmp/sdk_archive"
execute "scp $WIC_SDK_ZIP_LOCATION jetson@$JETSON_IP:/tmp/sdk_archive/"
execute_on_jetson "unzip /tmp/sdk_archive/WIC_SDK_ARM_1.1.0.zip -d /tmp/sdk_archive/"
execute_on_jetson "sudo /tmp/sdk_archive/WIC_SDK_ARM/WIC_SDK-Linux_aarch64-1.1.0.run"

info "Adding camera license file..."
execute "scp $WIC_LICENSE_LOCATION jetson@$JETSON_IP:/opt/workswell/wic_sdk/"

info "Adding line to .bashrc to source the environment variables..."
execute "echo 'source /opt/workswell/wic_sdk/set_env_variables' | ssh jetson@$JETSON_IP 'cat >> ~/.bashrc'"

info "Done!"
success
