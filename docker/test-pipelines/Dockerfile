# Get Ubuntu Xenial base image
FROM debian:bullseye

# Set Debian frontend to avoid error messages during building
ARG DEBIAN_FRONTEND=noninteractive

# Install packages
RUN apt-get update && apt-get install --no-install-recommends -y \
    apt-utils \
    software-properties-common

# Install Gstreamer 1.0
RUN apt-get install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-pulseaudio

# Copy over the pipeline script
COPY receive-pipeline.sh /
