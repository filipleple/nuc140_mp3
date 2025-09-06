FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
  build-essential \
  gcc-arm-none-eabi \
  binutils-arm-none-eabi \
  make \
  python3 \
  git \
  && apt-get clean

WORKDIR /workdir
