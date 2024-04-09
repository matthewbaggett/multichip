FROM ubuntu:latest

LABEL maintainer="Matthew Baggett <matthew@baggett.me>"

ENV DEBIAN_FRONTEND="teletype" \
    TERM=xterm-256color \
    LANG=C.UTF-8

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

#ENV PATH="/app:/app/bin:/app/vendor/bin:${PATH}"
WORKDIR /build

RUN apt-get -qq update && \
    apt-get install -yqq \
    git \
    ssh \
    build-essential \
    make \
    gcc \
    gcc-multilib \
    g++-multilib \
    module-assistant \
    expect \
	g++ \
	gawk \
	texinfo \
	libssl-dev \
	bison \
	flex \
	fakeroot \
	cmake \
	unzip \
	gperf \
	autoconf \
	device-tree-compiler \
	libncurses5-dev \
	pkg-config \
	bc \
	python-is-python3 \
	passwd \
	openssl \
	openssh-server \
	openssh-client \
	vim \
	file \
	cpio \
	rsync
 
COPY luckfox-sdk /build
RUN  cd /build/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/ && \
      source env_install_toolchain.sh && \
     cat  /etc/profile
RUN ls -lah
RUN build.sh