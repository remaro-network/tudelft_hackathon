FROM rezenders/ignition:foxy

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y\
  git \
  sudo \
  tzdata \
  locales \
  lsb-release \
  build-essential \
  keyboard-configuration \
	&& rm -rf /var/lib/apt/lists/

# Set the locale
RUN sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && \
    locale-gen
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# create new user
RUN adduser --disabled-password \
--gecos '' docker

#  Add new user docker to sudo group
RUN adduser docker sudo

# Ensure sudo group users are not asked for a password when using
# sudo command by ammending sudoers file
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> \
/etc/sudoers

# now we can set USER to the user we just created
USER docker
ENV DEBIAN_FRONTEND noninteractive

WORKDIR /home/docker/
RUN git clone https://github.com/ArduPilot/ardupilot.git -b Sub-4.1 --recurse
WORKDIR /home/docker/ardupilot

# RUN git config --global --add safe.directory /ardupilot
RUN USER=docker Tools/environment_install/install-prereqs-ubuntu.sh -y
RUN . ~/.profile

ENV PATH=/opt/gcc-arm-none-eabi-6-2017-q2-update/bin:$PATH
ENV PATH=$PATH:/home/docker/ardupilot/Tools/autotest
ENV PATH=/usr/lib/ccache:$PATH

WORKDIR /home/docker/ardupilot
RUN ["/bin/bash","-c","./waf configure && make sub"]
WORKDIR /home/docker/ardupilot/ArduSub

COPY entrypoint.sh /home/docker/entrypoint.sh
ENTRYPOINT ["/home/docker/entrypoint.sh"]
