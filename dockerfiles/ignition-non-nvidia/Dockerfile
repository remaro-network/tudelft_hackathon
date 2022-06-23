FROM rezenders/ignition:hackathon

USER root
RUN apt update && apt install -y --no-install-recommends \
  libgl1-mesa-glx \
  libgl1-mesa-dri \
	&& rm -rf /var/lib/apt/lists/
USER docker
