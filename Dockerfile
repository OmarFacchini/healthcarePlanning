FROM aiplanning/planutils:latest

# Install solvers and tools
RUN planutils install -y val
RUN planutils install -y ff
RUN planutils install -y metric-ff
RUN planutils install -y enhsp
RUN planutils install -y popf
RUN planutils install -y optic
RUN planutils install -y tfd
RUN planutils install -y downward
RUN apt-get update && apt-get install -y openjdk-8-jre-headless locales
RUN locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8 && LC_ALL=en_US.UTF-8
    
# Modify the configuration file to enable hostfs, i.e. use the host file system
RUN perl -pi.bak -e "s/mount hostfs = no/mount hostfs = yes/g" /etc/apptainer/apptainer.conf

ENV PATH=$PATH:/root/.planutils/bin
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

# Default command to start bash
CMD ["/bin/bash"]
