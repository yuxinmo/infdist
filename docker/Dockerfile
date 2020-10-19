FROM ubuntu:focal

ENV TZ=Europe/Vienna
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -qy \
    cython3 \
    iputils-ping \
    net-tools \
    iproute2 \
    python3 \
    python3-dev \
    python3-plotly \
    python3-psutil \
    python3-numpy \
    pkg-config \
    sqlite3 \
    python3-pip \
    g++ \
    git \
    npm \
    vim \
    libnss3 \
    libatk-bridge2.0-0 \
    libgdk-pixbuf2.0-0 \
    libgtk-3-0 \
    libgbm-dev \
    libxss1 \
    xvfb

RUN pip3 install --upgrade pip

RUN git clone https://github.com/zeroos/ns3-infdist.git /ns3_workspace
RUN pip3 install -r /ns3_workspace/requirements.txt
RUN git clone https://github.com/zeroos/infdist.git /infdist
RUN pip3 install -r /infdist/infdist/requirements.txt

RUN cd /ns3_workspace/ns-allinone-3.30/ns-3.30 && python3 ./waf configure && python3 ./waf

# Set user and group
# ARG user=appuser
# ARG group=appuser
# ARG uid=1000
# ARG gid=1000
# RUN groupadd -g ${gid} ${group}
# RUN useradd -u ${uid} -g ${group} -s /bin/sh -m ${user}
# 
# 
RUN cd /root && npm install electron@6.1.4 orca

# Switch to user
# USER ${uid}:${gid}

COPY ./bashrc /root/.bashrc
COPY ./orca.sh /orca.sh
RUN chmod 777 /orca.sh
