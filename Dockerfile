FROM ubuntu:24.04

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    patch \
    perl \
    python3 \
    python3-pip \
    python3-dev \
    gfortran \
    libblas-dev \
    liblapack-dev \
    coinor-libipopt-dev \
    libmetis-dev \
    pybind11-dev \
    && rm -rf /var/lib/apt/lists/*

RUN cmake --version

# get casadi
RUN git clone https://github.com/casadi/casadi.git \
    && cd casadi \
    && mkdir build \
    && cd build \
    && cmake -DWITH_IPOPT=ON -DWITH_BUILD_IPOPT=ON -DWITH_BUILD_MUMPS=ON -DWITH_BUILD_METIS=ON -DWITH_FATROP=ON -DWITH_BUILD_FATROP=ON -DWITH_BUILD_REQUIRED=ON .. 

RUN cd casadi/build \
    && make
RUN cd casadi/build \ 
    && make install

# get pybind11
# RUN pip install --no-cache-dir pybind11 --break-system-packages

# get python packages
RUN apt update && apt install -y python3-matplotlib python3-shapely

# get pybind11 from source
# mkdir build
# cd build
# cmake ..
# make check -j 4

# get PMP
# RUN git clone git@gitlab.kuleuven.be:meco/arena/parametric-motion-planner.git \
#     && cd parametric-motion-planner \
#     && mkdir build \
#     && cd build \
#     && cmake ..