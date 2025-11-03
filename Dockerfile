FROM ubuntu:xenial

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Use Tsinghua mirror for faster downloads
RUN sed -i 's/archive.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list && \
    apt-get update

# Install dependencies
RUN apt-get install -y \
    python \
    python-dev \
    g++ \
    scons \
    zlib1g-dev \
    m4 \
    swig \
    git \
    vim \
    gcc-5-aarch64-linux-gnu \
    libgoogle-perftools-dev \
    make \
    && rm -rf /var/lib/apt/lists/*


# Set the working directory
WORKDIR /gem5

# Default command
CMD ["/bin/bash"]
