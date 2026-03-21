FROM nvidia/cuda:12.2.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# 1. Install System Dependencies & FIX CUDA CONFLICTS
# Note: We removed 'cuda-libraries-dev-12-2' to avoid version conflicts.
# We only force reinstall 'cuda-nvcc-12-2' which is the critical missing piece.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential software-properties-common wget unzip curl git cmake \
    libopencv-dev libboost-all-dev libusb-1.0-0-dev libprotobuf-dev protobuf-compiler \
    libeigen3-dev libceres-dev libhdf5-dev hdf5-tools libglew-dev libglfw3-dev \
    libcanberra-gtk-module ffmpeg libgtest-dev libgmock-dev \
    libogre-1.12-dev libimgui-dev libfreetype-dev \
    python3.10-venv python3.10-dev python3-pip \
    sudo usbutils vim \
    libglvnd-dev libgl1-mesa-dev libegl1-mesa-dev \
    nano \
    ca-certificates && \
    # --- FORCE RESTORE CUDA COMPILER ---
    apt-get -o Dpkg::Options::="--force-overwrite" install -y --reinstall cuda-nvcc-12-2 && \
    # Ensure the generic symlink exists
    ln -sfn /usr/local/cuda-12.2 /usr/local/cuda && \
    # Cleanup
    rm -rf /var/lib/apt/lists/*

# 2. Install Python Dependencies
RUN pip3 install --upgrade pip && \
    pip3 install numpy h5py opencv-python

# 3. Install Node.js
RUN curl -fsSL https://deb.nodesource.com/setup_16.x | bash - \
    && apt-get install -y nodejs

WORKDIR /tmp

# 4. Install Sophus
RUN wget https://github.com/strasdat/Sophus/archive/1.22.10.zip && \
    unzip 1.22.10.zip && \
    cd Sophus-1.22.10 && \
    mkdir build && cd build && \
    cmake -DSOPHUS_USE_BASIC_LOGGING=ON .. && \
    cmake --build . --target install && \
    cd /tmp && rm -rf Sophus*

# 5. Install Pybind11
RUN wget https://github.com/pybind/pybind11/archive/v2.11.0.zip && \
    unzip v2.11.0.zip && \
    cd pybind11-2.11.0 && \
    mkdir build && cd build && \
    cmake .. -DPYBIND11_TEST=OFF && \
    cmake --build . && \
    cmake --build . --target install && \
    cd /tmp && rm -rf pybind11*

# 6. Prepare Source Code
COPY metavision_open_full_5.1.1.tar.gz /tmp/
COPY metavision_sdk_advanced_sources_5.1.1.tar.gz /tmp/

WORKDIR /opt/metavision_sdk

# 7. Smart Merge
RUN mkdir -p /tmp/open_extract /tmp/adv_extract && \
    tar -xvf /tmp/metavision_open_full_5.1.1.tar.gz -C /tmp/open_extract && \
    tar -xvf /tmp/metavision_sdk_advanced_sources_5.1.1.tar.gz -C /tmp/adv_extract && \
    if [ $(ls -1 /tmp/open_extract | wc -l) -eq 1 ]; then cp -rf /tmp/open_extract/*/* .; else cp -rf /tmp/open_extract/* .; fi && \
    if [ $(ls -1 /tmp/adv_extract | wc -l) -eq 1 ]; then cp -rf /tmp/adv_extract/*/* .; else cp -rf /tmp/adv_extract/* .; fi && \

# 8. Python Setup
RUN pip install --no-cache-dir -r utils/python/requirements_openeb.txt && \
    find . -name "requirements_sdk_advanced.txt" -exec pip install --no-cache-dir -r {} \;

# 9. Compile the SDK
WORKDIR /opt/metavision_sdk/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release \
    -DCOMPILE_METAVISION_STUDIO=ON \
    -DUSE_SOPHUS=ON \
    -DUSE_TORCH=OFF \
    -DPYTHON3_SITE_PACKAGES=/usr/local/lib/python3.10/dist-packages && \
    cmake --build . --config Release -- -j 4 && \
    cmake --build . --target install

# 10. Final Environment Setup
ENV UDEV=1

# 11. SETUP USER & PERMISSIONS
RUN useradd -m -s /bin/bash studio_user && \
    usermod -aG video,plugdev,root studio_user && \
    chown -R studio_user:studio_user /opt/metavision_sdk && \
    find /opt/metavision_sdk -name chrome-sandbox -exec chown root:root {} \; && \
    find /opt/metavision_sdk -name chrome-sandbox -exec chmod 4755 {} \;

USER studio_user

# 12. FORCE PATH GLOBALLY (Crucial for nvcc)
ENV PATH="/usr/local/cuda/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/lib:/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"

# (Optional) Keep the setup script for Metavision specific tools
RUN echo "source /opt/metavision_sdk/build/utils/scripts/setup_env.sh" >> /home/studio_user/.bashrc
RUN pip install numpy h5py opencv-python psutil

CMD ["/bin/bash"]
