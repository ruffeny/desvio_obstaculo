# Use a imagem do Ubuntu 20.04 como base
FROM ubuntu:20.04

# Configuração inicial do ambiente
ENV DEBIAN_FRONTEND=noninteractive

# Instalar pacotes do sistema
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    gnupg \
    lsb-release \
    software-properties-common \
    curl \
    build-essential \
    cmake \
    git   \
    python3-tk

# Adicionar o repositório ROS Noetic
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Instalar o ROS Noetic
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# Inicializar o ambiente ROS
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Criar workspace do ROS
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/

#go to src directory
RUN cd /root/catkin_ws/src

# Clonar o pacote desvio_obstaculo para a pasta src
RUN git clone https://github.com/ruffeny/desvio_obstaculo.git /root/catkin_ws/src/desvio_obstaculo

#clonar arquivos do clearpathrobotics urdf
RUN git clone https://github.com/akhil22/velodyne_simulator.git /root/catkin_ws/src/velodyne_simulator
RUN git clone -b noetic-devel https://github.com/clearpathrobotics/LMS1xx.git /root/catkin_ws/src/LMS1xx
RUN git clone https://github.com/clearpathrobotics/fath_pivot_mount_description.git /root/catkin_ws/src/fath_pivot_mount_description
RUN git clone https://github.com/ros-drivers/flir_camera_driver.git /root/catkin_ws/src/flir_camera_driver


RUN rm -rf /root/catkin_ws/src/flir_camera_driver/spinnaker_camera_driver/


RUN mv /root/catkin_ws/src/velodyne_simulator/velodyne_description /root/catkin_ws/src/
RUN rm -rf /root/catkin_ws/src/velodyne_simulator/



#go back to catkin_ws workspace
RUN cd /root/catkin_ws/


# Compilar o pacote desvio_obstaculo usando catkin_make
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Adicionar chave e repositório do VS Code
RUN curl -sSL https://packages.microsoft.com/keys/microsoft.asc | apt-key add - \
    && echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list

# Instalar o VS Code
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        code \
        libx11-xcb1 \
        libxcb-dri3-0 \
        libdrm2 \
        libgbm1 \
        libasound2 \
        libatspi2.0-0 \
        libgtk-3-0 \
    && rm -rf /var/lib/apt/lists/*


# Definir o ponto de entrada padrão para o shell
CMD ["bash"]

RUN apt-get update
RUN apt-get install -y python3-tk

#xhost +local:root
#docker build -t ruffeny:husky .
#docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" ruffeny:husky



