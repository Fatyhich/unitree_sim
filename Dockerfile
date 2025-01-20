FROM ubuntu:20.04

RUN addgroup --gid 1000 --system oversir \
 && adduser  --uid 1000 --system \
            --ingroup oversir \
            --home /home/oversir \
            --shell /bin/bash oversir

RUN chown -R oversir:oversir /home/oversir

ARG DEBIAN_FRONTEND=noninteractive

RUN ln -snf /usr/share/zoneinfo/Etc/UTC /etc/localtime && echo "Etc/UTC" > /etc/timezone

RUN DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get update && \
    DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get install -y tzdata && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    wget \
    curl \
    sudo \
    nano \
    python3 \
    python3-pip \
    xorg \
    x11-apps \
    xserver-xorg-core \
    git \
    && rm -rf /var/lib/apt/lists/*

ENV DISPLAY=:0
ENV XAUTHORITY=/tmp/.Xauthority

RUN usermod -aG sudo oversir
RUN echo 'oversir ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER oversir
WORKDIR /home/oversir

CMD ["/bin/bash"]


# display the clock
# CMD ["xclock"]
