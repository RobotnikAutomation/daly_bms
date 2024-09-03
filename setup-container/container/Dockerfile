# Copyright 2024 Robotnik Automation S.L.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    1. Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#
#    2. Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#    3. Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Maintanier: Guillem Gari  <ggari@robotnik.es> Robotnik Automation S.L.
#
# Description: Container template processor.
#              This container provides the required tools to process
#              Helm charts and perform YAML transformations (using yq).
#              It allows users to obtain necessary container files for
#              running, building, testing, etc., without installing
#              any tools other than Docker on the host machine.

ARG version="use-args"
ARG image_tag="use-args"
ARG image_mirror="use-args"


FROM  ${image_mirror}alpine:${image_tag} as base
LABEL maintainer="Guillem Gari <ggari@robotnik.es>" \
      org.opencontainers.image.title="setup-container" \
      org.opencontainers.image.version="${version}" \
      org.opencontainers.image.description="Container template processor" \
      org.opencontainers.image.source="https://github.com/RobotnikAutomation" \
      org.opencontainers.image.vendor="Robotnik Automation S.L." \
      org.opencontainers.image.licenses="BSD-3-Clause" \
      org.opencontainers.image.authors="Guillem Gari <ggari@robotnik.es>"

FROM  base as downloader

# Install bash
RUN apk add --no-cache bash curl wget openssl

# Install helm
RUN true \
    && curl \
      -fsSL -o get_helm.sh \
      https://raw.githubusercontent.com/helm/helm/master/scripts/get-helm-3 \
    && chmod 700 get_helm.sh \
    && ./get_helm.sh \
    && rm get_helm.sh \
    && which helm

# Install yq
RUN wget \
      https://github.com/mikefarah/yq/releases/latest/download/yq_linux_amd64 \
      -O /usr/local/bin/yq \
    && chmod +x /usr/local/bin/yq

FROM  base

# Create a non-root user
ARG user_name=helm

RUN true \
    && addgroup -g 1001 $user_name \
    && adduser -u 1001 -D -G $user_name $user_name

RUN --mount=type=bind,from=downloader,source=/usr/local/bin/yq,target=/tmp/yq \
    --mount=type=bind,from=downloader,source=/usr/local/bin/helm,target=/tmp/helm \
      true \
      && apk add --no-cache bash openssl \
      && cp /tmp/yq /usr/local/bin/yq \
      && cp /tmp/helm /usr/local/bin/helm


# Set the default command to bash
CMD ["/bin/bash"]


# Switch to non-root user
USER $user_name
ENV USERNAME $user_name

# Set the working directory to the user's home
WORKDIR /home/$user_name

RUN --mount=type=bind,\
source=./setup-container,\
target=/tmp/setup-container \
    true \
    && mkdir -p setup-container \
    && mkdir -p output \
    && chmod 755 output \
    && cp -R /tmp/setup-container /$HOME \
    && cp setup-container/values.yaml values.yaml \
    && cd setup-container \
    && rm -rf container \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/setup-container \
    && true

CMD [\
      "/usr/local/bin/helm", \
      "template", \
      "./setup-container", \
      "--values", \
      "./values.yaml", \
      "--output-dir", \
      "./output", \
      "--post-renderer", \
      "./setup-container/scripts/post-process.sh", \
      "--post-renderer-args", \
      "./output" \
]