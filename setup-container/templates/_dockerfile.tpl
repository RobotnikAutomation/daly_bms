{{- define "dockerfile.localCopy" -}}
{{- range .Values.builder.local }}
#Copy {{ .source }}
COPY --chown=$USER_NAME \
  {{ .source }}/  \
  {{ .target }}

{{ end }}
{{- end -}}
{{- define "pip.install" -}}
RUN \
{{- range $index, $req := .Values.builder.requirements }}
{{- if and (eq $req.type "pip") (eq $req.install "pip") }}
{{- range $reqIndex, $reqFile := $req.requirements }}
    --mount=\
type=bind,\
source={{ $reqFile }}/requirements.txt,\
target=/tmp/requirements-{{ printf "%02d" (add $reqIndex 1) }}.txt \
{{- end }}
{{- end }}
{{- end }}
true \
    && pip install \
{{- range $index, $req := .Values.builder.requirements }}
{{- if and (eq $req.type "pip") (eq $req.install "pip") }}
{{- range $reqIndex, $reqFile := $req.requirements }}
               -r /tmp/requirements-{{ printf "%02d" (add $reqIndex 1) }}.txt \
{{- end }}
{{- if $req.additional }}
{{- range $req.additional }}
              {{ . }} \
{{- end }}
{{- end }}
{{- end }}
{{- end }}
    && true
{{- end -}}



{{- define "dockerfile.requirements" -}}
#{{ .name }}
RUN \
{{- if eq .install "local" }}
    --mount=\
type=bind,\
from=packages,\
source=/data/debs,\
target=/tmp/debs \
{{- end }}
    true \
    && if \
        timeout 2 curl -IsS http://${ros_mirror} &>/dev/null; \
        then \
        sed -i \
            "s#packages.ros.org#${ros_mirror}#" \
            /etc/apt/sources.list.d/ros-latest.list ;\
        fi \
{{- if eq .install "local" }}
    && echo "deb [trusted=yes] file:///tmp/debs/ ./" | tee /etc/apt/sources.list.d/debs.list \
    && apt-get update \
{{- else }}
    && apt-fast update \
{{- end }}
    && apt-fast install -q -y \
        --no-install-recommends \
{{- range .packages }}
{{- if eq $.type "ros" }}
        ros-${ROS_DISTRO}-{{ . }} \
{{- else }}
        {{ . }} \
{{- end }}
{{- end }}
    && sed -i "s#${ros_mirror}#packages.ros.org#" /etc/apt/sources.list.d/ros-latest.list \
    && apt-get clean -q -y \
    && apt-get autoremove -q -y \
    && rm -rf /var/lib/apt/lists/* \
{{- if eq .install "local" }}
    && rm /etc/apt/sources.list.d/debs.list \
{{- end }}
    && true
{{- end -}}



{{- define "dockerfile" -}}
{{ include "header" .}}
ARG base_image="use-args"
ARG ros_distro="use-args"
ARG image_base_version="use-args"
ARG version="use-args"
ARG output_path=/tmp
ARG output_pkg=${output_path}/deb.pkgs.tar
ARG alpine_version="use-args"
ARG alpine_mirror="use-args"


FROM  ${base_image}:${ros_distro}-base-${image_base_version} as base
LABEL maintainer="Guillem Gari <ggari@robotnik.es>" \
      org.opencontainers.image.title="{{ .Values.images.name }}" \
      org.opencontainers.image.version="${version}" \
      org.opencontainers.image.description="{{ .Values.description }}" \
      org.opencontainers.image.source="{{ .Values.repo }}" \
      org.opencontainers.image.vendor="Robotnik Automation S.L." \
      org.opencontainers.image.licenses="BSD-3-Clause" \
      org.opencontainers.image.authors="Guillem Gari <ggari@robotnik.es>"

FROM ${base_image}:${ros_distro}-builder-${image_base_version} as builder-base

USER root

ENV DEBIAN_FRONTEND=noninteractive
ARG ros_mirror="user-args"

{{- range .Values.builder.requirements }}
{{- if eq .name "builder" }}
{{ include "dockerfile.requirements" . }}
{{- end }}
{{- end }}

USER $USER_NAME

RUN --mount=type=bind,\
source=./container/builder/repos/common.repo.yaml,\
target=/tmp/common.repo.yaml,ro \
    vcs import \
        --input /tmp/common.repo.yaml \
        --shallow \
        ./src/

USER $USER_NAME

{{ include "dockerfile.localCopy" . }}

{{ include "pip.install" . }}

FROM builder-base as test

RUN compile_workspace.sh

RUN test_workspace.sh

FROM builder-base as builder

RUN generate_debs.sh


ARG output_pkg
RUN tar -cvzf ${output_pkg} ./debs

FROM ${base_image}:${ros_distro}-builder-${image_base_version} AS compressed

ARG output_path
ARG output_pkg
COPY --from=builder ${output_pkg} ${output_path}/builder.pkgs.tar.gz

RUN find ${output_path} -name '*.pkgs.tar.gz' -exec tar -xvzf {} -C ${output_path} \;

# Generate Packages.gz
RUN cd ${output_path}/debs && dpkg-scanpackages . | gzip -9c > Packages.gz

# Compress all
RUN cd ${output_path} && tar -cvzf ${output_pkg} ./debs


# Output setup
FROM ${alpine_mirror}alpine:${alpine_version} AS packages
RUN mkdir -p /data
WORKDIR /data
ARG output_pkg
COPY --from=compressed ${output_pkg} /data/debs.pkgs.tar.gz
ARG pkg_path=/data/debs.pkgs.tar.gz
RUN cd /data && tar -xf $pkg_path --strip-components=1

FROM base

USER root

ENV DEBIAN_FRONTEND=noninteractive
ARG ros_mirror="user-args"

{{ range .Values.builder.requirements }}
{{- if eq .name "run" }}
{{ include "dockerfile.requirements" . }}
{{ end }}
{{- end }}

{{ include "pip.install" . }}

{{ range .Values.builder.requirements }}
{{- if eq .name "local" }}
{{ include "dockerfile.requirements" . }}
{{ end }}
{{- end }}

USER ${USER_NAME}

# Set environment variables
{{- range $index, $env := .Values.builder.default_environment }}
ENV {{ $env.name | printf "%-20s" }} {{ if $env.value }}"{{ $env.value }}"{{ else }}"INFO"{{ end }}
      {{- end }}
{{- end -}}
