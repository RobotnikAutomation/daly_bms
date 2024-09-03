{{- define "build.base" -}}
{{ include "header" .}}
#
# Description: {{ .Values.description }} build {{ if eq .flavor "cd"}}for continous delivery{{- end }} compose
#
name: {{ .Values.compose.name }}
services:
  container:
    image: >-
      {{- if eq .flavor "test"}}
      {{ include "image" . }}-test
      {{- else if eq .flavor "debs"}}
      {{ include "image" . }}-debs
      {{- else}}
      {{ include "image" . }}
      {{- end }}
    build:
      context: ../..
      dockerfile: container/builder/Dockerfile
      {{- if eq .flavor "test"}}
      target: test
      {{- end }}
      {{- if eq .flavor "debs"}}
      target: builder
      {{- end }}
      {{- if eq .flavor "cd"}}
      pull: true
      no_cache: true
      network: host
      {{- end }}
      args:
        base_image: {{ include "image-base" . }}
        image_base_version: {{ .Values.builder.image_base_version }}
        ros_distro: {{ .Values.ros.general.distro }}
        version: {{ .Values.images.version }}
        alpine_version: {{ .Values.builder.alpine_version }}
        ros_mirror: {{ .Values.builder.ros_mirror }}
        {{- $mirrorSuffix := "" -}}
        {{- $mirrorRegistry := "" -}}
        {{- $siteName := "" -}}
        {{- if eq .flavor "cd" -}}
          {{- $siteName = .Values.images.sites.cd -}}
        {{- else -}}
          {{- $siteName = .Values.images.sites.local -}}
        {{- end -}}
        {{- range .Values.sites -}}
          {{- if eq .name $siteName -}}
            {{- if .mirror.enabled -}}
              {{- $mirrorSuffix = .mirror.suffix -}}
              {{- $mirrorRegistry = .registry -}}
            {{- end -}}
          {{- end -}}
        {{- end }}
        alpine_mirror: {{ if and $mirrorSuffix $mirrorRegistry }}{{ $mirrorRegistry }}/{{ $mirrorSuffix }}/{{ else }}""{{ end }}
    {{- if eq .flavor "test"}}
    environment:
      GEN_COMMAND: "test_workspace.sh"
      STARTUP_TYPE: generic
    {{- end }}
    {{- if eq .flavor "debs"}}
    environment:
      GEN_COMMAND: "find $${USER_WORKSPACE}/debs -name *.deb -exec mv {} /data \\;"
      STARTUP_TYPE: generic
    volumes:
      - source: ../debs
        target: /data
        type: bind
    {{- end }}
{{- end -}}
