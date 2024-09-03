{{- define "run.base" -}}
{{ include "header" .}}
#
# Description: {{ .Values.description }} {{ if eq .flavor "debug"}}debug{{- else }}run{{- end }} compose
#
name: {{ .Values.compose.name }}
services:
  {{ .Values.compose.services.run.name }}:
    environment:
      HEALTHCHECK_NODES: /$${ROBOT_ID}/$${NODE_NAME}
      ROBOT_BMS_PORT: {{ .Values.ros.bms.port }}
      ROBOT_ID: {{ .Values.ros.general.robot_id }}
      ROS_DOMAIN_ID: "{{ .Values.ros.general.domain_id }}"
      {{- if eq .flavor "debug"}}
      LOG_LEVEL: DEBUG
      STARTUP_TYPE: generic
      GEN_COMMAND: sleep infinity
      SIGNAL_DESTROY: SIGTERM
      {{- else }}
      LOG_LEVEL: {{ .Values.ros.general.log_level }}
      {{- end }}
    {{- if .Values.compose.services.run.group_add }}
    group_add:
{{ toYaml .Values.compose.services.run.group_add | indent 6 }}
    {{- end }}
    image: {{ include "image" . }}
    {{- if .Values.compose.services.run.network_mode }}
    network_mode: {{ .Values.compose.services.run.network_mode }}
    {{- end }}
    {{- if hasKey .Values.compose.services.run "privileged" }}
    privileged: {{ .Values.compose.services.run.privileged }}
    {{- end }}
    {{- if hasKey .Values.compose.services.run "restart" }}
    restart: {{ .Values.compose.services.run.restart }}
    {{- end }}
    volumes:
{{ toYaml .Values.compose.services.run.volumes | indent 6 }}
      - type: bind
        source: ./.bash_history
        target: >-
          /home/robot/.bash_history
    {{- if eq .flavor "debug"}}
    {{- $distro := .Values.ros.general.distro }}
    {{- $pythonVersion := "" }}
    {{- range .Values.ros.distros }}
      {{- if eq .name $distro }}
        {{- $pythonVersion = .python.version }}
      {{- end }}
    {{- end }}
    {{- range .Values.builder.local }}
      {{- if .debug }}
        {{- $source := .source }}
        {{- range .folders }}
          {{- if eq .target.base "ros" }}
            {{- if eq .target.type "code" }}
      - type: bind
        source: ../../{{ $source }}/{{ .source }}
        target: >-
          /opt/ros/{{ $distro }}/lib/python{{ $pythonVersion }}/site-packages/{{ $source }}/
            {{- else if eq .target.type "share" }}
      - type: bind
        source: ../../{{ $source }}/{{ .source }}
        target: >-
          /opt/ros/{{ $distro }}/share/{{ $source }}/{{ .source }}
            {{- end }}
          {{- end }}
        {{- end }}
      {{- end }}
    {{- end }}
    {{- end }}
{{- end -}}
