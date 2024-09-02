{{- define "image" -}}
{{- $site := .site | default .Values.images.sites.local -}}
{{- $registry := "" -}}
{{- $project := "" -}}
{{- range .Values.sites -}}
  {{- if eq .name $site -}}
    {{- $registry = .registry -}}
    {{- $project = .project -}}
  {{- end -}}
{{- end -}}
{{ $registry }}/{{ $project }}/{{ .Values.images.name }}:{{ .Values.ros.general.distro }}-{{ .Values.images.version }}
{{- end -}}

{{- define "image-base" -}}
{{- $site := .site | default .Values.images.sites.local -}}
{{- $registry := "" -}}
{{- $base_image := "" -}}
{{- range .Values.sites -}}
  {{- if eq .name $site -}}
    {{- $registry = .registry -}}
    {{- $base_image = .base_image -}}
  {{- end -}}
{{- end -}}
{{ $registry }}/{{ $base_image }}
{{- end -}}


{{- define "header" -}}
# Copyright {{ now | date "2006" }} Robotnik Automation S.L.
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
{{- end -}}