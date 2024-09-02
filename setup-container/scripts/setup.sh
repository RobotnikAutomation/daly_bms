#!/bin/bash
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
# Description: Docker Non-Root File Copy Facilitator
#
#              This script enables a Docker container running as a non-root user
#              to copy files to the host machine without permission issues.
#
#              The tasks perfomened are:
#
#              1. Extracts the Alpine version (IMAGE_TAG) from the YAML file.
#              2. Identifies an available Docker registry mirror (IMAGE_MIRROR).
#              3. Sets host user and group IDs as environment variables.
#              4. Navigates to the container directory.
#              5. Builds and runs the Docker container using docker compose.
#
#              The script ensures proper configuration of the Docker image
#              by dynamically setting the Alpine version and selecting an
#              appropriate registry mirror. It also facilitates running the
#              container with the correct user permissions to avoid ownership
#              conflicts between the container and the host system.


set -e

# Path to your values.yaml file
values_file="values.yaml"

# Function to check if a registry is alive
# Function to check if a registry is alive
check_registry() {
    local registry=$1
    local timeout_duration=0.4
    local max_retries=3
    local retry_count=0

    # Extract hostname and port (default to 443 if not specified)
    local host=${registry%%:*}
    local port=${registry##*:}
    [[ "${port}" == "${host}" ]] && port=443

    while [ $retry_count -lt $max_retries ]; do
        # Try to connect with timeout
        if timeout $timeout_duration bash -c "</dev/tcp/$host/$port" &>/dev/null; then
            return 0  # Success
        fi

        retry_count=$((retry_count + 1))

        # If it's not the last attempt, wait a bit before retrying
        if [ $retry_count -lt $max_retries ]; then
            sleep 0.1  # Short pause between retries
        fi
    done

    return 1  # Failure after all retries
}


# Function to extract IMAGE_TAG
extract_image_tag() {
    local tag
    tag=$(sed -n '/^builder:/,/^[a-zA-Z]/{/^builder:/d; /^[a-zA-Z]/q; /^[[:space:]]*$/d; s/^  //; p}' "$values_file" | awk -F': ' '/alpine_version:/ {print $2}')
    echo "$tag" | tr -d '[:space:]'
}

# Function to extract IMAGE_MIRROR
extract_image_mirror() {
    local temp_file
    temp_file=$(mktemp)

    sed -n '/^sites:/,/^[a-zA-Z]/{/^sites:/d; /^[a-zA-Z]/q; /^[[:space:]]*$/d; s/^  //; p}' "$values_file" | awk '
    BEGIN {
        RS = "-"
        FS = "\n"
        OFS = ","
    }
    {
        name = registry = suffix = ""
        mirror_enabled = "false"

        for (i = 1; i <= NF; i++) {
            split($i, parts, ":")
            key = parts[1]
            value = parts[2]
            gsub(/^[ \t]+|[ \t]+$/, "", key)
            gsub(/^[ \t]+|[ \t]+$/, "", value)

            if (key == "name") name = value
            else if (key == "registry") registry = value
            else if (key == "mirror") in_mirror = 1
            else if (in_mirror && key == "enabled") mirror_enabled = tolower(value)
            else if (in_mirror && key == "suffix") suffix = value
        }

        if (mirror_enabled == "true" && name != "" && registry != "" && suffix != "") {
            print registry, suffix
        }
    }
    ' | while IFS=',' read -r registry suffix; do
        if check_registry "$registry"; then
            echo "${registry}/${suffix}/" > "$temp_file"
        fi
    done

    if [ -s "$temp_file" ]; then
        cat "$temp_file"
    fi

    rm "$temp_file"
}



SCRIPT_PATH="$(readlink -f "$(dirname "${0}")")"
cd "${SCRIPT_PATH}"
cd ../

# Main execution
IMAGE_TAG=$(extract_image_tag)
IMAGE_MIRROR=$(extract_image_mirror)

# Export variables
HOST_UID=$(id -u)
HOST_GID=$(id -g)

export HOST_UID
export HOST_GID
export IMAGE_TAG
export IMAGE_MIRROR

# Print the final values of IMAGE_MIRROR and IMAGE_TAG
echo "UID value: ${HOST_UID}"
echo "GID value: ${HOST_GID}"
echo "IMAGE_TAG value: ${IMAGE_TAG}"
echo "IMAGE_MIRROR value: ${IMAGE_MIRROR}"

cd container
docker compose up --build
