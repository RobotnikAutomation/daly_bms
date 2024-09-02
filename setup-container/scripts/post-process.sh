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
#Description: Helm Chart Template Processor
#
#             This script processes a Helm chart template directory and creates
#             non-YAML files listed in non-yaml-files.yaml.
#
#             It performs these main tasks:
#
#             1. Sets up environment and checks for required tools (yq)
#             2. Cleans up previous output files in the target directory
#             3. Extracts the chart name from Chart.yaml
#             4. Moves files from templates directory to target directory
#             5. Removes source lines from YAML files, preserving structure
#             6. Processes non-YAML files listed in non-yaml-files.yaml
#             7. Cleans up temporary files and directories
#
#             Usage as post-process script with helm template command:
#             helm template ./[CHART_FOLDER] \
#               --values [VALUES FILE] \
#               --output-dir [TARGET_DIR] \
#               --post-renderer [SCRIPT_PATH]/post-process.sh \
#               --post-renderer-args [TARGET_DIR]
#
#             If TARGET_DIR is not provided, it defaults to "output".


set -e

# Set the helm chart template directory
DOCKER_TEMPLATE="$(dirname "${BASH_SOURCE[0]}")"
DOCKER_TEMPLATE="$(dirname "$DOCKER_TEMPLATE")"

# Set the target directory, defaulting to "output" if not provided
TARGET_DIR=${1:-output}

# Extract the chart name from Chart.yaml
if [ -f "${DOCKER_TEMPLATE}/Chart.yaml" ]; then
    CHART_NAME=$(yq e '.name' "${DOCKER_TEMPLATE}/Chart.yaml")
    if [ -z "${CHART_NAME}" ]; then
        echo "Error: Unable to extract chart name from ${DOCKER_TEMPLATE}/Chart.yaml"
        exit 1
    fi
else
    echo "Error: ${DOCKER_TEMPLATE}/Chart.yaml not found"
    exit 1
fi

# Clean up previous files
echo "Cleaning up previous files in ${TARGET_DIR}"
if [ -d "${TARGET_DIR}" ]; then
    # Get directories to clean from the templates folder
    template_dir="${DOCKER_TEMPLATE}/templates"
    if [ -d "$template_dir" ]; then
        # Use a loop to get directory names
        for dir in "$template_dir"/*; do
            if [ -d "$dir" ]; then
                dir_name=$(basename "$dir")
                if [ -d "${TARGET_DIR}/${dir_name}" ]; then
                    echo "Removing ${TARGET_DIR}/${dir_name}"
                    rm -rf "${TARGET_DIR}/${dir_name}"
                fi
            fi
        done
    else
        echo "Warning: Template directory ${template_dir} not found. Skipping cleanup."
    fi
fi

# Set the source directory
SRC_DIR="${TARGET_DIR}/${CHART_NAME}/templates"

# Check if yq is installed
if ! command -v yq &> /dev/null
then
    echo "yq is not installed. Please install it first."
    echo "You can install it using:"
    echo "wget https://github.com/mikefarah/yq/releases/latest/download/yq_linux_amd64 -O /usr/bin/yq && chmod +x /usr/bin/yq"
    exit 1
fi

echo "Processing chart: ${CHART_NAME}"

# Move all files and directories from templates to target directory, excluding docker-template
if [ -d "${SRC_DIR}" ]; then
    echo "Moving files from ${SRC_DIR} to ${TARGET_DIR} (excluding ${DOCKER_TEMPLATE})"
    find "${SRC_DIR}" -mindepth 1 -maxdepth 1 ! -name "${DOCKER_TEMPLATE}" | while read -r item; do
        base_name=$(basename "${item}")
        if [ -d "${item}" ]; then
            # If it's a directory, move its contents
            mkdir -p "${TARGET_DIR}/${base_name}"
            mv -n "${item}"/* "${TARGET_DIR}/${base_name}/" 2>/dev/null || true
        else
            # If it's a file, move it if it doesn't exist in the target
            mv -n "${item}" "${TARGET_DIR}/" 2>/dev/null || true
        fi
    done

    # Remove the templates directory if it's empty
    if [ -z "$(ls -A "${SRC_DIR}")" ]; then
        rmdir "${SRC_DIR}" && echo "Removed empty templates directory" || echo "Failed to remove templates directory"
    fi

    # Remove the chart directory if it's empty
    if [ -z "$(ls -A "${TARGET_DIR}/${CHART_NAME}")" ]; then
        rmdir "${TARGET_DIR}/${CHART_NAME}" && echo "Removed empty chart directory" || echo "Failed to remove chart directory"
    fi
else
    echo "Source directory ${SRC_DIR} not found. Skipping file move."
fi

# Remove source line from YAML files while preserving the starting "---" line
echo "Removing source lines from YAML files"
cd "${TARGET_DIR}" || exit
while IFS= read -r -d '' file
do
    if [ "${file}" != "non-yaml-files.yaml" ] && [[ "${file}" != ./${DOCKER_TEMPLATE}/* ]]; then
        # Use sed to remove the source line but keep the first line (---)
        sed -i '/^# Source:/d' "${file}"
        # Remove the chart name from the beginning of the file path in the content
        sed -i "s|${CHART_NAME}/templates/||g" "${file}"
        echo "Processed: ${file}"
    fi
done < <(find . -name '*.yaml' -type f -print0)

# Process non-yaml-files.yaml
if [ -f "non-yaml-files.yaml" ]; then
    # Get the number of files
    file_count=$(yq e '.files | length' non-yaml-files.yaml)

    # Process each file entry
    for ((i=0; i<file_count; i++)); do
        original_path=$(yq e ".files[$i].path" non-yaml-files.yaml)
        # Remove the chart name from the beginning of the path
        path=${original_path#"${CHART_NAME}/templates/"}
        content=$(yq e ".files[$i].content" non-yaml-files.yaml)

        # Skip files in docker-template folder
        if [[ "${path}" == ${DOCKER_TEMPLATE}/* ]]; then
            echo "Skipping file in ${DOCKER_TEMPLATE}: ${path}"
            continue
        fi

        # Create directory if it doesn't exist
        mkdir -p "$(dirname "${path}")"

        # Write content to file
        echo -e "${content}" > "${path}"

        echo "Created file: ${path}"
    done

    # Remove non-yaml-files.yaml
    rm non-yaml-files.yaml
    echo "Removed non-yaml-files.yaml"
else
    echo "Warning: non-yaml-files.yaml not found. Skipping non-YAML file creation."
fi

echo "removing files in ${CHART_NAME}"
rm -rf "${CHART_NAME}"

echo "All operations completed."
