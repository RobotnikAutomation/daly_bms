# Copyright 2024 Robotnik Automation S.L.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# @maintanier Guillem Gari  <ggari@robotnik.es> Robotnik Automation S.L.

"""
Setup configuration for the daly_bms package.

This script reads the version from package.xml and sets up the package
using setuptools. It includes metadata such as the package name, version,
maintainer information, and entry points for console scripts.

The script defines utility functions to extract information from the
package.xml file, including version, description, license, and
author/maintainer details. These extracted details are then used to
configure the package setup using setuptools.

Functions:
    get_value_from_tag_in_package_xml:
        Extracts a value from a specific XML tag.
    get_people_from_package_xml:
        Extracts people's names and emails from XML tags.

The setup configuration includes package metadata, dependencies, and entry
points for console scripts.
"""
import os
from glob import glob
import xml.etree.ElementTree as ET
from setuptools import setup


def get_value_from_tag_in_package_xml(
    tag,
    file_path='package.xml',
):
    """
    Extract a value of given key from the package.xml file.

    This function parses the package.xml file and retrieves the value
    associated with the specified XML tag.

    Parameters
    ----------
    tag : str
        The XML tag to search for in the package.xml file.
    file_path : str, optional
        The path to the package.xml file. Default is 'package.xml'.

    Returns
    -------
    str
        The value associated with the specified tag.

    Raises
    ------
    RuntimeError
        If there's an error parsing the file or if the tag is not found.
    ValueError
        When the tag is not found.

    Notes
    -----
    This function is used to extract various metadata from the package.xml
    file, such as package name, version, description, and license.

    """
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
        element = root.find(tag)
        if element is None:
            raise ValueError(f"{tag} tag not found in package.xml")
        return element.text.strip()
    except Exception as not_found:
        raise RuntimeError(
            f"Failed to parse version from {file_path}: {not_found}"
        ) from not_found


def get_people_from_package_xml(tag, file_path='package.xml'):
    """
    Extract People name and email from the package.xml file.

    This function parses the package.xml file and retrieves the names and
    email addresses of people associated with the specified tag (e.g.,
    'author', 'maintainer').

    Parameters
    ----------
    tag : str
        The XML tag to search for in the package.xml file (e.g., 'author',
        'maintainer').
    file_path : str, optional
        The path to the package.xml file. Default is 'package.xml'.

    Returns
    -------
    tuple of str
        A tuple containing two strings:
        - A comma-separated list of names
        - A comma-separated list of email addresses

    Raises
    ------
    RuntimeError
        If there's an error parsing the file or if the tag is not found.

    Notes
    -----
    This function is used to extract author and maintainer information
    from the package.xml file.

    """
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
        people = []
        for person in root.findall(tag):
            name = person.text.strip()
            email = person.get('email')
            people.append((name, email))
        people_str = ", ".join(name for name, _ in people)
        email_str = ", ".join(email for _, email in people)
        return people_str, email_str
    except Exception as not_found:
        raise RuntimeError(
            f"Failed to parse {tag} from {file_path}: {not_found}"
        ) from not_found


PACKAGE_NAME = get_value_from_tag_in_package_xml('name')
DESCRIPTION = get_value_from_tag_in_package_xml('description')
LICENSE = get_value_from_tag_in_package_xml('license')
VERSION = get_value_from_tag_in_package_xml('version')
AUTHOR, AUTHOR_EMAIL = get_people_from_package_xml('author')
MAINTAINER, MAINTAINER_EMAIL = get_people_from_package_xml('maintainer')

setup(
    name=PACKAGE_NAME,
    version=VERSION,
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (
            os.path.join('share', PACKAGE_NAME, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))
        ),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
        'dalybms==0.5'
    ],
    zip_safe=True,
    author=AUTHOR,
    author_email=AUTHOR_EMAIL,
    maintainer=MAINTAINER,
    maintainer_email=MAINTAINER_EMAIL,
    keywords=['ROS'],
    description=DESCRIPTION,
    license=LICENSE,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'daly_bms = daly_bms.daly_bms_node:main'
        ],
    },
)
