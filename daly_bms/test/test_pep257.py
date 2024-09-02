# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This module contains a test function for checking PEP 257 compliance.

The test function uses pytest markers to indicate that it's a linter
and PEP 257 test. It runs the ament_pep257 tool on the codebase and
asserts that there are no code style errors or warnings related to
docstring conventions.
"""

import pytest
from ament_pep257.main import main


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """
    Run ament_pep257 on the codebase and check for PEP 257 compliance.

    This function uses the ament_pep257 tool to check for proper
    docstring conventions in the codebase as specified in PEP 257.
    It asserts that the return code from ament_pep257 is 0, indicating
    no errors or warnings were found.

    If any docstring style issues are detected, the test will fail and
    display an error message.

    Raises
    ------
    AssertionError
        If any PEP 257 style errors or warnings are found.

    """
    return_code = main(argv=['.', 'test'])
    assert return_code == 0, 'Found code style errors / warnings'
