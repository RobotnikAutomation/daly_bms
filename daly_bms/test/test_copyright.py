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
This module contains a test function for checking copyright notices.

The test function uses pytest markers to indicate that it's a copyright
and linter test. It runs the ament_copyright tool on the codebase and
asserts that there are no copyright-related errors.
"""

import pytest
from ament_copyright.main import main


@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    """
    Run ament_copyright on the codebase and check for copyright errors.

    This function uses the ament_copyright tool to check for proper
    copyright notices in the codebase. It asserts that the return code
    from ament_copyright is 0, indicating no errors were found.

    If any copyright issues are detected, the test will fail and display
    an error message.

    Raises
    ------
    AssertionError
        If any copyright errors are found.

    """
    return_code = main(argv=['.', 'test'])
    assert return_code == 0, 'Found copyright errors'
