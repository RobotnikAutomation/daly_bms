# Copyright 2017 Open Source Robotics Foundation, Inc.
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
This module contains a test function for checking code style using flake8.

The test function uses pytest markers to indicate that it's a flake8 and
linter test. It runs flake8 on the codebase and asserts that there are no
code style errors or warnings.
"""

import pytest
from ament_flake8.main import main_with_errors


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """
    Run flake8 on the codebase and check for code style errors/warnings.

    This function uses the ament_flake8 tool to run flake8 checks on the
    codebase. It asserts that the return code from flake8 is 0, indicating
    no errors or warnings were found.

    If any code style issues are detected, the test will fail and display
    the number of issues along with their descriptions.

    Raises
    ------
        AssertionError: If any flake8 errors or warnings are found.

    """
    return_code, errors = main_with_errors(argv=[])
    assert return_code == 0, (
        f'Found {len(errors)} code style errors / warnings:\n'
        f'{chr(10).join(errors)}'
    )
