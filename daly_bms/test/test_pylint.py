# Copyright 2024 Robotnik Automation S.L.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
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
# @maintanier Guillem Gari  <ggari@robotnik.es> Robotnik Automation S.L.

"""
This module contains unit tests for the package, including a pylint check.

It can be run as a standalone script or through unittest/colcon.
"""

import os
from pylint import epylint as lint


def find_python_modules_and_files(root_dir):
    """
    Find Python modules and standalone Python files in the given directory.

    Parameters
    ----------
    root_dir : str
               The root directory to search in.

    Returns
    -------
    modules : list
             list of directories containing __init__.py
    standalone_files : list
                       list of .py files in directories without __init__.py

    """
    modules = []
    standalone_files = []

    for dirpath, _, filenames in os.walk(root_dir):
        if '__init__.py' in filenames:
            modules.append(dirpath)
        else:
            py_files = [
                os.path.join(dirpath, f)
                for f in filenames
                if f.endswith('.py')
            ]
            standalone_files.extend(py_files)

    return modules, standalone_files


def run_pylint(targets):
    """
    Run pylint on the given targets.

    Parameters
    ----------
    targets : list
              List of file paths or directory paths to run pylint on.

    Returns
    -------
    total_output : str
                   The combined output from pylint for all targets.
    return_code : int
                  0 if all checks passed,
                  1 if issues were found.

    """
    total_output = ""
    return_code = 0

    for target in targets:
        total_output += f"Running pylint on: {target}\n"
        pylint_stdout, pylint_stderr = lint.py_run(target, return_std=True)
        output = pylint_stdout.getvalue() + pylint_stderr.getvalue()

        if "Your code has been rated at 10.00/10" not in output:
            return_code = 1
            total_output += output

    return total_output, return_code


def run_test_pylint():
    """
    Execute the main pylint testing function.

    This function performs the following steps:
    1. Gets the current working directory.
    2. Finds all Python modules and standalone files in the directory.
    3. Runs pylint on all found Python files.
    4. Prints the pylint output.
    5. Returns a code indicating whether all checks passed.

    Returns
    -------
    int
        Return code: 0 if all pylint checks passed, 1 if issues were found
        or no Python files were found to check.

    Prints
    ------
    str
        Pylint output and summary messages to stdout.

    """
    current_dir = os.getcwd()
    modules, standalone_files = find_python_modules_and_files(current_dir)

    all_targets = modules + standalone_files
    if not all_targets:
        print("No Python modules or files found.")
        return 1

    pylint_output, return_code = run_pylint(all_targets)

    print("\nPylint Output:")
    print(pylint_output)

    if return_code == 0:
        print("\nAll checks passed successfully!")
    else:
        print("\nPylint found issues. Please review the output above.")

    return return_code


def test_pylint():
    """
    Run all pylint tests and assert their success.

    This function serves as a wrapper for run_test_pylint(). It calls
    run_test_pylint() and asserts that the return code is 0, indicating
    that all pylint checks passed successfully.

    Raises
    ------
    AssertionError
        If run_test_pylint() returns a non-zero value, indicating that
        code style errors or warnings were found.

    Notes
    -----
    This function is designed to be used with pytest or a similar testing
    framework. The assertion allows the test to fail if any pylint issues
    are detected, making it suitable for inclusion in automated testing
    pipelines.

    """
    return_code = run_test_pylint()
    assert return_code == 0, 'Found code style errors / warnings'
