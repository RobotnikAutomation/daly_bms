# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added
- Self tests for improved reliability
- Docker files for containerized application deployment
- Helm chart in `setup-container` for comprehensive container setup
- `setup-container.yaml` for centralized configuration management
- Debug container to facilitate development
- DEB package generation workflow and functionality
- Helm chart postscript for non-YAML file creation
- Continuous integration workflows:
  - Test workflow
  - Simple non-crash execution test
- Containerized setup process, removing host system dependencies on helm and yq
- GitHub Actions workflows:
  - Changelog checks
  - `check-changes` for conditional workflow execution
- Variables for managing compose settings run flavor template
- Debug folders in `values.yaml` to reduce hardcoding
- Added `common.repos.yaml` to install required public repos
- Added devcontainer

### Changed
- Upgraded to dalybms version 0.5.0
- Restructured to package folder layout
- Enhanced launch file configuration
- Improved ROS code documentation
- Implemented `set-version` as a GitHub Action
- Prioritized local mirror for container image retrieval
- Migrated from CMake to pure Python package
- Expanded and refined README and CHANGELOG

### Fixed
- Refactored code to adhere to PEP standards

### Removed
- Staging image removal from registry upon PR closure


## [1.0.0] - 2023-06-02

### Changed
- First ROS2 Version


