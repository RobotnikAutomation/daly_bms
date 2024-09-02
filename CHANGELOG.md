# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added
- Helm chart in `setup-container` to setup all containers files.
- Created helm chart postscript to create non-YAML files.
- Containerized setup process, eliminating the need for helm or yq on the host system.
- `setup-container.yaml` file for centralized configuration of relevant variables.
- Changelog checks in GitHub Actions workflows.
- Remove of staging image from the registry when the pr is closed.
- Added github action check-changes that allows to really execute some workflows if there are relevant changes (if thera aren't the workflow succeds)
- Added some variables to manage the compose settings run flavor template
- Added debug folders on values.yaml avoiding to have it hardcoded

### Changed
- Changed set-version as github action instead of repeating everytime required
- Now the all containers images are obtanined from local mirror if they are available
- Migrated from cmake to pure python package

### Fixed
- Now comply with python style

## [0.1.1] - 2024-08-28


## [0.1.0] - 2024-08-23

### Added
- ROS2 migration and restructuring
  - Migrated from ROS1 to ROS2 structure
  - Separated ROS code from pure Python implementation
  - Updated launch file configuration
- Development and build improvements
  - DEB generator workflow and package generation functionality
  - Test workflow for continuous integration
  - Simple non crash on execution test workflow for continuous integration
  - Devcontainer configuration
  - Debug container for development purposes
- Docker support
  - Added Docker files for running the application
- Enhanced logging and configuration
  - Log level configuration and setup in launch file

### Changed
- Code quality and standards
  - Improved code to comply with PEP standards
  - Moved mapping attributes from summary to dataclasses
- Performance optimization
  - Reduced data frequency to 2Hz to match BMS 1Hz output
- Documentation updates
  - Improved documentation of ROS code
  - Updated documentation with communication data
  - Improved Readme and changelog
- Project structure
  - Moved to package folder structure

### Fixed
- Data handling and parsing
  - Corrected voltage measurement method
  - Fixed serial reading with proper handling
  - Corrected temperature reading
  - Resolved issues with data parsing

### Removed
- Dependencies
  - Removed dependency on rcomponent

## [1.0.0] - 2023-06-02

### Changed
- First ROS2 Version


