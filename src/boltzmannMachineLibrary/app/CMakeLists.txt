cmake_minimum_required(VERSION 2.6)

project(boltzmannMachineLibrary)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.

add_subdirectory(bmlApplication)

## make all applications show up as single target
icub_app_all()



