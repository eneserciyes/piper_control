#!/bin/bash

# Script to create a conda environment with:
# -  Python 3.10
# -  ipykernel
# -  piper_control (editable install)
#
# Usage:
#   ./conda_env.sh
# This script assumes that conda is already installed and available in the PATH.
# Check if conda is installed with:
#   conda --version
#
# This script can be used to create a conda env for local development or testing.

ENV_NAME="piper_control_env"
YAML_FILE="piper_control_env.yaml"


# Create the conda environment
if conda env update -p "$ENV_NAME" --file "$YAML_FILE" --prune; then
  echo "Conda environment '$ENV_NAME' with Python 3.10 created successfully."
else
  echo "Failed to create conda environment '$ENV_NAME'."
  exit 1
fi

# Initialize conda.
if eval "$(conda shell.bash hook)"; then
  echo "Conda initialized successfully."
else
  echo "Failed to initialize conda."
  exit 1
fi

# Activate the conda environment.
if conda activate "$ENV_NAME"; then
  echo "Conda environment '$ENV_NAME' activated successfully."
else
  echo "Failed to activate conda environment '$ENV_NAME'."
  exit 1
fi

# Install can-utils for
if eval "$(sudo apt install can-utils)"; then
  echo "can-utils installed successfully."
else
  echo "Failed install can-utils."
  exit 1
fi

# Install this project, editable.
if pip install -e .; then
  echo "piper_control installed successfully in conda environment '$ENV_NAME'."
else
  echo "Failed to install piper_control in conda environment '$ENV_NAME'."
  exit 1
fi
