#!/bin/bash

# Create virtual environment if it doesn't exist
if [ ! -d "robot_env" ]; then
    echo "Creating virtual environment..."
    python3 -m venv robot_env
fi

# Activate virtual environment
echo "Activating virtual environment..."
source robot_env/bin/activate

# Set PYTHONPATH
echo "Setting PYTHONPATH..."
export PYTHONPATH=$PYTHONPATH:$(pwd)

# Install required package
echo "Installing pyserial..."
pip3 install pyserial

echo "Setup complete! Virtual environment is activated and ready to use."
echo "You can now run your robot scripts." 