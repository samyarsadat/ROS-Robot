#!/bin/bash
#  The ROS Robot Project - Raspberry Pi Pico build script for CodeQL analysis.
#  This is a modified version of the script from the Pico Build Action.
#  This workaround is necessary because CodeQL needs run in the same environment as the build script.
#
#  Copyright 2024 Samyar Sadat Akhavi
#  Written by Samyar Sadat Akhavi, 2024.
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https: www.gnu.org/licenses/>.

set -e

# Get arguments
SOURCE_DIR=$1
OUTPUT_DIR="build"
BOARD_NAME="pico"
CMAKE_ARGS=""
MAKEFILES_GENERATOR="Unix Makefiles"

# Validate arguments
if [ -z "$SOURCE_DIR" ]; then
    echo "ERROR: Source directory not provided."
    exit 1
fi

if [ -z "$OUTPUT_DIR" ]; then
    echo "ERROR: Output directory not provided."
    exit 1
fi

if [ -z "$BOARD_NAME" ]; then
    BOARD_NAME="pico"
fi

if [ -z "$MAKEFILES_GENERATOR" ]; then
    MAKEFILES_GENERATOR="Ninja"
fi

# Check if the source directory exists
if [ ! -d "$SOURCE_DIR" ]; then
    echo "ERROR: Source directory does not exist."
    exit 1
fi

# Make paths absolute
OUTPUT_DIR_RELATIVE="$SOURCE_DIR/$OUTPUT_DIR"
SOURCE_DIR="$GITHUB_WORKSPACE/$SOURCE_DIR"
OUTPUT_DIR="$SOURCE_DIR/$OUTPUT_DIR"

# Echo arguments
echo "Configuration:"
echo "SOURCE_DIR=$SOURCE_DIR"
echo "OUTPUT_DIR=$OUTPUT_DIR"
echo "BOARD_NAME=$BOARD_NAME"
echo "CMAKE_ARGS=$CMAKE_ARGS"
echo "MAKEFILES_GENERATOR=$MAKEFILES_GENERATOR"

# Build the project
echo "Generating build files..."
mkdir "$OUTPUT_DIR" && cd "$OUTPUT_DIR"
cmake -DPICO_BOARD="$BOARD_NAME" -S "$SOURCE_DIR" -B "$OUTPUT_DIR" -G "$MAKEFILES_GENERATOR" $CMAKE_ARGS

echo "Building project..."
cd "$OUTPUT_DIR" && make -j$(nproc)