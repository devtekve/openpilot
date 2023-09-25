#!/usr/bin/env bash

set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd $DIR

# Take parameters as arguments
SOURCE_DIR=$1
OUTPUT_DIR=$2

# Check parameters
if [ -z "$SOURCE_DIR" ] || [ -z "$OUTPUT_DIR" ]; then
    echo "Error: No source or output directory provided."
    exit 1
fi

# Let's bring the submodules!
git submodule update --remote --init --recursive

# Build
docker compose -f ${SOURCE_DIR}/docker-compose.yml build
docker compose -f ${SOURCE_DIR}/docker-compose.yml up
