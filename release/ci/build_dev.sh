#!/usr/bin/env bash

set -e
# git diff --name-status origin/release3-staging | grep "^A" | less

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

cd $DIR

BUILD_DIR=/data/media/openpilot-dev
SOURCE_DIR="$(git rev-parse --show-toplevel)"
DEV_BRANCH="dev-c3"


## set git identity
#source $DIR/identity.sh
#export GIT_SSH_COMMAND="ssh -i /data/gitkey"

# Let's bring the submodules!
git submodule update --remote --init --recursive

# Build
docker compose -f ${SOURCE_DIR}/docker-compose.yml build
docker compose -f ${SOURCE_DIR}/docker-compose.yml up
