#!/usr/bin/env bash
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd $DIR

DEV_BRANCH=$1
if [ -z "$DEV_BRANCH" ]; then
    DEV_BRANCH="dev-c3-test"
fi

VERSION=$(date '+%Y.%m.%d')
SOURCE_DIR="$(git rev-parse --show-toplevel)"
OUTPUT_DIR=${SOURCE_DIR}/output

echo "Calling to build [${DIR}/build.sh ${SOURCE_DIR} ${OUTPUT_DIR} ${VERSION}]"
$DIR/build.sh "${SOURCE_DIR}" "${OUTPUT_DIR}" "${VERSION}"

echo "Calling to publish [${DIR}/publish.sh ${SOURCE_DIR} ${OUTPUT_DIR} ${DEV_BRANCH} ${VERSION}]"
$DIR/publish.sh "${SOURCE_DIR}" "${OUTPUT_DIR}" "${DEV_BRANCH}" "${VERSION}"
