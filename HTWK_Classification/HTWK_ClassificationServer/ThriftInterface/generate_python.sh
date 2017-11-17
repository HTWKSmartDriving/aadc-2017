#!/bin/sh

GEN_NAME="ThriftInterface"

OUTPUT_DIR="."
mkdir -p "${OUTPUT_DIR}"

thrift --gen py --out "${OUTPUT_DIR}" "${GEN_NAME}.thrift"

echo "Finished! your thrift files have been generated in ${OUTPUT_DIR}"
