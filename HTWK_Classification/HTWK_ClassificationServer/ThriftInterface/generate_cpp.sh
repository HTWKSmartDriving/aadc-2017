#!/bin/sh

GEN_NAME="ThriftInterface"

OUTPUT_DIR="cpp"
mkdir -p "${OUTPUT_DIR}"

thrift --gen cpp --out "${OUTPUT_DIR}" "${GEN_NAME}.thrift"

echo "Finished! your thrift files have been generated in ${OUTPUT_DIR}"
