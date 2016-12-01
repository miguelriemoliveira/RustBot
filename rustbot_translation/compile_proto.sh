#!/bin/bash
PROTOBUF_PATH="/workingcopy/protoc-3.1.0-linux-x86_64/include/google/protobuf/"

protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/timestamp.proto
protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/Header.proto
protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/Pixel.proto
protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/Image.proto
protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/PointField.proto
protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/PointXYZRGB.proto
protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/PointCloud2.proto
protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/SEVData.proto




