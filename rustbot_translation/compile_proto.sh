#!/bin/bash
PROTOBUF_PATH="/workingcopy/protoc-3.1.0-linux-x86_64/include/google/protobuf/"

protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/timestamp.proto
protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/Header.proto

protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/Point.proto
protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/Quaternion.proto
protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/Pose.proto
protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/PoseWithCovariance.proto

protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/Vector3.proto
protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/Twist.proto
protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/TwistWithCovariance.proto

protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/NavSatFix.proto
protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/Odometry.proto
protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/Image.proto
protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/PointField.proto
protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/PointCloud2.proto
protoc -I=./msgs -I=$HOME$PROTOBUF_PATH --python_out=src/ msgs/SEVData.proto




