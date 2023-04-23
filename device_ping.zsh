#!/bin/zsh

device=/dev/ttyACM1

while read -r line < $device; do
  # $line is the line read, do something with it
  # which produces $result
  echo $result > $device
done

