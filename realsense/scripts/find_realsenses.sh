#!/bin/bash

# set bashrc
num=`cat ~/.bashrc | grep 'REALSENSE_SERIAL_NUM' | wc -l`
if [ "$num" -gt "0" ]; then
  echo ".bashrc already contains some REALSENSE_SERIAL_NUM. If you changed realsense, delete the REALSENSE_SERIAL_NUM variables and rerun this script."
fi

# set zshrc
num=`cat ~/.zshrc | grep 'REALSENSE_SERIAL_NUM' | wc -l`
if [ "$num" -gt "0" ]; then
  echo ".zshrc already contains some REALSENSE_SERIAL_NUM. If you changed realsense, delete the REALSENSE_SERIAL_NUM variables and rerun this script."
fi

declare -i a=0
rs-enumerate-devices | grep 'Serial Number' | grep -v 'Asic' | grep -Eo '[0-9]{1,}' | while read -r number ; do
  # echo $number


  num=`cat ~/.bashrc | grep "$number" | wc -l`
  if [ "$num" -lt "1" ]; then

    # set bashrc
    echo "export REALSENSE_SERIAL_NUM$a=\"$number\"" >> ~/.bashrc
    echo "adding REALSENSE_SERIAL_NUM$a to .bashrc."
  else
    echo ".bashrc already contains REALSENSE_SERIAL_NUMBER: $number"
    
  fi

  set zshrc
  num=`cat ~/.zshrc | grep "$number" | wc -l`
  if [ "$num" -lt "1" ]; then

    echo "export REALSENSE_SERIAL_NUM$a=\"$number\"" >> ~/.zshrc
    echo "adding REALSENSE_SERIAL_NUM$a to .zshrc."
  else
    echo ".zshrc already contains REALSENSE_SERIAL_NUMBER: $number"

  fi
  ((a=a+1))
done

