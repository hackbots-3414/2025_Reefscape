#!/bin/bash
for fullpath in autons/*; do
  if [[ -d "$fullpath" ]]; then
    echo "[ INFO ] Skipping directory $fullpath"
    continue
  fi
  filename=`basename $fullpath`
  if [[ $filename = "auto-builder.py" ]]; then
    continue
  fi
  name="${filename%.*}" # bash magic from stack overflow
  auto_gen $fullpath > src/main/deploy/pathplanner/autos/$name.auto
  echo "[ CREATE ] $name.auto"
done
