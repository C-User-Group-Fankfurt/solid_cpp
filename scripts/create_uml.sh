#!/bin/bash

set -e

cd cmake-build-debug
cmake ..
cd ..
~/software/clang-uml/usr/local/bin/clang-uml

BRANCH=`git rev-parse --abbrev-ref HEAD`
PUML_NAME="docs/diagrams/${BRANCH[0]}.puml"
mv docs/diagrams/driving_system_class_diagram.puml "$PUML_NAME"

plantuml "$PUML_NAME" -tsvg
