This project implements a simplified driving system to explain the SOLID
design principles.

# Build
```bash
cmake . -B build
cmake --build build --config Release --target all --parallel
```

# UML via clang-uml
See official documentation [here](https://github.com/bkryza/clang-uml/tree/master/docs).
Configuration can be found in [.clang-uml](.clang-uml)

## Install Dependencies

```bash
sudo apt install ccache cmake libyaml-cpp-dev clang-12 libclang-12-dev libclang-cpp12-dev plantuml
git clone https://github.com/bkryza/clang-uml
cd clang-uml
git checkout v0.4.1
make LLVM_VERSION=12 release
make install DESTDIR=~/software/clang-uml/
```

## Generate UML

```bash
~/software/clang-uml/usr/local/bin/clang-uml
plantuml docs/diagrams/driving_system_class_diagram.puml -tsvg
```
