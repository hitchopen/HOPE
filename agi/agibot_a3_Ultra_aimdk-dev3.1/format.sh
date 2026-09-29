#!/bin/bash

set -e

cd $(dirname $0)

# clang-format --version | xargs echo "clang-format version: "
# cmake-format --version | xargs echo "cmake-format version: "
autopep8 --version | xargs echo "autopep8     version: "

# clang-format for C++ and Proto files
#find ./ -regex '.*\.cc\|.*\.cpp\|.*\.h\|.*\.proto' \
#-and -not -regex '.*\.pb\.cc\|.*\.pb\.h' \
#-and -not -path './examples/aimrt-cpp/build/*' \
#-and -not -path './aimdk/build/*' \
#| xargs clang-format-15 -i --style=file
#echo "clang-format done"

# cmake-format for CMake files
#find ./ -regex '.*\.cmake\|.*CMakeLists\.txt$' \
#-and -not -path './examples/aimrt-cpp/build/*' \
#-and -not -path './aimdk/build/*' \
#| xargs cmake-format -c ./.cmake-format.py -i
#echo "cmake-format done"

# autopep8 for Python files
find ./ -regex '.*\.py' \
-and -not -path './examples/aimrt-cpp/build/*' \
-and -not -path './aimdk/build/*' \
| xargs autopep8 -i --global-config ./.pycodestyle
echo "autopep8     done"
