# beginner_tutorials
Beginner tutorials repo for ENPM808X

# Build command


```colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers $(nproc)```

```bash
# run clang-format

  clang-format -i --style=Google $(find . -name *.cpp -o -name *.hpp | grep -vE -e "^(./build/|./install/|./log/)")

# run cppcheck

  mkdir results -p && cppcheck --enable=all --std=c++17 -I include/ --suppress=missingInclude --inline-suppr $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) &> results/cppcheck

# run cpplint

  mkdir results -p && cpplint  --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) &> results/cpplint

```