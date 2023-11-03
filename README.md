# beginner_tutorials
Beginner tutorials repo for ENPM808X

# Build command


```colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers $(nproc)```

```bash
# run clang-format

  clang-format -i --style=Google $(find . -name *.cpp -o -name *.hpp | grep -vE -e "^(./build/|./install/|./log/)")

# run cppcheck

  mkdir results -p && cppcheck --enable=all --std=c++11 -I include/ --suppress=missingInclude --inline-suppr $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) &> results/cppcheck

# run cpplint

  mkdir results -p && cpplint --filter="-legal/copyright" $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) &> results/cpplint

```