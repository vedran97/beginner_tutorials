# beginner_tutorials

Beginner tutorials repo for ENPM808X<br>
This package was built and tested for ros2-humble on an ubuntu 22.04 distro.<br>

## Install process

1. Clone this repo in a ros2 workspace : ```git clone https://github.com/vedran97/beginner_tutorials.git```
2. Install dependencies by running ```rosdep install -i --from-path src --rosdistro humble -y``` in the root of your workspace.
3. Build the package with the following command:

## Build commands

1. ```colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers $(nproc)```

## Running the code

1. Once the code is built in a ros2 ws, source the workspace.
2. to run the publisher: ```ros2 run beginner_tutorials talker```. Topic for publishing: ```Problem_Pub```
3. to run the subscriber: ```ros2 run beginner_tutorials listener```
4. using launch file: ```ros2 launch beginner_tutorials launch_pub_sub.py```
5. if using vscode terminal , before running rqt type this:```unset GTK_PATH```

## Instructions to run the cpptools

```bash
# run clang-format

  clang-format -i --style=Google $(find . -name *.cpp -o -name *.hpp | grep -vE -e "^(./build/|./install/|./log/)")

# run cppcheck

  mkdir results -p && cppcheck --enable=all --std=c++17 -I include/ --suppress=missingInclude --inline-suppr $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) &> results/cppcheck

# run cpplint

  mkdir results -p && cpplint  --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) &> results/cpplint

```
