# Installation
- Clone this repo to your catkin workspace, i.e. `../catkin_example/src/`
- Build with catkin tools: `catkin build`
- Source your workspace: `source `../catkin_example/devel/setup.bash`

# Running Tests
- Do `rostest your_package_name your_test.launch` to run a particular test
   i.e. `rostest tamu_sa testObstacles.launch`
- You can also go directly to a test folder and invoke a unittest directly
   i.e. `rostest testObstacles.launch`

# TODO
- Make tests automatic via a test suite or to run upon `catkin build`
- Remove folders in `tests`, seems redundant
- Test package for phoenix indepedence
- Add additional unit tests (test) for more coverage
- Separate map-updating from path-planning node: Create a "map-update" node and let
the planner "latch" subscribe to it
- Figure out how to use sphinx with rosdoc_lite

# Misc Notes
- Don't forget to add `<build_depend>rostest</build_depend>` to package.xml
- Also add `<test_depend>rosunit</test_depend>`
- Auto run tests by adding to CMakelists.txt`catkin_add_nosetests(path/to/my_test.py)`
- More info on unittest: [link][http://wiki.ros.org/unittest]
- Also read [link](http://wiki.ros.org/rostest/Writing)
- use `reset` after abnormal exit from `pdb`

# Debugging your tests
- use `rostest --text your_test.launch` to debug outputs, even the debugger can be used

