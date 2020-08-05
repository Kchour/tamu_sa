# tamu_sa

# Tests
- Do `rostest your_package_name your_test.launch` to run a particular test

# TODO
- Make tests automatic via a test suite or to run upon `catkin build`
- Remove folders in `tests`, seems redundant
- Test package for phoenix indepedence
- Add additional unit tests (test)
- Separate map-updating from path-planning node: Create a "map-update" node and let
the planner "latch" subscribe to it

# miscNotes
- Don't forget to add `<build_depend>rostest</build_depend>` to package.xml
- Also add `<test_depend>rosunit</test_depend>`
- Auto run tests by adding to CMakelists.txt`catkin_add_nosetests(path/to/my_test.py)`
- (link)[http://wiki.ros.org/unittest]
- Also read (link)[http://wiki.ros.org/rostest/Writing]
- use `reset` after abnormal exit from `pdb`

# Debugging info
- use `rostest --text your_test.launch` to debug outputs, even the debugger can be used

