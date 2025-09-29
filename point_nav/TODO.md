# TODO
1. [ ] Make turns smoother, Turn first then move (diagonal movement) or just curve to diagonal. Replace square movemnt (straight then turn then straight movement)
2. [ ] Goal point queing/reading file or list of points. Make an arg that lets you queue points that the rover follows in a sequential order. Maybe even change default so queue is default and an arg to override current goal but keep queue plus another arg to override queue.
3. [ ] Emergency stop capability
4. [ ] Turn throttle into an arg option instead of default. With another option to toggle instead of just for one point
5. [ ] (Optional Extra Task) Make an app or webview with UI access to all features for easier QOL controls and debugging

# DONE
1. [x] Enable reverse driving with a `allow_reverse` flag to turn it on and off. Make sure it has the correct reverse steering
2. [x] Add forward/backward/steering caps (0.5/0.5/0.7) to make the rover slower and easier to save if something goes wrong
3. [x] Add `set_goal` CLI helper with `count`/`delay`/`topic` options for better QOL
4. [x] Move default params into YAML to make changes to defaults more consistent
