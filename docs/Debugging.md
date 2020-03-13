### How to debug properly

#### Python

###### Interactive mode

Run your python script in interactive mode

```bash
python3 -i myscript.py
```

###### Segmentation Fault

In case you're getting nasty errors like

```
[INFO] [tree]: waiting for action server ... [NavigateToPose][FromConstant/Move]
[INFO] [tree]: ... connected to action server [NavigateToPose][FromConstant/Move]
Segmentation fault
```

Insert this snippet on top of you faulty script

```python
import faulthandler; faulthandler.enable()
```

Now run your script with

```bash
python3 -X faulthandler myscript.py
```

You now get a Stacktrace 🎉
```
[INFO] [tree]: waiting for action server ... [NavigateToPose][FromConstant/Move]
[INFO] [tree]: ... connected to action server [NavigateToPose][FromConstant/Move]
Fatal Python error: Segmentation fault

Thread 0x0000007f53fff1f0 (most recent call first):
  File "/usr/lib/python3.6/threading.py", line 299 in wait
  File "/usr/lib/python3.6/threading.py", line 551 in wait
  File "/usr/lib/python3.6/threading.py", line 1180 in run
  File "/usr/lib/python3.6/threading.py", line 916 in _bootstrap_inner
  File "/usr/lib/python3.6/threading.py", line 884 in _bootstrap

Current thread 0x0000007fab3dd010 (most recent call first):
  File "/opt/ros/eloquent/lib/python3.6/site-packages/rclpy/node.py", line 1158 in create_subscription
  File "/opt/ros/eloquent/lib/python3.6/site-packages/py_trees_ros/subscribers.py", line 119 in setup
  File "/opt/ros/eloquent/lib/python3.6/site-packages/py_trees/trees.py", line 93 in visited_setup
  File "/opt/ros/eloquent/lib/python3.6/site-packages/py_trees/trees.py", line 112 in setup
  File "/opt/ros/eloquent/lib/python3.6/site-packages/py_trees/trees.py", line 322 in setup
  File "/opt/ros/eloquent/lib/python3.6/site-packages/py_trees_ros/trees.py", line 427 in setup
  File "actions.py", line 235 in main
  File "actions.py", line 261 in <module>
Segmentation fault
```

*Happy Debugging...*


#### Linux C/C++

Coming soon...
