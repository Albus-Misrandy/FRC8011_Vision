# 2025 FRC 8011 Vision Scheme
## AprilTag recognition usage
After downloading the environment, run the following command in the main. py directory:

<code data-enlighter-language="raw" class="EnlighterJSRAW">python main.py</code>

If you want to run the code on jetson nano, when you turn on the jetson nano, you could run the code by following the steps.

<code data-enlighter-language="raw" class="EnlighterJSRAW">1.Use the shortcut "Crtl+Alt+T" to open the terminal.</code>

<code data-enlighter-language="raw" class="EnlighterJSRAW">2.cd AprilTag_Positioning</code>

<code data-enlighter-language="raw" class="EnlighterJSRAW">3.python main.py</code>

Note: The red camera should be connected to the USB port in the upper right corner, and the black camera should be connected to the USB port in the lower right corner.

## Control Board UI usage



## Unresolved issues
### 1.Currently only half of the pose is solved
Currently, AprilTags with IDs 17 to 22 can calculate camera coordinates and yaw angle, but those with IDs 6 to 11 have not been completed.
### 2.Only camera with red mounts can be used
Since the cameras' mounts have not ye been made, it can only use the one that in red.But both cameras should be connected to jetson nano in a certain order. 