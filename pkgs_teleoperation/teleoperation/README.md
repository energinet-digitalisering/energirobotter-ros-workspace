# Teleoperation

Teleoperation capabilities for the robot Elrik, with Vuer.

The robot will tracking the position of the hands, but have a constant rotation of the grippers pointing forward.

The tracking is relative to the head only in the z-axis, so it does not matter how tall a person is, or if they are sitting down, the tracking will be the same. It is not relative the x/y-axis, as it felt unnatural to be able to control the position of the arms by moving the head.

## Usage

1. Plug headset into computer with USB-C cable. From headset, allow Android Debugging (if not chosen to always allow).

2. Start Vuer app, note port.

3. Run command for [reverse port forwarding](https://medium.com/@lazerwalker/how-to-easily-test-your-webvr-and-webxr-projects-locally-on-your-oculus-quest-eec26a03b7ee) (example with port 8012):

    ```
    adb reverse tcp:8012 tcp:8012
    ```

4. On headset, go to address in browser: `http://localhost:8012`