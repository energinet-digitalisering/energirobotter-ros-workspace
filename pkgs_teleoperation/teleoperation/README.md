# Teleoperation

Teleoperation capabilities for the robot Elrik, with Vuer.

## Usage

1. Plug headset into computer with USB-C cable. From headset, allow Android Debugging (if not chosen to always allow).

2. Start Vuer app, note port.

3. Run command for [reverse port forwarding](https://medium.com/@lazerwalker/how-to-easily-test-your-webvr-and-webxr-projects-locally-on-your-oculus-quest-eec26a03b7ee) (example with port 8013):

    ```
    adb reverse tcp:8013 tcp:8013
    ```

4. On headset, go to address in browser: `http://localhost:8013`