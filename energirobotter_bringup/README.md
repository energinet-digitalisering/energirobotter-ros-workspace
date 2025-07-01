# Energirobotter Bringup

Bringup package for Energirobotter, with different launch scripts for each mode/feature.

This package contains lots of launch files that implements different behaviours/modes, like face following or teleoperation.
The `launch` folder is divided into two folders, for launch files that should be run directly on the robot micorcomputer, and those to be run on a "server", aka. a development computer connected to the same subnet. 

Install dependencies with:

```
rosdep install --from-paths src -y --ignore-src
```

- [Energirobotter Bringup](#energirobotter-bringup)
  - [Teleoperation Vuer](#teleoperation-vuer)
    - [Setup Robot](#setup-robot)
      - [Enable Servo Serial Forwarding](#enable-servo-serial-forwarding)
    - [Setup Visualisation](#setup-visualisation)
    - [Setup VR Headset](#setup-vr-headset)
      - [Wireless](#wireless)
      - [Wired](#wired)
    - [Calibrate and Launch](#calibrate-and-launch)
    - [V1 Demo notes (Danish)](#v1-demo-notes-danish)
  - [Teleoperation Unity](#teleoperation-unity)
    - [Setup VR Headset](#setup-vr-headset-1)
      - [Connected to computer with Unity](#connected-to-computer-with-unity)
      - [Only VR headset](#only-vr-headset)
    - [Setup Robot](#setup-robot-1)
    - [Calibrate and Launch](#calibrate-and-launch-1)
  - [Face Following](#face-following)


## Teleoperation Vuer

[Vuer](https://docs.vuer.ai/en/latest/) is used to interface between a Quest 3 VR headset and the robot control. 

The robot can be controlled wired or wireless, see instructions below. 

The camera can only be served in the headset over a secure connection, for this [ngrok](https://ngrok.com/) is used. `ngrok` is also used for remote teleoperation over different networks. 

### Setup Robot

1. Turn on the robot
2. From terminal on PC SSH into the robot (Elrik example):
   ```
   ssh elrik@192.168.1.105
   ```
3. If using `ngrok`, export your [authtoken from the ngrok dashboard](https://dashboard.ngrok.com/get-started/your-authtoken) as NGROK_AUTHTOKEN in your terminal:
   ```
   export NGROK_AUTHTOKEN=$YOUR_AUTHTOKEN
   ```
4. Run teleoperation on robot with (set appropriate flags):
   ```
   cd energinet/
   shumble
   sw
   ros2 launch energirobotter_bringup teleoperation_vuer.launch.py camera_enabled:=true stereo_enabled:=false ngrok_enabled:=true ik_enabled:=true rviz:=false
   ```
   > Only set `rviz:=true` if a display is connected to the computer. 

   > You can also run the `teleoperation_vuer.launch.py` from a PC and it will work if the robot and computer are on the same subnet. If not on robot set `camera_enabled:=false`. 

#### Enable Servo Serial Forwarding

1. On your phone, connect to the `ESP32_DEV` Wi-Fi network (password is `12345678`)
2. In browser, go to `192.168.4.1`
3. Click button `Start Serial Forwarding` - OBS! Make sure nothing is sending commands to the robot yet
4. Click botton `Stop Serial Forwarding` but don't click `OK`, it's now ready as a stop button if needed


### Setup Visualisation

1. If a display is not connected to the robot, start `RViz` on your computer in a teminal with:
   ```
   rviz2 -d src/energirobotter-ros-workspace/energirobotter_bringup/config/rviz/teleoperation.rviz
   ```


### Setup VR Headset

1. Turn on headset

#### Wireless
2. In the headset's browser, go to the `ngrok` URL shown in the terminal when launching the teleoperation. 

#### Wired
2. Plug USB cable into headset first, then put it on
3. Plug cable into computer, and accept USB connection in the headset (if you miss it, you can find it under notifications)
> If the "`USB-C Port Disabled, water and debris`" message pops up, restart the headset and try again. If that does not work, use the other end of the cable and try again.
4. From terminal on PC, enable reverse port forwarding:
   ```
   adb reverse tcp:8012 tcp:8012
   ```
5. In the headset's browser, go to the `localhost` URL shown in the terminal when launching the teleoperation. 


### Calibrate and Launch

1. In the headset, press "passthrough"
2. Calibrate the view (hold down Meta button on right controller)
3. Verify that tracking is working in RViz
4. Start `servos.launch.py` in a terminal on the robot:
   >Make sure the VR is tracking properly! As soon as the `servos.launch.py` is running, it will start moving!
   ```
   cd energinet/
   shumble
   sw
   ros2 launch energirobotter_bringup servos.launch.py
   ```
   
5. When done with the teleoperation, stop the `servos.launch.py` terminal

---


### V1 Demo notes (Danish)

Der er nogle ting vi skal gøre brugere opmærksomme på: 
- Pas på ledningen!
- Bevæg ikke hovedet for meget, trackingen vil drifte!
- Start langsomt, få en fornemmelse for hvordan robotten reagerer
- Der er ikke noget der stopper armene fra at kollidere med hinanden, så vær forsigtig 
- Sig til når man er done, programmet skal stoppes før headset kommer af

Steps: 
- Sæt streaming op på computer, så andre kan se med 
- Start med at tage headset på, og sikre at det sidder godt 
- Kalibrér med højre controller 
- Tjek at de kan se igennem 
- Tjek tracking i RViz 
- Hænderne på bordet (eller bare stille foran én) 
- Start programmet! 
- Vær klar til at stoppe, hvis der sker noget, eller de tager headset af for hurtigt


## Teleoperation Unity
> **Deprecated**

### Setup VR Headset 

Either you want to test an app directly from Unity, or you would directly run the app already built on the headset.

#### Connected to computer with Unity

1. Turn on headset
2. Create a boundary
3. Plug USB cable into headset and computer
> If the "`USB-C Port Disabled, water and debris`" message pops up, restart the headset and try again. If that does not work, use the other end of the cable and try again.
1. Enable Link
> Do NOT click on the `USB Detected` pop-up, only enable the link. 
1. In Unity, start the VR Interface app

#### Only VR headset

1. Turn on headset
2. Create a boundary
3. Start the VR Interface app


### Setup Robot
Have a screen connected to the Jetson of the robot, to verify tracking data sent from Unity with RViz. 

1. Turn on the tobot
2. In a teminal on the Jetson, open RViz2 with config file:
   ```
   rviz2 -d src/energirobotter-ros-workspace/energirobotter_bringup/config/rviz/teleoperation.rviz
   ```
3. In another terminal, run teleoperation with:
   ```
   ros2 launch energirobotter_bringup teleoperation_zeromq.launch.py ik_enabled:=true camera_enabled:=true ip_target:="192.168.1.102"
   ```
4. On your phone, connect to the `ESP32_DEV` Wi-Fi network
5. In browser, go to `192.168.4.1`
6. Click button `Start Serial Forwarding` - OBS! Make sure nothing is sending commands to the robot yet
7. Click botton `Stop Serial Forwarding` but don't click `OK`, it's now ready as a stop button if needed

### Calibrate and Launch

1. Calibrate the view (hold down Meta button on right controller)
2. Verify that tracking is working in RViz
3. Start `arm.launch.py` on the Jetson:
   >Make sure the VR is tracking properly! As soon as the `arm.launch.py` is running, it will start moving!
   ```
   cd energinet/
   shumble
   sw
   ros2 launch energirobotter_bringup arm.launch.py
   ```
4. When done with the teleoperation, stop the `arm.launch.py` terminal


## Face Following

> **Deprecated**

Use the `energirobotter_bringup` package's `vision.launch.py` to start the camera and face detection:

```
source install/setup.bash
ros2 launch energirobotter_bringup vision.launch.py use_compressed:=true
```

If no camera is available, run example with:
```
ros2 launch energirobotter_bringup vision.launch.py use_mock_camera:=true
```

> Note: The first time `face_detection` is run `ultralytics` will install som extra packages, and require a restart of the node.