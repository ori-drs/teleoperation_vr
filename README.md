# Teleoperation system

This repository includes the teleoperation interface between Meta Quest 3 VR Headset and robot.

## Usage
- On headset: 
    - Turn on the headset: hold the power button until 'Meta Horizon OS' shows up in the headset
    - Press menu button on the right controller to show the menu bar 
    - Connect to ori-wlan (the headset should automatically connect upon starting).
    - Open the Library to the right of the menu bar -> Go to Applications -> Open `Teleoperation_v3`.
    - Input the laptop IP on the UI.
    - Hit `Start/Stop` button to start or stop/pause the data stream
- On laptop:
    - Connect to ori-wlan.
    - Build this repository: 
    ```
    catkin build teleoperation_vr
    source devel/setup.bash
    ```
    - Launch the connection:
    ```
    roslaunch teleoperation_vr connect_vr.launch
    ```
- The laptop should now receive the right controller data:
    - Check:
    ```
    rostopic echo /vr_controller
    ```
    - The rostopic is using `controller.msg` type


## Notes
- If the thumbstick signal is not sending, stop the application by hitting Menu button -> Quit. Then re-launch the app.
- If you don't need to modify the Unity application:
    - No installation is required.
    - You don't need to connect the USB C cable.


## Connect Quest to Ubuntu with cable
- This is only required if you want to develop or debug Unity app, or download media from the headset
- Install adb:
```
sudo apt install adb
```
- Connect the device to the laptop using the USB-C cable
- Confirm connection:
```
adb devices
```
- There should be a prompt in the headset asking for permission to allow connection to the computer -> Hit `Allow`