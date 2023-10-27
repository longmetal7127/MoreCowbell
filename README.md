# Longmetal Code 2023
- Setup for 2023 year
- Starting with mecanum drive
- Minimum working setup


Made by Graham and Teddy

https://www.firstinspires.org/robotics/frc/playing-field

https://docs.limelightvision.io/en/latest/

https://docs.photonvision.org/en/latest/docs/examples/gettinginrangeofthetarget.html


Saturday Plans

Map out course Group A
Autonomous Group B

plot out 3 options
which side are we on
accounting for other teams?
servo code from last year
add graphs to smartboard
gryoscopic programming


Startup Procedure: 
## Instructions for Running the Robot
1. If you are reading this, you probably already got a computer with the code on it. If not, go get one.
2. Also get a controller (XBOX for now) and plug it into one of the USB slots in the computer.
3. Within the computer, open Visual Studio Code. You should be able to open it from the windows desktop.
4. In VS Code, there should be the words "Open a folder" in blue text in the center of the IDE. Click that. It will open up the computer's files.
5. If you aren't already in the desktop directory, click on the tab within the file explorer called "Desktop". For this robot, select the folder called "More Cowbell" and click the button called "Select Folder" or something. This will bring you back to VS Code and open up the files for the robot!
6. In the heirarchy within the "Explorer" tab on the left, you can navigate through the directories of the robot. One file in the root folder is called "build.gradle". This builds all the code for the robot so that you can run it. Right click on it and a list of options should show up. Click on "Deploy robot code" at the bottom. This will open another application called Drive Station.
7. Within Drive Station, in the bottom panel there should be an outlined green arrow. If it's not already filled in, click it to run the robot code.
8. Now, open the tab within the drive station that has a wheel on it. Within that tab there should be an enable and disable switch. When enabled, the robot can move, and vice versa for when it's disable. After making sure no one is nearby, enable the code, and you will be driving a fully functioning robot! If not, there's probably a bug somewhere or you forogt to do something.

## Debug
1. Sometimes the controller will recognize inputs but not move the robot after enabling the code. One reason for this is that the computer may be recognizing the wrong USB slot for the controller. If this is true, then navigate to the tab in the Drive Station that looks like a USB slot. Within there, make sure your controller (the one flashing green after you press a button) is in slot 0. Any other slot will not work. 
