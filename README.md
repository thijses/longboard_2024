# Thijs LongBoard (TLB) firmware
This repo contains the code (compiled using platformIO) for my custom electric longboard. This thing is a 12~60V (3~13S LiPo) BLDC speed-controller (currently FOC focussed, but sensorless (BEMF) option is in development), as well as an ESP32-S3 for control & radio. The PCB is intended to be mounted inbetween the board and the truck (where most boards have a soft-plastic spacer)

The design of the PCB (that this code is intended to run on) can be found here:  https://oshwlab.com/s.t.van.liempd/tlb  
[<img src="https://image.easyeda.com/pullimage/qjOKbqKg9DSRzFNJYLZQwF0ZJc26zn57OlJAEmKx.jpeg">](https://oshwlab.com/s.t.van.liempd/tlb)  
I've produced (using JLCPCB assembly service) the 1st revision ('R01'), and am currently working on R02. Fair warning, because of the fancyness of the PCB and components (as compared to my previous projects), the PCB production is exactly cost-optimized (at 200~300$ for 5 assembled PCBs).

To mount the PCB between the board and truck, some 3D printed pieces are required. The design for these can be found here:  
https://a360.co/4b4FQNT

[//]: # (i've commented out this embed-link {from the https://fusion.online.autodesk.com/ online portal} because github-flavored markdown doesn't do <iframe> objects.         <iframe src="https://myhub.autodesk360.com/ue2d88aa0/shares/public/SHd38bfQT1fb47330c9981fc96b1093cb8d4?mode=embed" width="1024" height="768" allowfullscreen="true" webkitallowfullscreen="true" mozallowfullscreen="true"  frameborder="0"></iframe>     )

I'm currently still testing the 1st revision PCB, progress can be found here:  
https://docs.google.com/spreadsheets/d/1WjNS1LBmWm-569doQyxE5OfJIkMKA3vcxIIeLDl5snc/edit?usp=sharing  

For the current state of the code, see the TODO list in main.cpp (which i may move to here).