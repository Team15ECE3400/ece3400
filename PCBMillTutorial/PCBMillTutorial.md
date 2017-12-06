PCB Mill Tutorial (In progress..)
by Daniel Edens, Oct. 27th, 2017

Introduction
The QCJ5 Quick Circuit Mill delivers the precision, speed, and automation required to take your prototype to the next level. Making a board on the mill enhances your PCB designing and machine skills. It’s also exciting to watch your board created live. The main difference between a milled board and a PCB is that a PCB is a sheet of dielectric material with copper placed as traces and pads, and a milled board has copper on the entire plane of two faces with a dielectric in between and traces and pads are defined by cuts. On the milled board, cuts are made through the copper and into the dielectric to completely isolate any sections of copper.

Part of the Maker Space?
If you are already part of the Maker Space, trained on the PCB mill, and want to mill out your own PCB, we can provide you with 1- or 2-layer boards. The cost of these will count against your final budget. Please send Kirstin an email with the approximate size of the board you want, and how many layers of copper you need (1 or 2).
Please designate one person from your team to learn how to use and work on the PCB mill. This will allow more teams to have at least one trained member. You can contact <did Daniel Edens graduate?> to request training. Include your team number in the email.

Designing a Layout
Be sure to check out Leah’s lecture slides on layout design and (Leah’s tutorial). Your layout design for the mill will be slightly different than an optimal PCB. For instance, a via on a milled board is created by drilling a hole and inserting and squishing a pin into the hole, so a header or through hole component cannot have traces on the same layer as itself since the component pins and the pin cannot exist in the same hole. This means that the trace must connect to the component pins on the opposite layer of the component as shown in figure 2. Please keep this in mind in your design. You may have to add vias to your board to make this possible. Figure 1 and 2 show this setup.

![](./Images/UnoMilledPCBTop.JPG)
Figure 1: Arduino Uno Milled Shield Top

![](./Images/UnoMilledPCBBottom.JPG)
Figure 2: Arduino Uno Milled Shield Bottom

PCB Mill Tutorial
This tutorial is based on the ![https://ece.uncc.edu/sites/ece.uncc.edu/files/media/isoproj5.pdf](QCJ5 manual) and goes more into detail on all the steps with pictures and extra steps not covered in the manual. Please read and understand this tutorial before training to maximize your learning experience. Also skim the online manual from page 8 to 32.

1) Importing Gerber Files
The only gerber files you need are the top layer .GTL, bottom layer .GBL, and drill which may be a .DRL or .TXT file. You will have to try .DRL and .TXT to see which works.
Place your gerber files in “Desktop/Mill Project Files/<your net id folder>/<project name>” shown in Figure 3. Open the ISOPro Software. If the software doesn’t load, restart the computer and try again. In the software, click “File > Import > Auto-detect File(s)”. Navigate to your gerber files and select the .GTL, .GBL, and .TXT or .DRL files.

![](./Images/1.JPG)
Figure 3

Import your gerber files using Auto Import.
![](./Images/2.JPG)
Figure 4

Select the .GBL, .GTL, and .txt Drill file (if this drill file doesn't work, try a different drill file in your gerber output)
![](./Images/3.JPG)
Figure 5

Select the size fo your PCB. The software believes the PCB could be one of these sizes.
![](./Images/4.JPG)
Figure 6

Click view->Layer Table
![](./Images/6.JPG)
Figure 8

Click view->Tool Table, and change the size of drill bit.
![](./Images/7.JPG)
Figure 9

![](./Images/8.JPG)
Figure 10

![](./Images/9.JPG)
Figure 11

![](./Images/10.JPG)
Figure 12

![](./Images/11.JPG)
Figure 13

![](./Images/12.JPG)
Figure 14

![](./Images/13.JPG)
Figure 15

![](./Images/14.JPG)
Figure 16

![](./Images/15.JPG)
Figure 17

![](./Images/16.JPG)
Figure 18

![](./Images/17.JPG)
Figure 19

![](./Images/18.JPG)
Figure 20

![](./Images/19.JPG)
Figure 21

![](./Images/20.JPG)
Figure 22

![](./Images/21.JPG)
Figure 23

![](./Images/22.JPG)
Figure 24

![](./Images/23.JPG)
Figure 25

![](./Images/24.JPG)
Figure 26

![](./Images/25.JPG)
Figure 27

![](./Images/26.JPG)
Figure 28

![](./Images/27.JPG)
Figure 29

![](./Images/28.JPG)
Figure 30

![](./Images/29.JPG)
Figure 31

![](./Images/30.JPG)
Figure 32

![](./Images/31.JPG)
Figure 33

![](./Images/32.JPG)
Figure 34

![](./Images/33.JPG)
Figure 35

![](./Images/34.JPG)
Figure 36

![](./Images/35.JPG)
Figure 37

![](./Images/36.JPG)
Figure 38

![](./Images/37.JPG)
Figure 39

![](./Images/38.JPG)
Figure 40
