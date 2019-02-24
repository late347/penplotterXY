# XYPlotter

XY Plotter Project

C++ FreeRTOS firmware for a makeblock XY plotter device with lasermodule. To be utilized with mDraw desktop GUI.
Code was tested with an emulator board and seemed to work in that emulation environment (time criticality was not enforced in that emulation). Also code worked quite well on the real plotter device.

In the end, there was at least one bug found

1.) the laser module SCtimer had a bug in it which was later verified with oscilloscope. So the bug was that the SCtimer didn't operate at proper duty cycle when supposedly 100%, I think the bug was in setLaserValue function. The bug could have been corrected by having smaller  value in that specific register. E.g. something like this LPC_SCT0->MATCHREL[1].H=997. For more detailed examples about duty cycle and SCtimer programming refer to the SCtimer cookbook made by NXP.

2.) otherwise the plotter still worked in penMode and laserMode, it was slightly slow, but also accurate as verified by the laser plots on paper.
