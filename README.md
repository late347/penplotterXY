# XYPlotter

XY Plotter Project



LASER WARNINING!!!
Use any code at your own responsibility. It would be suggested that you disconnect the power cord from the laser module before running any code. If you intend to use laser module, it is suggested that you first put the correct and proper safety glasses on before running any code, and even before connecting the power cord to the laser module. Use any code at your own risk.


C++ FreeRTOS firmware for a makeblock XY plotter device with lasermodule. To be utilized with mDraw desktop GUI.
Code was tested with an emulator board and seemed to work in that emulation environment (time criticality was not enforced in that emulation). Also code worked quite well on the real plotter device.

Bresenham algorithm part of the code was inspired by the original Bresenham's IBM journal article about that algorithm (should be available online for free)

In the end, there was at least one bug found

1.) the laser module SCtimer had a bug in it which was later verified with oscilloscope. So the bug was that the SCtimer didn't operate at proper duty cycle when supposedly 100%, I think the bug was in setLaserValue function. The bug could have been corrected by having smaller  value in that specific register. E.g. something like this LPC_SCT0->MATCHREL[1].H=997. For more detailed examples about duty cycle and SCtimer programming refer to the SCtimer cookbook made by NXP.

2.) more of an undocumented feature type of bug was that the mDraw image had to be mirrored  (inside mDraw GUI) for the plot to be accurate on the paper. This evidently had crept into the design along development. There was some unclear documentation about that with the actual plotter but I wasn't able to resolve the issue in the code in due time.

Otherwise the plotter still worked in penMode and laserMode, it was slightly slow, but also accurate as verified by the laser plots on paper.
