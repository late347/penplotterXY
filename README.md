# XYPlotter

XY Plotter Project



LASER WARNING!!!
Use any code at your own responsibility. It would be suggested that you disconnect the power cord from the laser module before running any code. If you intend to use laser module, it is suggested that you first put the correct and proper safety glasses on before running any code, and even before connecting the power cord to the laser module. Use any code at your own risk.


C/C++ FreeRTOS firmware for a makeblock XY plotter device with lasermodule. To be utilized with mDraw desktop GUI program.
Code was tested with an emulator board and seemed to work in that emulation environment (time criticality was not enforced in that emulation). Also code worked quite well on the real plotter device.

Bresenham algorithm part of the code was inspired by the original Bresenham's IBM journal article about that algorithm (should be available online for free)

In the end, there were a couple issues, but otherwise it was quite rewarding and practically useful (working) project.
Fixing the issues isn't really feasible since the hardware is no longer available anyway to test any fixes.

1.) This was a bad issue in hindsight. But it was discovered much after the project was done. It affected e.g. the G-code parser. The entire usercode (not freertos itself and not the lpc libraries) should never have used C++ style strings and other heap allocated objects in C++ style inside the project. Only the basic C-style strings should have been used, or alternatively use the FreeRTOS API to create heap allocated objects in accordance to the user chosen FreeRTOS heap scheme, and use the same API to deallocate, I think. C++ strings etc. were not safe to allocate in regular C++ style within the FreeRTOS.

2.) the laser code with SCtimer had a bug in it which was later verified with oscilloscope. So the bug was that the SCtimer didn't operate at proper duty cycle when supposedly 100%. It was a "off by one" issue with the sctimer counter. It should have been written to just use 100% duty cycle internally as something lie 99.7% duty cycle. I think the bug was in setLaserValue function. The bug could have been corrected by having smaller  value in that specific register. E.g. something like this LPC_SCT0->MATCHREL[1].H=997. For more detailed examples about duty cycle and SCtimer programming refer to the SCtimer cookbook made by NXP.

3.) more of an undocumented feature type of bug was that the mDraw image had to be mirrored  (inside mDraw GUI) for the plot to be accurate on the paper. This evidently had crept into the design along development. There was some unclear documentation about that with the actual plotter but I wasn't able to resolve the issue in the code in due time. 

With hindsight I think this bug could have been  fixed but it would require switching the direction of the initial state of that one axis inside calibration task, and also invert that one axis direction inside the refactored_setupBresenhamDirPins() function. 


4.) also there was an unknown issue with one of the three plotters. For some reason the same exact code ran worse on one of the three plotters. It may have been a hardware issue (when I asked about it from teacher), or a code issue. For the two out of three plotters the same code ran accurately in pencilmode and lasermode in regular fashion...
 


Otherwise the plotter still worked in penMode and laserMode, it was slightly slow, but also accurate as verified by the accurate path of the laser plots on paper.
