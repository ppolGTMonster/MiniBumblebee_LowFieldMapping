# Easy Scalable, Low-Cost Open Source Magnetic Field Detection System for Evaluating Low-Field MRI Magnets using a Motion Tracked Robot

## Author:
Pavel Povolni <pavel.povolni@tuebingen.mpg.de>

## Other Contributors to this hard- and software:
Felix Glang
  *  https://github.com/fglang
  * helped with the optimizers to design the hardware (fmincon & genetic)

Praveen Valsala
  * https://github.com/praveenivp
  * helped with reconstruction of MRI Data

Florian Birk
  * https://github.com/birkfl
  * helped with the Motion Tracking & the openCV Library

Sergej Maltsev
  * https://github.com/TheWinterProgrammer
  * helped with the Hardware 

## How to use

We tried our best to make the hardware as reproducible as possible and to develop the software without expensive extra toolboxes in Matlab (data evaluation), Python (motion tracking) and C++ (embedded code).
The PCBs were designed in KiCad 7 (free).
The mechanical components were designed with Autodesk Inventor 2021, although the Step files can be opened in any other CAD tool.

To get you started with the project as easily as possible, there are 4 instruction pdf's:
* Hallsensor (description of hardware & software for the hall sensor and calibration)
* Robot (description of the hardware & software for the robot used, including control)
* Motion tracking (description of the motion tracking setup & camera, including calibration)
* Mapping (description of the measurement process)


## Citing

If you use this code, please cite the corresponding paper:
Povolni, P., Bendfeld, R., Maltsev, S. et al. Easy scalable, low-cost open-source magnetic field detection system for evaluating low-field MRI magnets using a motion-tracked robot. Magn Reson Mater Phy (2025). https://doi.org/10.1007/s10334-025-01239-1

https://link.springer.com/article/10.1007/s10334-025-01239-1
