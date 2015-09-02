## PX4 Flight Core and PX4 Middleware ##

This repository contains the PX4 Flight Core, with the main applications located in the src/modules directory. It has been modified from the original repo to run the autopilot base on [this book](http://uavbook.byu.edu/doku.php).  It is developed to be used as an understanding and development tool for students and researchers that have taken the Flight Dynamics class offered at Brigham Young University and could be addapted for use by others familar with the textbook. 

### Simulation ### 

A full MatLab simulation of an autopilot is used in the class and can be found in the [Matlab_simulator](https://github.com/MAGICC-UAVbook/Matlab_simulator) repository. This simulation can be used for Software and Harware in the loop (SIL\HIL) using the [SIL](https://github.com/MAGICC-UAVbook/SIL) and [MATLink](https://github.com/MAGICC-UAVbook/MATLink) repositories respectively.

### Setup ### 

See the wiki page for information on building and flashing the code.  There are a couple of models that can be used for flying this autopilot.  These can be used by setting the autostart to 2198 or 2199 ($ param set SYS_AUTOSTART 2198). 2198 is ment for HIL and 2199 is for flying the [White Knight III](http://www.et.byu.edu/~gellings/white_knight_3.html).  Most of the setup can be done from the [nutshell](https://pixhawk.org/dev/wiring) (calibration is done from the commander) can alternatively be done from QGroundControl.

### Apps ###

At this point the textbook autopilot uses four apps: controller, estimator, path_follower, and path_manager.  These apps are the code from chapters 6, 8, 10, and 11 respectively.  Each app is structure such that a student rewrite the autopilot would only have to rewrite the ###_example class.  The ###_base class is a vertual class that handles that comunication and sets up a vertual function for the ###_example class to inherit and implement.

### Testing ###

This autopilot code was tested in simulation as well as in flight.  It was never working particularly well but it worked.  It was flown on a [White Knight III](http://www.et.byu.edu/~gellings/white_knight_3.html).  Here is a [video](https://www.youtube.com/watch?v=ClOzU5v_bPo).  Here is the [presentation](http://www.et.byu.edu/~gellings/white_knight_3/presentation.pdf). 


