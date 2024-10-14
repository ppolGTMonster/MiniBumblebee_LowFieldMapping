We managed our environment with Anaconda.

The most important libraries are:
* openCV in version 4.5.5 (!)
After this version, something was changed in the position estimation of the Aruco markers and this code will not longer work

* Python 3.9.18
Works perfectly with the used openCV version. 
Other versions caused difficulties (the combination Py/Numpy/openCV seems to be critical)

* Numpy 1.26.3
Apparently, the openCV version used seems to be critical to the numpy version installed. 
We had big problems to get this to work reliably. 


However it worked fine with this Py/Numpy/openCV combination

* Scipy 1.12.0
Other versions should not be a problem. At least in my code, where i used it. 
I don't know if scipy is used internally in openCV somewhere 