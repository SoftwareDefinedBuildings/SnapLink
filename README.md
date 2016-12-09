CellMate : Visual Appliance Identification and Control in Smart Buildings
=========================================================================


What is CellMate?
-----------------
CellMate is a system that allows you to interact with smart building appliances by taking pictures of them.


How to run CellMate server?
---------------------------
The easiest way to run CellMate server is using the prebuilt [docker image](https://hub.docker.com/r/kaifeichen/cellmate/). To get the docker image, run
```
docker pull kaifeichen/cellmate
```

If you want to build your own environment, follow these [instructions](https://gist.github.com/kaifeichen/5839d0cbf357ede24662dc7c6e1f401f).

Do you have CellMate client?
----------------------------
Yes! There is an Android client https://github.com/kaifeichen/CellMate_Android
There are also test scripts in the [test/ folder](test).

Citation
--------------

You must cite the following paper if you use this code in any ways:
```
@inproceedings{chen2015intuitive,
  title={Intuitive Appliance Identification using Image Matching in Smart Buildings},
  author={Chen, Kaifei and Kolb, John and F{\"u}rst, Jonathan and Hong, Dezhi and Katz, Randy H},
  booktitle={Proceedings of the 2nd ACM International Conference on Embedded Systems for Energy-Efficient Built Environments},
  pages={103--104},
  year={2015},
  organization={ACM}
}
```

License
-------
```
Copyright (c) 2016, Regents of the University of California
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions 
are met:

 - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the
   distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED 
OF THE POSSIBILITY OF SUCH DAMAGE.
```

Questions?
--------------

Pleaes Email Kaifei Chen <kaifei@berkeley.edu>
