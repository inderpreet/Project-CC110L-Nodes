# Project CC110L Nodes

This is a simple prject where I use MSP430G2553 Microcontrollers with the Anaren CC110L Booster Packs to make a simple wireless node system. Nothing fancy and a barebones started project is provided for getting off the ground fast.

blog post: [https://bodgewires.github.io/cc110l-demo-nodes/](https://bodgewires.github.io/cc110l-demo-nodes/)

## How to Import

Only the essential files are checked into the Git repository to reduce the size and hassle of version upgrades etc. As per wiki...

The following project files should be checked into source control:

.ccsproject
.cproject
.project
.settings folder
makefile.defs (if using SYS/BIOS)
.ccsproject has project information specific to CCS.

.cproject and .project are Eclipse CDT project files.

.settings folder is a standard Eclipse folder that has settings that apply for the project.

makefiles.defs has additional make rules included by the generated project makefile for SYS/BIOS projects.


The following files and folders should not be checked into source control:

\Debug or \Release \<configuration name> folder
.xdchelp
\.config folder
\.launches folder
Configuration folders like \Debug or \Release are generated folders that contain build artifacts like object files and do not need to be checked into source control.

.xdchelp is a generated file that is used to help display error messages in the problems view and does not need to be checked into source control.

.config and .launches are generated folders that do not need to be checked into source control.

So here is what you do...
### Steps
* clone of download the code or the repository the way you like it. I leave that upto you and we only need the CCS folder in the repo for the next steps.
* Create a new project in CCS like the way you would normally do. Setup and finish the wizard if thats your thing.
* Right click the project and click Import > General > File System. 
* Navigate to the CCS folder from the repository. Select the folder and Import. Overwrite Everything!
* Build and debug.
* Donate to my PayPal(optional)


## How to Use

There are three files in the root directory of this project. Import the whole project into Code composer Studio and then right click on either the rx or tx file. 
Exclude it from build and then rebuild and program the output. Repeat for the other file. Now you should have two controllers programmed with TX and RX firmware.
DONE!

## License

This project is licensed under the GPLv2 license - see the [LICENSE.md](LICENSE.md) for details

http://inderpreet.github.io


Designed by Inderpreet Singh(inderpreet.github.io)

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file LICENSE.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").
