# The DSKY-matic Project
A Functioning DSKY Replica - October 2020

### Why is it called "DSKY-matic"?

Because *"Riley Rainey's Apollo Guidance Computer DSKY Replica Project"* doesn't really roll off the tongue.

![The first DSKY-matic prototype](images/front-early-sm.jpg)

Naming a project is often difficult. In this case it's especially hard, given that there are already a number of replicas of the original Apollo Guidance Computer hardware described on-line. I created this project almost entirely for my own gratification. I wanted to create a replica of the original DSKY that was faithful to the original look and feel. And I wanted to be able to interact with it.  I named it **DSKY-matic** to give everyone else an easy name to use to distinguish this from other efforts. 

DSKY-matic is an open project so that others can reproduce the work or build upon my results.

### Components and cloning this project

The Electroluminescent Display, Alarm Panel, and Keyboard are each modular components. They work together through the
  Raspberry Pi 4 to simulate the AGC, but each could be swapped out with similar redesigns.

![The first DSKY-matic prototype](images/DSKY-matic-blocks.png)

The hardware and firmware for each module resides in three separate git submodules. You will need to follow some specific steps to clone the entire tree on your local machine. First, clone this repository:

        $ git clone https://github.com/rrainey/DSKY-matic.git

Go into the project folder:

        $ cd DSKY-matic

Now clone all submodules:

        $ git submodule update --init --recursive --remote

### Directory structure

* **hardware** - 3D-printable frame components designed using Fusion 360. STEP and the original Fusion source files are provided. Hardware models used in thie replica were derived from models found in the [AGC Mechanical CAD project](https://github.com/rrainey/agc-mechanical-cad).

* **software** - software components designed to run on the projects embedded RaspBerry Pi 3/4.  This siftware is a Apollo Guidance Computer virtual machine interfacing to the deisplays and keyboard via I2C and serial communications drivers.

### Credits

This work would not have been possible without the generous and open work of these projects:

* [The Virtual AGC Project](https://www.ibiblio.org/apollo/) -- Ron Burkey and others have worked for years to build software emulator of the AGC hardware, supporting developments tools, a collection of original NASA and MIT documents, as well as an archive of transcribed original CM and LEM programs.
* [Adafruit Industries](https://www.adafruit.com/) -- Lady Yada and others
* [The Raspberry Pi Foundation](https://www.raspberrypi.org/about/) -- The Raspberry Pi Foundation is a UK-based charity that is the source fabulous Linux-compatible low-cost computers.

## DSKY-matic License

Creative Commons Attribution/Share-Alike, all text above must be included in any redistribution. See license.txt for additional details.

## My Background

My name is Riley Rainey. I'm a software developer by profession. I spent a number of years building aerospace simulations as my day job.

## Getting Support

There's no official support available, but you can [leave comments or create an issue](https://github.com/rrainey/DSKY-alarm-panel-replica/issues) in this GitHub project.


[![Creative Commons License](https://i.creativecommons.org/l/by-sa/4.0/88x31.png)](http://creativecommons.org/licenses/by-sa/4.0/)  
This work is licensed under a [Creative Commons Attribution-ShareAlike 4.0 International License](http://creativecommons.org/licenses/by-sa/4.0/).