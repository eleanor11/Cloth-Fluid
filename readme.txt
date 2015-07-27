============================================================================
NVIDIA Flex - 0.8 
============================================================================

Flex is a particle-based simulation library designed for real-time applications.
Please see the manual included in this release package for more information on
the solver API and usage.

Supported Platforms
-------------------

Windows 32/64 bit
Linux 64 bit (tested with Ubuntu 12.04 LTS)

Requirements
------------

* A CUDA compute capability 3.0 or greater graphics card
* NVIDIA driver of at least version 347.25
* CUDA 7.0 toolkit (required for building the demo)

============================================================================
Demo 
============================================================================

bin\platform\flexDemoRelease(.exe)

Notes 
-----

* Some scenes also have fluid emitters that can be started using 'space'.

Command Line Options
--------------------

-fullscreen=wxh		Start fullscreen e.g.: -fullscreen=1280x720
-msaa=0				Disable multisampling (default is on)
-device=n			Choose CUDA device to run on

Controls
--------

w,s,a,d - Fly Camera
right mouse - Mouse look
shift + left mouse - Particle select and drag

p - Pause/Unpause
o - Step
h - Hide/Show onscreen help

left/right arrow keys - Move to prev/next scene
up/down arrow keys - Select next scene
enter - Launch selected scene
r - Reset current scene

e - Draw fluid surface
v - Draw points
f - Draw springs
i - Draw diffuse
m - Draw meshes

space - Toggle fluid emitter
y - Toggle wave pool
c - Toggle video capture
u - Toggle fullscreen
j - Wind gust
b - Remove a convex
- - Remove a plane
Esc - Quit

Known Issues
------------

* Some performance problems when fluid covers a large area of the screen

Contact
------------------
mmacklin@nvida.com
