# About

libBipedal contains various algorithms related to bipedal walking. This library
was originally developed as part of my Bachelor's Thesis at [H2T](http://h2t.anthropomatik.kit.edu/).

This library is written in C++11, but still uses boost::shared_ptr to integrate better
with ```Simox```.

libBipedal is licensed under the simplified BSD 2-clause license.

# How to build

    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make

# Structure

```src/``` and ```include/``` are subdived into several components:

1. ```pattern``` algorithms related to generating dynamically stable walking pattern.
	Currently the approach based on ZMP Preview control [1] is implemented.

2. ```stabilizer``` algorithms related stabilizers for bipedal walking.
	Currently the approach based on [2] is implemented. Also, a heuristic
	approach that requires the global pose of several reference frames.

3. ```recovery``` algorithms for push recovery. The implemented approach
	is based on the Future Immediate Capture Point [3].

# Todo

* Evalutae interface: We could make Simox optional for most code.

* MMM is only needed for ```TrajectoryPlayer``` and ```TrajectoryExporter``` make it optional

* The push recovery is very basic and is mostely a hack: Do this properly.
  I think the best way would be to integrate a force-feedback to check when the recovery foot
  has actually reached the floor. This is a problem because we are actually **falling** thus
  the pre-planed foot trajectory will much likely not hit the floor when expected.

* Add in-depth documentation for the public interfaces

* ```bipedal.h``` should be split up into ```Pattern.h```, ```Stabilizer.h``` etc.
  to avoid rebuilding the whole project if a predefine was changed.

# Dependencies

- [Simox](http://simox.sourceforge.net/) build with SimDynamics

- [Bullet 2](https://github.com/bulletphysics/bullet3) build with double precision

- [MMMcore](https://gitlab.com/mastermotormap/mmmcore) for trajectory playback

# References

    [1] Kajita, Shuuji, et al. "Biped walking pattern generation by using preview control of zero-moment point." Robotics and Automation, 2003. Proceedings. ICRA'03. IEEE International Conference on. Vol. 2. IEEE, 2003.
    [2] Kajita, Shuuji, et al. "Biped walking stabilization based on linear inverted pendulum tracking." Intelligent Robots and Systems (IROS), 2010 IEEE/RSJ International Conference on. IEEE, 2010.
	[3] Koolen, Twan, et al. "Capturability-based analysis and control of legged locomotion, Part 1: Theory and application to three simple gait models." The International Journal of Robotics Research 31.9 (2012): 1094-1113.

