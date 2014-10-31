# About

libBipedal contains various algorithms related to bipedal walking. This library
was originally developed as part of my Bachelor's Thesis at [H2T](http://h2t.anthropomatik.kit.edu/).

# How to build

    mkdir build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    cd build
    make

# Structure

```src/``` and ```include/``` are subdived into several components:

1. ```pattern``` algorithms related to generating dynamically stable walking pattern.
	Currently the approach based on ZMP Preview control [1] is implemented.

2. ```stabilizer``` algorithms related stabilizers for bipedal walking.
	Currently the approach based on [2] is implemented. Also, a heuristic
	approach that requires the global pose of several reference frames.

# Dependencies

- [Simox](http://simox.sourceforge.net/) build with SimDynamics

- [Bullet 2](https://github.com/bulletphysics/bullet3) build with double precision

- [MMMcore](https://gitlab.com/mastermotormap/mmmcore) for trajectory playback

# References

    [1] Kajita, Shuuji, et al. "Biped walking pattern generation by using preview control of zero-moment point." Robotics and Automation, 2003. Proceedings. ICRA'03. IEEE International Conference on. Vol. 2. IEEE, 2003.
    [2] Kajita, Shuuji, et al. "Biped walking stabilization based on linear inverted pendulum tracking." Intelligent Robots and Systems (IROS), 2010 IEEE/RSJ International Conference on. IEEE, 2010.

