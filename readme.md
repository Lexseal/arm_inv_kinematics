This is simple script to convert cartesian queries to angles that motors with encoders can follow.

Right now, it spits out the desired angles given a set of coordinates. However, given each query takes less than a millisecond, numerical derivative can be calculated to give angular speed based on the end point velocity.

Further work is needed to link this to the simulation. Moreover, a model for the wrist will be added later.