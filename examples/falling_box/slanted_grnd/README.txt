PURPOSE: Description of slanted_grnd falling box example
FILENAME: README.txt
AUTHOR:   Roberto Shu
LAST EDIT: 
----------------------------------------------

-------- INTRODUCTION ------------
This is an extension of the falling box sample. Now instead of having a single contact boint at the bottom of the box, there are 3 contact points. They are located at the bottom of box; one at each corner and one in the middle. 

We further extend the problem by setting the ground at an angle. This will cause the box to rotate upon impact and slide down if it violates the friction cone. 

The new generalized coordiante system q,
	q = [x,y,theta]'

Phi the constraint violation is the distance between the potential contacts and the slanted ground line. 


