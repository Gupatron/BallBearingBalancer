# BallBearingBalancer


This was a project that I began at the end of my freshman year of university. I saw a video by Aaed Musa, an engineering YouTube creator who made a video about a platform that can balance a steel ball bearing and decided that
I wanted to replicate it. At the time I was taking linear algebra which ended up being really useful in understanding the Inverse Kinematics of the platform. 





**How it Works** 

The platform operates using three Nema 17 Stepper motors that control heights of the three legs of the platform. Based on the height of the legs you can manipulate the pitch and roll of the platform. Inorder to determine how much the 
platform pitched and rolled I used a PID control loop to keep the ball balanced on the platform. The position of the ball was tracked using a resistive touch pad. These work by changing resistance depending on where pressure is being 
applied to the pad. You can then use a microcontroller to measure the voltage drop at the point where pressure is being applied and interpolate to find position. Once the position of the ball is known we define an error term in our control
loop. The error is defined as the distance from the center of the platform. The goal of the PID Controller is to minimize the error or make it zero. When the error is zero, the ball is at the center of the platform. 

Since I was using stepper motors, I used two leg links to convert rotational translation into linear translation. This added an additional calculation where the angle of the stepper motor was determined using the law of cosines.
The PID loop would give the desired distance to the ground, since this length, and the two link lengths were known, the motor angle was computed using the Law of Cosines. 
