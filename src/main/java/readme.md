Before beginning, I strongly recommend you read the research-papers about pure pursuit under #coding in the discord chat. Many of the algorithms we will use
can be found under "be0e06de00e07db66f97686505c3f4dde2e332dc_1.pdf". If you want further clarification on pure pursuit, another supplementary research paper
is "https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf";


Let's go through some basics. I have a robot and I need it to move from point A to point B. How do I achieve that?
There are two parts to this:
    1) Path generation
    2) Path following
    
These two steps go hand in hand. The path generation should build a trajectory consisting of velocities, and points to go through (we will 
call these waypoints). The path follower (pure pursuit), on the other hand, should be look at the trajectory and traverse from the start to end.

----Path generation

Our goal is to create a path from start to stop (and all the waypoints in between) with desired velocities at each point in the path.
We achieve these in these following steps:
    1) user adds desired waypoints.
    2) we inject waypoints in between at set intervals.
    3) a smoothing algorithm is applied to round off the sharp corners.
    4) we traverse the path and determine curvature at a given area. 
    5) we plan our velocity at each point based off curvature
    6) we smooth out velocity to be within velocity and acceleration constraints.  Acceleration and Deceleration are never instantaneous. 
    
   
Below I have pasted my rationale for using curvature to determine velocity.
A few nuances must be taken into account...to list a few: previous velocity, max acceleration, max velocity, the path ahead. We will go into more
detail about the path ahead. Just like it's imperative that you slow down before rounding a curve, the robot must slow down before making a turn.
However, note that you can take wider turns and a faster speed, so how can this be achieved? Currently, I propose a solution: 1) Curvature.
With curvature, you essentially draw a circle whose perimeter encompasses 3 consecutive points.
Curvature is defined as 1/r...as radius decreases, curvature approaches inf (but never inf, unless the 3 points are on top of each other) and we
slow down (and vice versa). 


