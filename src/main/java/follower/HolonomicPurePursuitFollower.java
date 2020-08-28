package follower;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import trajectory.Trajectory;
import util.MathFunctions;

public class HolonomicPurePursuitFollower extends PurePursuit{


    public HolonomicPurePursuitFollower(Trajectory trajectory, double SCANNING_DISTANCE, double MAX_LOOKAHEAD_DISTANCE, double MIN_LOOKAHEAD_DISTANCE) {
        super(trajectory, SCANNING_DISTANCE, MAX_LOOKAHEAD_DISTANCE, MIN_LOOKAHEAD_DISTANCE);
    }


    @Override
    public double getRotation() {
        //transform from global reference to robot reference frame
        double error;

        //System.out.println(trajectory.heading(previousClosestIndex));


        if(Double.isNaN(trajectory.heading(previousClosestIndex))) {
            Vector2D robotFrame = previousLookaheadPoint.subtract(robotPose.getVector());
            error = Math.atan2(robotFrame.getY(), robotFrame.getX()) - robotPose.getHeading() + (isReverse ? Math.PI : 0d);
        }
        else {
            error = trajectory.heading(previousClosestIndex) - robotPose.getHeading();
        }
        return MathFunctions.angleWrap(error);
    }

    @Override
    public Vector2D getVelocity() {
        Vector2D robotFrame = previousLookaheadPoint.subtract(robotPose.getVector());


        double direction = MathFunctions.angleWrap( Math.atan2(robotFrame.getY(), robotFrame.getX())  + (isReverse ? Math.PI : 0d) );


        double velocity = trajectory.velocity(previousClosestIndex) * (isReverse ? -1d : 1d);

        return new Vector2D(Math.cos(direction) * velocity  , Math.sin(direction) * velocity  );
    }

    @Override
    public double getError() {
        return robotPose.getVector().distance(trajectory.v(trajectory.size() - 1));
    }
}
