package follower;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import trajectory.Trajectory;
import util.MathFunctions;

public class PurePursuitFollower extends PurePursuit{

    public PurePursuitFollower(Trajectory trajectory, double SCANNING_DISTANCE, double MAX_LOOKAHEAD_DISTANCE, double MIN_LOOKAHEAD_DISTANCE) {
        super(trajectory, SCANNING_DISTANCE, MAX_LOOKAHEAD_DISTANCE, MIN_LOOKAHEAD_DISTANCE);
    }

    @Override
    public double getRotation() {
        //transform from global reference to robot reference frame
        Vector2D robotFrame = previousLookaheadPoint.subtract(robotPose.getVector());

        double error = Math.atan2(robotFrame.getY(), robotFrame.getX()) - robotPose.getHeading() + (isReverse ? Math.PI : 0d);

        return MathFunctions.angleWrap( error );
    }

    @Override
    public Vector2D getVelocity() {
        return new Vector2D(trajectory.velocity(previousClosestIndex) * (isReverse ? -1d : 1d), 0);
    }

    @Override
    public double getError() {
        return robotPose.getVector().distance(trajectory.v(trajectory.size() - 1));
    }
}
