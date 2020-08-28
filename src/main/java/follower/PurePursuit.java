package follower;

import geometry.Pose3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import trajectory.Trajectory;

public abstract class PurePursuit {
    protected final Trajectory trajectory;
    private final double SCANNING_DISTANCE;
    private final double MAX_LOOKAHEAD_DISTANCE;
    private final double MIN_LOOKAHEAD_DISTANCE;

    protected int previousClosestIndex;
    protected int previousLookaheadIndex;

    protected Vector2D previousClosestPoint;
    protected Vector2D previousLookaheadPoint;

    protected boolean isReverse;
    protected Pose3D robotPose;



    public PurePursuit(Trajectory trajectory, double SCANNING_DISTANCE, double MAX_LOOKAHEAD_DISTANCE, double MIN_LOOKAHEAD_DISTANCE){
        super();
        this.trajectory = trajectory;
        this.SCANNING_DISTANCE = SCANNING_DISTANCE;
        this.MAX_LOOKAHEAD_DISTANCE = MAX_LOOKAHEAD_DISTANCE;
        this.MIN_LOOKAHEAD_DISTANCE = MIN_LOOKAHEAD_DISTANCE;
        this.isReverse = false;

        this.previousClosestIndex = 0;
        this.previousLookaheadIndex = 0;
        this.previousClosestPoint = trajectory.v(previousClosestIndex);
        this.previousLookaheadPoint = trajectory.v(previousLookaheadIndex);
        this.robotPose = new Pose3D(0,0,0);

    }

    public void markForward() {
        isReverse = false;
    }

    public void markReverse() {
        isReverse = true;
    }

    public void updateRobotPose(Pose3D robotPose) {
        this.robotPose = robotPose;
        updateClosestPoint();
        updateLookaheadPoint();
        //System.out.println(previousLookaheadIndex);
    }

    private void updateClosestPoint() {
        double bestDist = 1000;
        int bestIndex = previousClosestIndex;

        //System.out.println(previousLookaheadIndex);

        for (int i = previousClosestIndex; i < trajectory.size() && i < previousLookaheadIndex; i++) {
            double distance = trajectory.v(i).distance(robotPose.getVector());
            if (distance < bestDist) {
                bestDist = distance;
                bestIndex = i;
            }
        }

        previousClosestIndex = bestIndex;
        previousClosestPoint = trajectory.v(previousClosestIndex);




    }
    private void updateLookaheadPoint() {
        double lookaheadDistance = getLookaheadDistance();
        for (int i = previousClosestIndex + 1; i < trajectory.size() - 1; i++) {
            Vector2D E = trajectory.v(i);
            Vector2D L = trajectory.v(i + 1);
            Vector2D d = L.subtract(E);
            Vector2D f = E.subtract(robotPose.getVector());

            double a = d.dotProduct(d);
            double b = 2 * f.dotProduct(d);
            double c = f.dotProduct(f) - lookaheadDistance * lookaheadDistance;

            double distance = trajectory.v(i).distance(robotPose.getVector());
            double discriminant = b * b - 4 * a * c;

            if(distance > lookaheadDistance) break;

            if (discriminant >= 0) {
                //do stuff
                discriminant = java.lang.Math.sqrt(discriminant);

                double t1 = (-b - discriminant) / (2 * a);
                double t2 = (-b + discriminant) / (2 * a);

                if (t1 >= 0d && t1 <= 1d) {
                    previousLookaheadIndex = i;
                    previousLookaheadPoint = E.add(d.scalarMultiply(t1));
                    return;
                } else if (t2 >= 0d && t2 <= 1d) {
                    previousLookaheadIndex = i;
                    previousLookaheadPoint = E.add(d.scalarMultiply(t2));
                    return;
                }
            }

            if(i == trajectory.size() - 2){
                previousLookaheadIndex = trajectory.size() -1;
                previousLookaheadPoint = trajectory.v(previousLookaheadIndex);
                return;
            }
        }
    }
    private double getLookaheadDistance() {

        double max_curvature = 0;
        double distance = 0;

        for(int i = previousClosestIndex; i < trajectory.size() - 1; i++) {
            distance += trajectory.v(i).distance(trajectory.v(i + 1));
            max_curvature+= trajectory.curvature(i);
            if(distance >= SCANNING_DISTANCE) {
                break;
            }

        }
        max_curvature /= distance;

        return Math.max((MAX_LOOKAHEAD_DISTANCE / Math.pow((max_curvature + 1d), 0.64)), MIN_LOOKAHEAD_DISTANCE);

    }


    public Vector2D getLookaheadPoint() {
        return previousLookaheadPoint;
    }

    public Vector2D getClosestPoint() {
        return previousClosestPoint;
    }


    public abstract double getRotation();

    public abstract Vector2D getVelocity();

    public abstract double getError();

}
