package trajectory;

import geometry.Pose3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import util.MathFunctions;
import util.spline.Spline2D;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;


public class TrajectoryBuilder
{
	/**
	 * contains all the constraints the TrajectoryBuilder needs to follow
	 */
	private final TrajectoryConstraints constraints;

	/**
	 * contains the (x,y) coordinates of waypoints
	 * ith index is the coordinate of the ith point
	 */
	private List<Pose3D> poses;
	private List<Integer> poseIndex;

	/**
	 * constructor
	 * use of arraylists slightly enhances performance.
	 */
	public TrajectoryBuilder() {
		this.constraints = new TrajectoryConstraints();
		this.poses = new ArrayList<>();
		this.poseIndex = new ArrayList<>();


	}

	/**
	 * Adds a waypoint to the path
	 * @param point the waypoint that needs to be added
	 * @return the the added waypoint
	 */
	public void addPose(Pose3D point) {
		poses.add(point);
	}

	/**
	 * Clears trajectory
	 */
	public void clear() {
		poses.clear();
	}

	/**
	 * builds the trajectory to be used by the follower;
	 * @return the annotated trajectory
	 * @throws IllegalStateException if trajectory is not complete
	 */

	public Trajectory build() throws IllegalStateException{
		if(poses.size() < 2) throw new IllegalStateException("trajectory size " + poses.size() + "is smaller than expected");

		List<Pose3D> path = createPoints(poses, poseIndex, constraints.ticksPerInch);
		List<Double> pathCurvature = determinePathCurvature(path);
		List<Double> velocityMarkers = determinePathVelocity(path, pathCurvature, constraints.maxVelocity, constraints.maxAcceleration, constraints.turnConstant);

		path = createHeading(path, poses, poseIndex, velocityMarkers, constraints.veerConstant);

		return new Trajectory(path, velocityMarkers, pathCurvature);
	}

	/**
	 * creates a trajectory with waypoints
	 * @param poses array of poses
	 * @param ticksPerInch number of points within a inch
	 * @return high definition path
	 */
	private List<Pose3D> createPoints(List<Pose3D> poses, List<Integer> poseIndex, double ticksPerInch) {
		List<Vector2D> points = poses.stream().map(Pose3D::getVector).collect(Collectors.toList());

		double distance = 0;

		for(int i = 0; i < points.size() - 1; i++)
			distance += points.get(i).distance(points.get(i + 1));

		double increment = 1d / (distance * ticksPerInch);

		Spline2D spline2D = new Spline2D(points);
		spline2D.calcSpline();

		List<Pose3D> path = new ArrayList<>();

		for(double d = 0; d < 1d; d += increment) {
			path.add(new Pose3D(spline2D.getPoint(d), Double.NaN));
		}


		int bestIndex = 0;
		for(Pose3D point : poses){
			double bestDistance = 1000000000d;
			for(int i = bestIndex; i < path.size(); i++) {
				distance = path.get(i).getVector().distance(point.getVector());
				if(distance < bestDistance)
				{
					bestIndex = i;
					bestDistance = distance;
				}
			}
			poseIndex.add(bestIndex);
		}

		return path;

	}

	/**
	 * Determines path curvature
	 * @param path trajectory waypoints
	 * @return array of curvature values where the ith index is the curvature of the ith point.
	 */
	private List<Double> determinePathCurvature(List<Pose3D> path)
	{
		//create curvature list
		List<Double> curvatureList = new ArrayList<>();


		//runs check to see if path length is 2. If 2, then answer is [max, max]
		if(path.size() == 2)
		{
			for(int i = 0; i < 2; i++) curvatureList.add(0d);
			return curvatureList;
		}

		for(int i = 0; i < path.size() - 2; i++) {
			Vector2D a = path.get(i).getVector(), b = path.get(i + 1).getVector(), c = path.get(i + 2).getVector();

			double triangleArea = (b.getX()-a.getX())*(c.getY()-a.getY())
					- (b.getY()-a.getY())*(c.getX()-a.getX());

			double curvature = Math.abs(4 * triangleArea /
					(a.distance(b) * b.distance(c) * c.distance(a)));

			curvatureList.add(curvature);

		}

		//make sure path.length() == curvatureList.length()
		//a simple fix would be to duplicate the last item in the list twice.
		double last = curvatureList.get(curvatureList.size() - 1);
		for(int i = 0; i < 2; i++)
			curvatureList.add(last);

		return curvatureList;
	}

	/**
	 * Determines path velocity
	 * @param path via points
	 * @param pathCurvature curvature values
	 * @param maxVelocity maximum robot velocity
	 * @param maxAcceleration maximum robot acceleration
	 * @param turnConstant ratio for cornering speeds.
	 * @return array of velocities where the ith index is the instantaneous velocity at the ith point
	 */
	private List<Double> determinePathVelocity(List<Pose3D> path, List<Double> pathCurvature, double maxVelocity, double maxAcceleration, double turnConstant)
	{
		//create velocity list
		List<Double> velocity = new ArrayList<>();

		for(int i = 0; i < pathCurvature.size(); i++) {
			velocity.add(
					Math.min(maxVelocity, turnConstant / pathCurvature.get(i))
			);
		}

		//set velocity of last point to zero
		velocity.set(velocity.size() - 1, 0.0);
		velocity.set(0,
				2d
		);

		for(int i = velocity.size() - 2; i >= 0; i--) {
			double distance = path.get(i + 1).getVector().distance(path.get(i).getVector());
			double newVelocity = Math.min(velocity.get(i),
					Math.sqrt( Math.pow(velocity.get(i + 1),2) + 2 * maxAcceleration * distance));
			velocity.set(i, newVelocity);
		}


		for(int i = 1; i < velocity.size(); i++) {
			double distance = path.get(i - 1).getVector().distance(path.get(i).getVector());
			double newVelocity = Math.min(velocity.get(i),
					Math.sqrt( Math.pow(velocity.get(i - 1),2) + 2 * maxAcceleration * distance));
			velocity.set(i, newVelocity);
		}

		return velocity;
	}

	private List<Pose3D> createHeading(List<Pose3D> path, List<Pose3D> poses, List<Integer> poseIndex, List<Double> velocityMarkers, double veerConstant) {
		List<Double> timeEstimate = new ArrayList<>();
		List<Double> rawHeading = new ArrayList<>();

		for(int i = 0, j = 0; i < path.size(); i++) {
			Vector2D arrow;
			if(i == 0) {
				timeEstimate.add(0d);
				arrow = path.get(1).getVector().subtract(path.get(0).getVector());
			}
			else
			{
				double vel = velocityMarkers.get(i - i); //inches per second
				double distance = path.get(i).getVector().distance(path.get(i - 1).getVector()); //inches
				double time = distance / vel; //seconds;

				timeEstimate.add(timeEstimate.get(i-1) + time);
				arrow = path.get(i).getVector().subtract(path.get(i - 1).getVector());
			}

			if(i > poseIndex.get(j)) {
				j += 1;
			}

			if(Double.isNaN(poses.get(j).getHeading()))
			{
				rawHeading.add(MathFunctions.angleWrap(Math.atan2(arrow.getY(), arrow.getX())));
			}
			else
			{
				rawHeading.add(poses.get(j).getHeading());
			}
		}

//		System.out.println(timeEstimate);
		List<Pose3D> enhancedPath = new ArrayList<>();
		//forward iteration
		for(int i = 0; i < path.size(); i++) {
			if(i == 0) {
				enhancedPath.add(new Pose3D(path.get(i).getVector(), rawHeading.get(i)));
			}
			else {
				double headingBefore = enhancedPath.get(i - 1).getHeading();
				double headingNow =  rawHeading.get(i);

				double deltaTime = timeEstimate.get(i) - timeEstimate.get(i-1);
				double deltaHeading = headingNow - headingBefore;
				double adjustedDeltaHeading = Math.min(deltaHeading * deltaTime, veerConstant);

//				System.out.println("time: " + timeEstimate.get(i) + "\nheading before: " + headingBefore + "\nheading now: " + headingNow  + '\n' +
//						"correction: " + adjustedDeltaHeading +
//
//						"\nnew heading: " + (headingBefore+adjustedDeltaHeading) + "\n");


				enhancedPath.add(new Pose3D(path.get(i).getVector(), headingBefore + adjustedDeltaHeading));
			}
		}

		return enhancedPath;
	}
}
