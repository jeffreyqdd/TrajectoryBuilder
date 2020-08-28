package trajectory;

public class TrajectoryConstraints
{
	/**
	 * number of points within a inch
	 */
	public final double ticksPerInch = 10;

	/**
	 * Max acceleration of the robot in inches/sec^2
	 */
	public final double maxAcceleration = 10;
	
	/**
	 * Max velocity of the robot in inches/sec;
	 */
	public final double maxVelocity = 30;
	
	/**
	 * A constant ratio used to determine the speed at the given curvature
	 */
	public final double turnConstant = 4;

	/**
	 * A constant number in rad/s to determine the angular velocity to rotate at.
	 */

	public final double veerConstant = 0.3;

}
