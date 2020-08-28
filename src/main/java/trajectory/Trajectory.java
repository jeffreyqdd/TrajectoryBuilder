package trajectory;

import geometry.Pose3D;
import geometry.Waypoint5D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import java.util.ArrayList;
import java.util.List;

public class Trajectory
{
	private final List<Waypoint5D> marker;

	Trajectory(List<Pose3D> path, List<Double> velocity, List<Double> curvature){
		assert(path.size() == velocity.size());
		assert(velocity.size() == curvature.size());

		this.marker = new ArrayList<>();

		for(int i = 0; i < path.size(); i++) {
			marker.add(new Waypoint5D(
					path.get(i).getVector(),
					curvature.get(i),
					velocity.get(i),
					path.get(i).getHeading()
			));

		}


	}

	public int size() {return marker.size();}

	public double x(int i){
		return marker.get(i).getX();
	}
	public double y(int i){
		return marker.get(i).getY();
	}
	public Vector2D v(int i) { return marker.get(i).getPoint(); }
	public double curvature(int i){
		return marker.get(i).getCurvature();
	}
	public double velocity(int i){
		return marker.get(i).getVelocity();
	}
	public double heading(int i){
		return marker.get(i).getHeading();
	}

	@Override
	public String toString() {
		StringBuilder s = new StringBuilder();

		for(Waypoint5D waypoint5D : marker)
		{
			s.append(waypoint5D).append("\n");
		}
		return s.toString();
	}
}
