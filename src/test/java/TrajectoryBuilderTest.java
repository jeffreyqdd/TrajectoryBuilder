import geometry.Pose3D;
import trajectory.Trajectory;
import trajectory.TrajectoryBuilder;
import util.Stopwatch;


public class TrajectoryBuilderTest {

    public static void main(String[] args) {
        Stopwatch stopwatch = new Stopwatch();

        stopwatch.start();
        TrajectoryBuilder builder = new TrajectoryBuilder();
        builder.addPose(new Pose3D(0,0,Double.NaN));
        builder.addPose(new Pose3D(0,10,Math.PI));
        builder.addPose(new Pose3D(10,10,0));
        Trajectory traj = builder.build();
        stopwatch.stop();

        System.out.println(stopwatch);

    }
}
