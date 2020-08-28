//package util.interactive.visualizer;
//
//import geometry.Pose3D;
//import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
//import trajectory.Trajectory;
//import trajectory.TrajectoryBuilder;
//import trajectory.TrajectoryConstraints;
//import util.StdDraw;
//import util.Stopwatch;
//
//import java.awt.*;
//import java.awt.event.KeyEvent;
//import java.util.ArrayList;
//import java.util.List;
//
//


import follower.HolonomicPurePursuitFollower;
import follower.PurePursuit;
import geometry.Pose3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import trajectory.Trajectory;
import trajectory.TrajectoryBuilder;
import trajectory.TrajectoryConstraints;
import util.MathFunctions;
import util.StdDraw;

import java.awt.*;
import java.awt.event.KeyEvent;
import java.io.IOException;
import java.lang.management.ManagementFactory;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.sun.management.OperatingSystemMXBean;
import util.Stopwatch;

import javax.management.MBeanServerConnection;

public class Simulation implements Runnable{

    private static final double renderRate = 20d;
    private static final double logicRate = 30d;

    private static Pose3D robotPose;
    private static boolean showRobot;
    private static PurePursuit follower = null;
    private static List<Pose3D> robotPath;
    private static Trajectory traj;
    private static int selectedPoint = -1;


    private static RenderBuilder renderBuilder;

    static {
        try {
            renderBuilder = new RenderBuilder();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void init() {
        int WIDTH = 1200;
        int HEIGHT = 720;
        int adjustedScaleX = WIDTH / (HEIGHT / 72);

        StdDraw.setCanvasSize(WIDTH,HEIGHT);
        StdDraw.setXscale(-adjustedScaleX, adjustedScaleX);
        StdDraw.setYscale(-72, 72);

        StdDraw.enableDoubleBuffering();

        robotPath = new ArrayList<>();


    }

    private void builderLogic() {
        //UI
        if(StdDraw.isKeyPressed(KeyEvent.VK_Q))
        {
            while(StdDraw.isKeyPressed(KeyEvent.VK_Q)){}

            robotPath.add(new Pose3D(StdDraw.mouseX(), StdDraw.mouseY(), Double.NaN));

        }
        if(StdDraw.isKeyPressed(KeyEvent.VK_E))
        {
            while(StdDraw.isKeyPressed(KeyEvent.VK_E)){}
            if(robotPath.size() > 0) {
                robotPath.remove(robotPath.size() - 1);
                selectedPoint = Math.min(selectedPoint, robotPath.size() - 1);
            }
        }
        if(StdDraw.isKeyPressed(KeyEvent.VK_W) && traj != null)
        {
            while(StdDraw.isKeyPressed(KeyEvent.VK_W)){}
            showRobot = true;
            follower = new HolonomicPurePursuitFollower(traj, 24,12,6);
            robotPose = new Pose3D(robotPath.get(0).getVector(), 0d);
        }
        if(StdDraw.isKeyPressed(KeyEvent.VK_R))
        {
            while(StdDraw.isKeyPressed(KeyEvent.VK_R)){}

            for(Pose3D v : robotPath)
            {
                System.out.println("builder.addPose(new Pose3D(" +v.getX() + ", " + v.getY() + ", " + (Double.isNaN(v.getHeading()) ? "Double.NaN" : v.getHeading()) +  "));");
            }
        }

        if(StdDraw.isKeyPressed(KeyEvent.VK_A) && selectedPoint != -1)
        {
            robotPath.set(selectedPoint, new Pose3D(StdDraw.mouseX(), StdDraw.mouseY(), robotPath.get(selectedPoint).getHeading()));
        }
        if(StdDraw.isKeyPressed(KeyEvent.VK_S))
        {
            while(StdDraw.isKeyPressed(KeyEvent.VK_S)){}
            selectedPoint = -1;
        }
        if(StdDraw.isKeyPressed(KeyEvent.VK_D) && selectedPoint != -1)
        {
            while(StdDraw.isKeyPressed(KeyEvent.VK_A)){}

            Vector2D v = (new Vector2D(StdDraw.mouseX(), StdDraw.mouseY())).subtract(robotPath.get(selectedPoint).getVector());

            double angle = Math.atan2(v.getY(), v.getX());

            if(selectedPoint != -1) robotPath.set(selectedPoint, new Pose3D(robotPath.get(selectedPoint).getVector(), angle));
        }


        if(StdDraw.isMousePressed()) {
            double d = 1;
            Vector2D loc = new Vector2D(StdDraw.mouseX(), StdDraw.mouseY());

            for (int i = 0; i < robotPath.size(); i++) {
                if (robotPath.get(i).getVector().distance(loc) < d) {
                    //while (StdDraw.isMousePressed()) {
                    //robotPath.set(i, new Pose3D(StdDraw.mouseX(), StdDraw.mouseY(), robotPath.get(i).getHeading()));
                    selectedPoint = i;
                    //}
                }
            }
        }

        //Logic
        Stopwatch watch = new Stopwatch();

        watch.start();
        if(robotPath.size() >= 2) {
            TrajectoryBuilder builder = new TrajectoryBuilder();
            for (Pose3D p : robotPath) builder.addPose(p);
            traj = builder.build();
            watch.stop();
            renderBuilder.updateInfo(robotPath, traj, selectedPoint, watch.elapsed(), robotPose);
        }
        else if(robotPath.size() >= 1) {
            watch.stop();
            renderBuilder.updateInfo(robotPath, null, selectedPoint, watch.elapsed(), robotPose);
            follower = null;
        }



    }

    private void followerLogic() {
        if(!showRobot)
            return;


        follower.updateRobotPose(robotPose);

        double rotation = follower.getRotation();
        Vector2D velocity = follower.getVelocity();
        //System.out.println(renderBuilder.logicFPS);
        //System.out.println(robotPose + ": " + velocity.getX() *  renderBuilder.logicFPS + ", " + velocity.getY() * renderBuilder.logicFPS);
        renderBuilder.follower = follower;

        robotPose = new Pose3D(
                robotPose.getX() + (velocity.getX() *  renderBuilder.logicFPS),
                robotPose.getY() +  (velocity.getY() *  renderBuilder.logicFPS),
                robotPose.getHeading() + rotation * renderBuilder.logicFPS);

        if(follower.getError() < 1)
            showRobot = false;

    }


    private void executeRun() throws IOException {
        Stopwatch watch = new Stopwatch();

        watch.start();
        long lastLogicTick = 0L;
        long lastRenderTick = 0L;

        while (true) {

            long elasped = watch.elapsed();

            try {
                if (((elasped - lastLogicTick) / 1000000000.0) >= 1d / logicRate) {
                    run();
                    renderBuilder.logicFPS = (elasped - lastLogicTick) / 1000000000.0;
                    lastLogicTick = elasped;
                }
                if (((elasped - lastRenderTick) / 1000000000.0) >= 1d / renderRate) {
                    renderBuilder.graphicsFPS = (elasped - lastRenderTick) / 1000000000.0;
                    renderBuilder.run();
                    lastRenderTick = elasped;
                }
            }
            catch (Exception e) {
                //System.out.println("error with new point");
                e.printStackTrace();
                break;
            }

        }
    }


    public static void main(String[] args) throws IOException {
        Simulation sim = new Simulation();

        sim.init();
        sim.executeRun();
    }

    @Override
    public void run() {
        builderLogic();
        followerLogic();
    }
}

//doubles as UI
class RenderBuilder implements Runnable {

    List<Pose3D> robotPath;
    Pose3D robotPose;
    Trajectory robotTrajectory;
    int selectedPoint = -1;
    long buildTime;

    double load = 0d;
    public double graphicsFPS = 0;
    public double logicFPS = 0;
    long cpuUsuage = 0;
    public PurePursuit follower;

    //cpu usuage
    ArrayList<Long> runningAvg = new ArrayList<>();
    MBeanServerConnection mbsc = ManagementFactory.getPlatformMBeanServer();

    OperatingSystemMXBean osMBean = ManagementFactory.newPlatformMXBeanProxy(
            mbsc, ManagementFactory.OPERATING_SYSTEM_MXBEAN_NAME, OperatingSystemMXBean.class);

    RenderBuilder() throws IOException {
        for(int i = 0; i < 10; i++)
            runningAvg.add(10L);
    }


    private void render() {
        StdDraw.clear();

        //render border
        StdDraw.setPenColor(StdDraw.BLACK);
        StdDraw.setPenRadius(0.01);
        StdDraw.line(-72, -72, -72, 72);
        StdDraw.line(72, -72, 72, 72);


        //render center image
        StdDraw.picture(0,0, "src/main/java/util/assets/map.png", 144, 144);

        //render instructions image
        StdDraw.setFont(new Font(Font.MONOSPACED, Font.PLAIN ,12));
        StdDraw.textLeft(75, 70, "q - new point on cursor");
        StdDraw.textLeft(75, 65, "e - remove latest point");
        StdDraw.textLeft(75, 60, "w - run simulation");
        StdDraw.textLeft(75, 55, "r - export path to console");

        StdDraw.textLeft(75, 45, "mouse to select point");
        StdDraw.textLeft(75, 40, "a - move selected point");
        StdDraw.textLeft(75, 35, "d - rotate selected point");
        StdDraw.textLeft(75, 30, "s - de-select point");

        //render text and image for path points
        if(robotPath != null) {
            for (int i = 0; i < robotPath.size(); i++) {
                StdDraw.textLeft(-120, 70 - 5 * i, robotPath.get(i).toString());
            }

            for(int i = 0; i < robotPath.size(); i++) {
                StdDraw.setPenRadius(0.02);
                if(i == selectedPoint) StdDraw.setPenColor(StdDraw.YELLOW);
                else StdDraw.setPenColor(StdDraw.BLACK);
                StdDraw.point(robotPath.get(i).getX(), robotPath.get(i).getY());

                if(selectedPoint == i && !Double.isNaN(robotPath.get(i).getHeading())) {
                    StdDraw.setPenRadius(0.007);
                    StdDraw.line(robotPath.get(i).getX(), robotPath.get(i).getY(),
                            robotPath.get(i).getX() + (Math.cos(robotPath.get(i).getHeading()) * 5d),
                            robotPath.get(i).getY() + (Math.sin(robotPath.get(i).getHeading()) * 5d));
                }
            }

        }
        if(robotTrajectory != null) {
            StdDraw.setPenColor(StdDraw.BLACK);
            StdDraw.setPenRadius(0.003);
            double distance = 0;
            TrajectoryConstraints constraints = new TrajectoryConstraints();
            for (int i = 0; i < robotTrajectory.size() - 1; i++) {

                double percentage = (robotTrajectory.velocity(i) / constraints.maxVelocity);
                int number =(int)(percentage * 255);
                number = Math.max(0, Math.min(number, 255));

                StdDraw.setPenColor(255 - number, number, 0);

                distance += robotTrajectory.v(i).distance(robotTrajectory.v(i + 1));
                Vector2D v1 = robotTrajectory.v(i), v2 = robotTrajectory.v(i + 1);
                StdDraw.line(v1.getX(), v1.getY(), v2.getX(), v2.getY());
            }

            StdDraw.setPenColor(StdDraw.RED);
            StdDraw.setFont(new Font(Font.MONOSPACED, Font.PLAIN ,12));
            StdDraw.textLeft(75, 5, "build time: " + (buildTime / 1000000L) + " milliseconds");
            StdDraw.textLeft(75, 0, "distance: " + MathFunctions.round(distance,2) + " inches");
        }
        StdDraw.setPenColor(StdDraw.RED);
        StdDraw.setFont(new Font(Font.MONOSPACED, Font.PLAIN ,12));
        StdDraw.textLeft(75, -5, "cpu load: " + cpuUsuage);
        StdDraw.textLeft(75, -10, "logic: " +  MathFunctions.round(1d/ logicFPS,0) + "fps");
        StdDraw.textLeft(75, -15, "graphics: " +  MathFunctions.round(1d/ graphicsFPS,0) + "fps");

        if(robotPose != null) {
            StdDraw.setPenColor(StdDraw.BLUE);
            StdDraw.setPenRadius(0.01);
            StdDraw.point(robotPose.getX(), robotPose.getY());
            Vector2D v = follower.getLookaheadPoint();
            Vector2D v2 = follower.getClosestPoint();
            StdDraw.point(v.getX(), v.getY());

            StdDraw.setPenColor(StdDraw.YELLOW);
            StdDraw.point(v2.getX(), v2.getY());
            StdDraw.line(robotPose.getX(),robotPose.getY(),
                    robotPose.getX() + (Math.cos(robotPose.getHeading()) * 5d),
                    robotPose.getY() + (Math.sin(robotPose.getHeading()) * 5d));

        }


        StdDraw.show();
    }
    public void updateInfo(List<Pose3D> robotPath, Trajectory robotTrajectory, int selectedPoint, long buildTime, Pose3D robotPose) {
        this.robotPath = robotPath;
        this.robotTrajectory = robotTrajectory;
        this.selectedPoint = selectedPoint;
        this.buildTime = buildTime;
        this.robotPose = robotPose;
    }

    private void wrap(){
        long nanoBefore = System.nanoTime();
        long cpuBefore = osMBean.getProcessCpuTime();
        render();


        long cpuAfter = osMBean.getProcessCpuTime();
        long nanoAfter = System.nanoTime();

        long percent;
        if (nanoAfter > nanoBefore)
            percent = ((cpuAfter-cpuBefore)*100L)/
                    (nanoAfter-nanoBefore);
        else percent = 0;

        runningAvg.remove(0);
        runningAvg.add(percent);
        long sum = 0;
        for(Long l : runningAvg)
            sum += l;

        cpuUsuage = sum / runningAvg.size();
    }
    @Override
    public void run() {

        wrap();
    }
}