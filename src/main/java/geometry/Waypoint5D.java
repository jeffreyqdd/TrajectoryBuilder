package geometry;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import util.MathFunctions;

public class Waypoint5D {
    private double x, y, curvature, velocity, heading;


    public Waypoint5D() {
        this(0,0,0,0,Double.NaN);
    }

    public Waypoint5D(Vector2D v, double curvature, double velocity) {
        this(v.getX(), v.getY(), curvature, velocity, Double.NaN);
    }

    public Waypoint5D(Vector2D v, double curvature, double velocity, double heading) {
        this(v.getX(), v.getY(), curvature, velocity, heading);
    }

    public Waypoint5D(double x, double y, double curvature, double velocity, double heading) {
        this.x = MathFunctions.round(x,2);
        this.y = MathFunctions.round(y,2);
        this.curvature = MathFunctions.round(curvature,2);
        this.velocity = MathFunctions.round(velocity,2);

        if(Double.isNaN(heading))
            this.heading = heading;
        else
            this.heading = MathFunctions.round(heading,2);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Vector2D getPoint() {return new Vector2D(x,y); }

    public double getCurvature() {
        return curvature;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getHeading() {
        return heading;
    }

    @Override
    public String toString() {
        return "{ position: (" + x + ", " + y + "), curvature: " + curvature + ", velocity: " + velocity + ", heading: " + heading + "}";
    }
}
