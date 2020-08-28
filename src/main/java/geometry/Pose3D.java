package geometry;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import util.MathFunctions;

public class Pose3D {
    private double x, y, heading;

    public Pose3D() {
        this(0,0,0);
    }

    public Pose3D(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose3D(Vector2D v, double heading) {
        this(v.getX(), v.getY(), heading);
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public Vector2D getVector(){
        return new Vector2D(x,y);
    }

    @Override
    public String toString() {
        if(Double.isNaN(heading))
        {
            return "{ " + MathFunctions.round(x,1) + ", " + MathFunctions.round(y,1) + ", " + heading + "}";
        }
        return "{ " + MathFunctions.round(x,1) + ", " + MathFunctions.round(y,1) + ", " + MathFunctions.round(heading, 2) + "}";
    }

}
