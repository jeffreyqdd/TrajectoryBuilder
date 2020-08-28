package util.spline;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import util.spline.BasicSpline;
import util.spline.Cubic;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.List;
import java.util.Vector;

public class Spline2D extends BasicSpline {
    private Vector<Vector2D> points;

    private Vector<Cubic> xCubics;
    private Vector<Cubic> yCubics;

    private static final String vector2DgetXMethodName = "getX";
    private static final String vector2DgetYMethodName = "getY";

    private Method vector2DgetXMethod;
    private Method vector2DgetYMethod;

    private static final Object[] EMPTYOBJ = new Object[] { };

    public Spline2D(List<Vector2D> points) {
        this.points = new Vector<>(points);
        this.xCubics = new Vector<>();
        this.yCubics = new Vector<>();

        try {
            vector2DgetXMethod = Vector2D.class.getDeclaredMethod(vector2DgetXMethodName, new Class[] { });
            vector2DgetYMethod = Vector2D.class.getDeclaredMethod(vector2DgetYMethodName, new Class[] { });
        } catch (SecurityException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (NoSuchMethodException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }


    public void addPoint(Vector2D point) {
        this.points.add(point);
    }

    public Vector<Vector2D> getPoints() {
        return points;
    }

    public void calcSpline() {
        try {
            calcNaturalCubic(points, vector2DgetXMethod, xCubics);
            calcNaturalCubic(points, vector2DgetYMethod, yCubics);
        } catch (IllegalArgumentException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IllegalAccessException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (InvocationTargetException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public Vector2D getPoint(double position) {

        position = position * xCubics.size(); // extrapolate to the arraysize
        int      cubicNum = (int) position;
        double   cubicPos = (position - cubicNum);
        return new Vector2D(xCubics.get(cubicNum).eval(cubicPos),
                yCubics.get(cubicNum).eval(cubicPos));
    }
}
