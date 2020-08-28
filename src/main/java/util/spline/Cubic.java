package util.spline;

public class Cubic {
    private double a,b,c,d;

    public Cubic(double a, double b, double c, double d) {
        this.a =a;
        this.b =b;
        this.c =c;
        this.d =d;
    }

    public double eval(double u) {
        return (((d*u) + c)*u + b)*u + a;
    }
}