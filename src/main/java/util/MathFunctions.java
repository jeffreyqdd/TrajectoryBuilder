package util;

public class MathFunctions {
    public static double angleWrap(double angle) {
        while(angle < -java.lang.Math.PI) {
            angle += 2d * java.lang.Math.PI;
        }
        while(angle > java.lang.Math.PI) {
            angle -= 2d * java.lang.Math.PI;
        }

        return angle;
    }

    public static double round(double num, int numDecimals){
        return Math.round(num * Math.pow(10, numDecimals)) / Math.pow(10, numDecimals);
    }


}
