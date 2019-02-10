package org.firstinspires.ftc.teamcode;
/**
 * Created by tkroon on 2/9/19.
 *
 */

public class KinematicsInv {
    //The lengths of the two segments of the robot’s arm. Using the same length for both
    // segments allows the robot to reach the (0,0) coordinate.
    private double len1;
    private double len2;

    // Kinematics constructor
    KinematicsInv(double len1, double len2) {
        this.len1 = len1;
        this.len2 = len2;
    }

    // The law of cosines, transformed so that C is the unknown. The names of the sides
    // and angles correspond to the standard names in mathematical writing.
    // Later, we have to map the sides and angles from our scenario to a, b, c, and C, respectively.
    private static double  lawOfCosines(double a, double b, double c)  {
        return Math.acos((a*a + b*b - c*c) / (2 * a * b));
    }
    // The distance from (0,0) to (x,y). HT to Pythagoras.
    private static double distance(double x, double y ){
        return Math.sqrt(x*x + y*y);
    }

    // Calculating the two joint angles for given x and y.
    public double[] angles(double x, double y) {
        double[] angles = new double[] {0,0};
        // First, get the length of line dist.
        double dist = distance(x, y);
        // Calculating angle D1 is trivial. Atan2 is a modified arctan() function that
        // returns unambiguous results.
        double D1 = Math.atan2(y, x);
        // D2 can be calculated using the law of cosines
        // where a = dist, b = len1, and c = len2.
        double D2 = lawOfCosines(dist, len1, len2);
        // Then angles[0] is simply the sum of D1 and D2.
        angles[0] = Math.toDegrees(D1 + D2);
        // angles[1] can also be calculated with the law of cosine,
        // but this time with a = len1, b = len2, and c = dist.
        angles[1] = Math.toDegrees(lawOfCosines(len1, len2, dist));

        return angles;
    }
}