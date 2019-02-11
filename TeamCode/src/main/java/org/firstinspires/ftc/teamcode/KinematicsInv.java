package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

/**
 * https://appliedgo.net/roboticarm/
 * Adapted by tkroon on 2/9/19.
 *
 */

public class KinematicsInv {
    //The lengths of the two segments of the robot’s arm. Using the same length for both
    // segments allows the robot to reach the (0,0) coordinate.
    private double len1;
    private double len2;
    private double angleLimits [][] = {{0, 90}, {0, 90}, {0, 90}};


    // Kinematics constructor
    KinematicsInv(double len1, double len2) {
        this.len1 = len1;
        this.len2 = len2;
    }

    KinematicsInv(double len1, double len2, int minA1, int maxA1, int minA2, int maxA2, int minA3, int maxA3) {
        this.len1 = len1;
        this.len2 = len2;
        angleLimits[0][0] = minA1;
        angleLimits[0][1] = maxA1;
        angleLimits[1][0] = minA2;
        angleLimits[1][1] = maxA2;
        angleLimits[2][0] = minA3;
        angleLimits[2][1] = maxA3;
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
        double[] angles = new double[] {0,0,0};
        // First, get the length of line dist.
        double dist = distance(x, y);
        // Calculating angle A1 is trivial. Atan2 is a modified arctan() function that
        // returns unambiguous results.
        double A1 = Math.atan2(y, x);
        // A2 can be calculated using the law of cosines
        // where a = dist, b = len1, and c = len2.
        double A2 = lawOfCosines(dist, len1, len2);
        // Then angles[0] is simply the sum of D1 and D2.
        angles[0] = Range.clip(Math.toDegrees(A1 + A2),angleLimits[0][0],angleLimits[0][1]);
        // angles[1] can also be calculated with the law of cosine,
        // but this time with a = len1, b = len2, and c = dist.
        angles[1] = Range.clip(Math.toDegrees(lawOfCosines(len1, len2, dist)),angleLimits[1][0],angleLimits[1][1]);
        // compute angle D3
        angles[2] = Range.clip(angles[0] + angles[1],angleLimits[2][0],angleLimits[2][1]);;
        return angles;
    }
}