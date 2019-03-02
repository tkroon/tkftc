package org.firstinspires.ftc.teamcode;

/**
 * Created by tkroon on 2/13/19.
 * Data holder class for Kinematics
 */

public class KinematicData {
    private double shoulderAngle;
    private double elbowAngle;
    private double wristAngle;
    private double validX;
    private double validY;

    KinematicData (double shoulder, double elbow, double spin, double x, double y){
        shoulderAngle = shoulder;
        elbowAngle = elbow;
        wristAngle = spin;
        validX = x;
        validY = y;
    }

    public double getShoulder() {
        return shoulderAngle;
    }

    public void setShoulder(double shoulder) {
        shoulderAngle = shoulder;
    }
    public double getElbow() {
        return elbowAngle;
    }

    public void setElbow(double elbow) {
        elbowAngle = elbow;
    }
    public double getWrist() {
        return wristAngle;
    }

    public void setWrist(double spin) {
        wristAngle = spin;
    }
    public double getX() {
        return validX;
    }

    public void setX(double x) {
        validX = x;
    }

    public double getY() {
        return validY;
    }

    public void setY(double y) {
        validY = y;
    }
}
