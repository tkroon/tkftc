package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRoverRuckus;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus;

@Autonomous(name = "tv6autotomanousfacecratermsi (Blocks to Java)", group = "")
public class tv6autotomanousfacecratermsi extends LinearOpMode {

    private DcMotor lifter;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private BNO055IMU imu;
    private VuforiaRoverRuckus vuforiaRoverRuckus;
    private TfodRoverRuckus tfodRoverRuckus;
    private Servo Test;
    private DcMotor sholderServo;

    double mineralPosition;
    ElapsedTime zElapsedTimer;
    double robotIsSleapy;
    double highSpeed;
    double turnSpeed;

    /**
     * Describe this function...
     */
    private void getLifterZero() {
        zElapsedTimer = new ElapsedTime();
        zElapsedTimer.reset();
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setTargetPosition(-10000);
        lifter.setPower(1);
        while (lifter.isBusy() && zElapsedTimer.seconds() < 3) {
            telemetry.addData("lifterEncoderPosition", lifter.getCurrentPosition());
            telemetry.update();
        }
        lifter.setPower(0);
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void holdLifterZero() {
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // 10 so it doesn't jam
        lifter.setTargetPosition(10);
        lifter.setPower(1);
    }

    /**
     * Describe this function...
     */
    private void retractLifter() {
        if (opModeIsActive()) {
            lifter.setTargetPosition(25);
            lifter.setPower(1);
        }
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double lowSpeed;
        double whichWay;

        lifter = hardwareMap.dcMotor.get("lifter");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor ");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        vuforiaRoverRuckus = new VuforiaRoverRuckus();
        tfodRoverRuckus = new TfodRoverRuckus();
        Test = hardwareMap.servo.get("Test");
        sholderServo = hardwareMap.dcMotor.get("sholderServo");

        mineralPosition = 0;
        lowSpeed = 0.6;
        highSpeed = 1;
        turnSpeed = 0.6;
        // 0 = crater 1 = depot
        whichWay = 0;
        initMotor();
        getLifterZero();
        holdLifterZero();
        initLookingOp();
        initImu();
        findgold();
        // if no minerals seen guess left, is what this does
        if (mineralPosition == 0) {
            mineralPosition = 2;
        }
        // Prompt user to press start buton.
        if (opModeIsActive()) {
            // Put run blocks here.
            robotIsSleapy = 0;
            getOffLander();
            sleepy();
            Drive(lowSpeed, 11);
            sleepy();
            retractLifter();
            sleepy();
            // right
            if (mineralPosition == 1) {
                goToTurn(-45);
                sleepy();
                Drive(lowSpeed, 20);
                sleepy();
                Drive(lowSpeed, -20);
                sleepy();
            }
            // middle
            if (mineralPosition == 2) {
                goToTurn(0);
                Drive(lowSpeed, 15);
                sleepy();
                Drive(lowSpeed, -15);
                sleepy();
            }
            // left
            if (mineralPosition == 3) {
                goToTurn(45);
                sleepy();
                Drive(lowSpeed, 20);
                sleepy();
                Drive(lowSpeed, -20);
            }
            if (whichWay == 0) {
                facingCrater();
            } else {
                facingDepot();
            }
            while (opModeIsActive()) {
                // Display orientation info.
                telemetry.addData("rot about Z", getHeading());
                telemetry.addData("Left Encoder", leftMotor.getCurrentPosition());
                telemetry.addData("Right Encoder", rightMotor.getCurrentPosition());
                telemetry.update();
            }
        }

        vuforiaRoverRuckus.close();
        tfodRoverRuckus.close();
    }

    /**
     * Describe this function...
     */
    private void findgold() {
        // Put initialization blocks here.
        mineralPosition = lookAtStuff();
        telemetry.addData("right1 center2 left3 Heck0", mineralPosition);
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void getOffLander() {
        if (opModeIsActive()) {
            zElapsedTimer = new ElapsedTime();
            zElapsedTimer.reset();
            lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lifter.setTargetPosition(4750);
            lifter.setPower(1);
            while (lifter.isBusy() && zElapsedTimer.seconds() < 3 && opModeIsActive()) {
                telemetry.addData("lifterEncoderPosition", lifter.getCurrentPosition());
                telemetry.update();
            }
            lifter.setPower(0);
        }
    }

    /**
     * Describe this function...
     */
    private void initImu() {
        BNO055IMU.Parameters imuParameters;

        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);
    }

    /**
     * Describe this function...
     */
    private void initLookingOp() {
        vuforiaRoverRuckus.initialize("", VuforiaLocalizer.CameraDirection.BACK,
                true, false, VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES,
                0, 0, 0, 0, 0, 0, true);
        tfodRoverRuckus.initialize(vuforiaRoverRuckus, (float)0.4, true, true);
    }

    /**
     * Describe this function...
     */
    private void sleepy() {
        if (robotIsSleapy == 1 && opModeIsActive()) {
            sleep(1000);
        }
    }

    /**
     * Describe this function...
     */
    private void initMotor() {
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Describe this function...
     */
    private void claimDepot() {
        if (opModeIsActive()) {
            Test.setPosition(1);
            sleep(1600);
        }
    }

    /**
     * Describe this function... things
     */
    private double lookAtStuff() {
        double leftRightOrCenter;
        List <Recognition> recognitions;
        double goldMineralX;
        double silverMineral1X;
        double silverMineral2X;

        tfodRoverRuckus.activate();
        leftRightOrCenter = 0;
        while (!isStarted()) {
            telemetry.addData(">", "Press Play to start");
            telemetry.addData("rot about z", getHeading());
            telemetry.addData("lifterEncoderPosition", lifter.getCurrentPosition());
            // Put loop blocks here.
            recognitions = tfodRoverRuckus.getRecognitions();
            telemetry.addData("# Objects Recognized", recognitions.size());
            if (recognitions.size() >= 1) {
                goldMineralX = -1;
                silverMineral1X = -1;
                silverMineral2X = -1;
                for (Recognition recognition : recognitions) {
                    if (recognition.getLabel().equals("Gold Mineral")) {
                        goldMineralX = recognition.getLeft();
                    } else if (silverMineral1X == -1) {
                        silverMineral1X = recognition.getLeft();
                    } else {
                        silverMineral2X = recognition.getLeft();
                    }
                }
                if (goldMineralX != -1) {
                    telemetry.addData("gold x", goldMineralX);
                    if (goldMineralX > 600) {
                        telemetry.addData("Gold Mineral Position", "Right");
                        leftRightOrCenter = 1;
                    } else {
                        telemetry.addData("Gold Mineral Position", "Center");
                        leftRightOrCenter = 2;
                    }
                } else {
                    telemetry.addData("Gold Mineral Position", "Left?");
                    leftRightOrCenter = 3;
                }
            }
            telemetry.update();
        }
        tfodRoverRuckus.deactivate();
        return leftRightOrCenter;
    }

    /**
     * Describe this function...
     */
    private float getHeading() {
        Orientation angles;

        // Get absolute orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Describe this function...
     */
    private void setHeading(double tgtHeading, double speed, double allowance) {
        float heading;
        double turnDirection;

        if (opModeIsActive()) {
            heading = getHeading();
            if (Math.abs(tgtHeading - heading) > allowance) {
                // 1=left 0=right
                if (tgtHeading < heading) {
                    turnDirection = 0;
                    leftMotor.setPower(speed);
                    rightMotor.setPower(-speed);
                }
                if (tgtHeading > heading) {
                    turnDirection = 1;
                    leftMotor.setPower(-speed);
                    rightMotor.setPower(speed);
                }
                while (Math.abs(tgtHeading - heading) > allowance && opModeIsActive()) {
                    heading = getHeading();
                    telemetry.addData("Current Heading", heading);
                    telemetry.addData("Target Heading", tgtHeading);
                    telemetry.update();
                }
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Drive(double speed, double distance) {
        double ePulsesPerInch;

        if (opModeIsActive()) {
            ePulsesPerInch = 1120 / (4 * Math.PI);
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + (int) Math.round(ePulsesPerInch * distance));
            rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + (int) Math.round(ePulsesPerInch * distance));
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);
            while ((leftMotor.isBusy() || rightMotor.isBusy()) && opModeIsActive()) {
                telemetry.addData("rot about Z", getHeading());
                telemetry.addData("Left Encoder", leftMotor.getCurrentPosition());
                telemetry.addData("Right Encoder", rightMotor.getCurrentPosition());
                telemetry.update();
            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }

    /**
     * Describe this function...
     */
    private void facingDepot() {
        goToTurn(85);
        sleepy();
        Drive(highSpeed, 18.5);
        sleepy();
        goToTurn(45);
        sleepy();
        Drive(highSpeed, 24);
        sleepy();
        goToTurn(137);
        sleepy();
        Drive(highSpeed, -30);
        sleepy();
        claimDepot();
        sleepy();
        goToTurn(135);
        sleepy();
        Drive(highSpeed, 58);
    }

    /**
     * Describe this function...
     */
    private void facingCrater() {
        goToTurn(-95);
        sleepy();
        // USED TO BE -51
        Drive(highSpeed, -47);
        sleepy();
        goToTurn(-47);
        sleepy();
        Drive(highSpeed, -28);
        sleepy();
        claimDepot();
        sleepy();
        // -45
        goToTurn(-40);
        sleepy();
        Drive(highSpeed, 64);
    }

    /**
     * Describe this function...
     */
    private void goToTurn(double turn) {
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setHeading(turn, turnSpeed / 1, 25);
        setHeading(turn, turnSpeed / 4, 5);
        setHeading(turn, turnSpeed / 8, 1);
    }
}
