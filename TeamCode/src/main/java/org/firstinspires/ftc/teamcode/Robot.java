package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by tkroon on 2/7/19.
 * Abstracts all the parts of the Robot to clarify the OpMode
 * Inputs: takes the hardwareMap and telemetry from the opmode's context
 */

public class Robot {
    protected DcMotor leftMotor;
    protected DigitalChannel shoulderDownLimit;
    protected DigitalChannel shoulderUpLimit;
    protected CRServo elbowServo;
    protected Servo clawServo;
    protected Servo wristServo;
    protected DcMotor rightMotor;
    protected DcMotor lifter;
    protected DcMotor shoulderMotor;
    protected CRServo spinnerServo;
    protected AnalogInput AngleSensor;
    protected Servo avatarServo;
    protected HardwareMap hardwareMap;
    protected double angle;
    protected Telemetry telemetry;
    protected boolean shoulderMaxDown;
    protected boolean shoulderMaxUp;
    private LinearOpMode opMode;
    private BNO055IMU imu;

    // Robot constructor creates robot object and sets up all the actuators and sensors
    Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.opMode = opMode;

        //**************** Motors ************************************/
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lifter = hardwareMap.get(DcMotor.class, "lifter");

        shoulderMotor = hardwareMap.get(DcMotor.class, "sholderServo");
        shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //**************** Servos ************************************/
        elbowServo = hardwareMap.get(CRServo.class, "elbowservo");
        wristServo = hardwareMap.get(Servo.class, "wristservo");
        clawServo = hardwareMap.get(Servo.class, "clawservo");
        spinnerServo = hardwareMap.get(CRServo.class, "clawServo2");
        avatarServo = hardwareMap.get(Servo.class, "Test");

        //**************** Sensors ************************************/
        shoulderDownLimit = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        shoulderDownLimit.setMode(DigitalChannel.Mode.INPUT);

        shoulderUpLimit = hardwareMap.get(DigitalChannel.class, "digitalTouch2");
        shoulderUpLimit.setMode(DigitalChannel.Mode.INPUT);

        AngleSensor = hardwareMap.get(AnalogInput.class, "AngleSensor");

        // initialize hardware and data
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        readSensors();
        sendTelemetry();
        initImu();

        if(shoulderMaxUp) { // if arm is at top limit folded reset encoder to zero
            shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //     0 Folded :: elbow 169deg :: wrist 0.267
            // -1200 straight up and down :: alinged elbow angle 85deg
            // -3500 parallel to floor
        }
    }

    // Sends messages and values to bottom of driver's screen
    void sendTelemetry() {
        telemetry.addData("Status", "Running");
        telemetry.addData("ShoulderMaxDown", shoulderMaxDown);
        telemetry.addData("ShoulderMaxUp", shoulderMaxUp);
        telemetry.addData("Left Motor Power", leftMotor.getPower());
        telemetry.addData("Right Motor Power", rightMotor.getPower());
        telemetry.addData("Shoulder Servo port 1", shoulderMotor.getPower());
        telemetry.addData("Elbow servo port 2", elbowServo.getPower());
        telemetry.addData("Spinner servo", spinnerServo.getPower());
        telemetry.addData("wrist servo port ", wristServo.getPosition());
        telemetry.addData("volts", AngleSensor.getVoltage());
        telemetry.addData("Angle", angle);
        telemetry.addData("Shoulder Encoder", shoulderMotor.getCurrentPosition());
        telemetry.update();
    }

    // Reads and sets sensor values on robot for OpMode to use
    void readSensors() {
        shoulderMaxDown = !shoulderDownLimit.getState();
        shoulderMaxUp = !shoulderUpLimit.getState();
        angle = (AngleSensor.getVoltage()) * 81;
    }

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

    // Move straight speed and distance in inches
    void move(double speed, double distance) {
        double ePulsesPerInch;

        if (opMode.opModeIsActive()) {
            ePulsesPerInch = 1120 / (4 * Math.PI);
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + (int) Math.round(ePulsesPerInch * distance));
            rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + (int) Math.round(ePulsesPerInch * distance));
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);
            while ((leftMotor.isBusy() || rightMotor.isBusy()) && opMode.opModeIsActive()) {
                //telemetry.addData("rot about Z", getHeading());
                telemetry.addData("Left Encoder", leftMotor.getCurrentPosition());
                telemetry.addData("Right Encoder", rightMotor.getCurrentPosition());
                telemetry.update();
            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    float getHeading() {
        Orientation angles;
        // Get absolute orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    void setHeading(double tgtHeading, double speed, double allowance) {
        float heading;

        if (opMode.opModeIsActive()) {
            heading = getHeading();
            if (Math.abs(tgtHeading - heading) > allowance) {
                // 1=left 0=right
                if (tgtHeading < heading) {
                    leftMotor.setPower(-speed);
                    rightMotor.setPower(speed);
                }
                if (tgtHeading > heading) {
                    leftMotor.setPower(speed);
                    rightMotor.setPower(-speed);
                }
                while (Math.abs(tgtHeading - heading) > allowance && opMode.opModeIsActive()) {
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

    // Move shoulder speed and position
    void setShoulderPos(int position, double speed) {
        if (opMode.opModeIsActive()) {

            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setTargetPosition(position);
            shoulderMotor.setPower(speed);
            while (shoulderMotor.isBusy() && opMode.opModeIsActive()) {
                telemetry.addData("Shoulder Encoder", shoulderMotor.getCurrentPosition());
                telemetry.update();
            }
            shoulderMotor.setPower(0);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
