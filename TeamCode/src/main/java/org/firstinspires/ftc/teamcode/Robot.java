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
    // Robot constructor creates robot object and sets up all the actuators and sensors
    Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.opMode = opMode;
        // upper arm length; lower arm length; shoulder min,max; elbow min,max; wrist min,max
        kinematics = new Kinematics(15, 12, 0, 142, 5, 180, 0, 220);

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
        setElbowAngle = elbowAngle;

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
        telemetry.addData("wrist servo", wristServo.getPosition());
        telemetry.addData("Elbow Angle", elbowAngle);
        telemetry.addData("Elbow Set angle", setElbowAngle);
        telemetry.addData("Shoulder Angle", shoulderAngle);
        telemetry.addData("Shoulder Encoder", shoulderMotor.getCurrentPosition());
        telemetry.addData("Kinematics On:",kinematicEnabled);
        telemetry.addData("armTargetX", armTargetX);
        telemetry.addData("armTargetY", armTargetY);
        telemetry.update();
    }

    /** ************** Elbow PID ********************************************
    * Reads and sets sensor values on robot for OpMode to
    * 90deg shoulder -1200 straight up and down :: aligned elbow angle 85deg
    * 0deg shoulder -3500 parallel to floor
    * shoulderEncoderPerDegree encoder clicks to a degree
     ************** Elbow PID ********************************************/
    void readSensors() {
        shoulderMaxDown = !shoulderDownLimit.getState();
        shoulderMaxUp = !shoulderUpLimit.getState();
        elbowAngle = 274 - AngleSensor.getVoltage() * 81;
        shoulderAngle = shoulderMotor.getCurrentPosition()/shoulderEncoderPerDegree + shoulderAngleShift;
        wristAngle = wristServo.getPosition();
    }

    // Initialize the imu within the expansion hub
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

    // Get the Robot heading
    float getHeading() {
        Orientation angles;
        // Get absolute orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    // Set the robot heading
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
                if(shoulderMaxUp || shoulderMaxDown) {
                    shoulderMotor.setPower(0);
                }
                telemetry.addData("Shoulder Encoder", shoulderMotor.getCurrentPosition());
                telemetry.update();
            }
            shoulderMotor.setPower(0);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    void executeKinematics() {
        KinematicData kd = kinematics.calculate(armTargetX,armTargetY);
        // returns current valid x and y
        armTargetX = kd.getX();
        armTargetY = kd.getY();
        telemetry.addData("Kinematic shoulder", kd.getShoulder());
        telemetry.addData("Kinematic elbow", kd.getElbow());
        telemetry.addData("Kinematic wrist", kd.getWrist());

        // **** Shoulder ****
        if(!Double.isNaN(kd.getShoulder())) {
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int position = (int)Math.round((kd.getShoulder() - shoulderAngleShift) * shoulderEncoderPerDegree);
            shoulderMotor.setTargetPosition(position);
            shoulderMotor.setPower(.8);
        }

        // **** Elbow ****
        setElbowAngle = kd.getElbow();

        // **** Wrist **** .00453 servo/degree
        if(!Double.isNaN(kd.getWrist())) {
            double wristPosition = (kd.getWrist()) * .00453;
            wristServo.setPosition(wristPosition);
            telemetry.addData("Wrist servo value", wristPosition);
        }
    }

    // shift the robot right/left a fixed amount
    void shift(String direction) {
        double heading = getHeading();
        double speed = .2;
        double allowance = 2;
        double angle = driveDirection * ((direction.equals("right")) ? -45 : +45);
        // shifts from current "front" direction
        move(speed, 4);
        setHeading(heading + angle, speed, allowance);
        move(speed, 2.8);
        setHeading(heading, speed, allowance);
        move(speed, -6);
    }

    // Variable Definitions for Robot
    // System
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
    protected Telemetry telemetry;
    private LinearOpMode opMode;
    private BNO055IMU imu;
    // operational values
    protected double elbowAngle;
    protected boolean shoulderMaxDown;
    protected boolean shoulderMaxUp;
    protected double setElbowAngle;
    protected double shoulderAngle;
    protected double wristAngle;
    protected boolean kinematicEnabled = false;
    protected double driveDirection = 1;
    protected double armTargetX = 0;
    protected double armTargetY = 5;
    protected boolean elbowDriven = false;
    protected Pid pidElbow;
    private double shoulderAngleShift = 142;
    private double shoulderEncoderPerDegree = 25;
    private Kinematics kinematics;

    private final double elbowPidKp = .2;     // Scale Angle error to servo speed
    private final double elbowPidTi = .2;   // Eliminate integral error in 1 sec.
    private final double elbowPidTd = .1;   // Account for error in 0.1 sec.
    // Protect against integral windup by limiting integral term.
    private final double elbowPidIntMax = 1.0;  // Limit to max speed.
    private final double elbowOutMax = 1.0;  // Motor output limited to 100%.
    private final double elbowPidIntMin = 0;  // Limit to max speed.
    private final double elbowOutMin = 0;  // Motor output limited to 100%.
}
