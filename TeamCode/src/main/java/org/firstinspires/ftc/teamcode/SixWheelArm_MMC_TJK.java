package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class SixWheelArm_MMC_TJK extends LinearOpMode {
    private Robot robot;
    private double pidPower = .20;
    private boolean elbowDriven;
    private double drivePowerScale = 1;
    private double shoulderPowerScale = .60;
    private double elbowPowerScale = 1;
    private double wristIncrement = 0.017;

    @Override
    public void runOpMode() {
        // initialize Robot
        robot = new Robot(hardwareMap, telemetry, this);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.readSensors();
            robot.sendTelemetry();

            //**************** Drive direction (Controller A) *********************************
            // Control: left_stick_button :: normal forward drive / alternate 180deg drive
            //***********************************************************************
            if (gamepad1.left_stick_button){
                if (robot.driveDirection == 1) {
                    robot.driveDirection = -1;
                } else {
                    robot.driveDirection = 1;
                }
            }

            //**************** Drive (Controller A) *********************************
            // Control: Left Stick
            // Proportional drive with left stick (Arcade Style)
            //***********************************************************************
            double drive = robot.driveDirection * gamepad1.left_stick_y;
            double turn  = gamepad1.left_stick_x;
            double leftMotorPower  = Range.clip((drive - turn)* drivePowerScale, -1.0, 1.0);
            double rightMotorPower = Range.clip((drive + turn) * drivePowerScale, -1.0, 1.0);
            robot.leftMotor.setPower(leftMotorPower);
            robot.rightMotor.setPower(rightMotorPower);

            //**************** Arm Control (Controller A) ******************************
            // mode is kinematic or manual
            //**************************************************************************
            // Switch between kinematic and manual pushing down right stick
            if (gamepad1.right_stick_button) {
                robot.kinematicEnabled = !robot.kinematicEnabled;
            }

            if (robot.kinematicEnabled) {
                robot.armTargetX += gamepad1.right_stick_x/2;
                robot.armTargetY -= gamepad1.right_stick_y/2;
                robot.armTargetX = Range.clip(robot.armTargetX, 0,27);
                robot.armTargetY = Range.clip(robot.armTargetY, 0,27);
                elbowDriven = false;
                robot.executeKinematics();
            } else {
                //**************** Shoulder (Controller A) ******************************
                // Control: Right stick left/right :: Shoulder in/out
                // so x moves shoulder in and out (down/out -) (up/in +)
                // stick left - / right +
                // y moves elbow up and down
                //***********************************************************************
                robot.shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (gamepad1.right_stick_x > 0 && !robot.shoulderMaxDown || //Stick right value  +
                        gamepad1.right_stick_x < 0 && !robot.shoulderMaxUp) { //Stick left value -
                    robot.shoulderMotor.setPower(-gamepad1.right_stick_x * shoulderPowerScale);
                } else {
                    robot.shoulderMotor.setPower(0);
                }

                //**************** Elbow (Controller A) *********************************
                // Control: Left stick up/down :: Elbow up/down
                // y moves elbow up and down
                //***********************************************************************
                if (gamepad1.right_stick_y > 0 && robot.elbowAngle < 270 || //Stick up value  +
                        gamepad1.right_stick_y < 0 && robot.elbowAngle > 2) { //Stick down value -
                    robot.elbowServo.setPower(-gamepad1.right_stick_y * elbowPowerScale);
                    robot.setElbowAngle = robot.elbowAngle;
                    elbowDriven = true;
                } else {
                    robot.elbowServo.setPower(0);
                    elbowDriven = false;
                }

                //**************** Wrist (Controller A) *********************************
                // Control: Dpad up/down :: Wrist up/down
                //***********************************************************************
                if (gamepad1.dpad_up) {
                    robot.wristServo.setPosition(robot.wristServo.getPosition() + wristIncrement);
                } else if (gamepad1.dpad_down) {
                    robot.wristServo.setPosition(robot.wristServo.getPosition() - wristIncrement);
                }
            }
            //**************** Mineral Spinner (Controller A) ***********************
            // Control: right bumper/trigger :: Spinner release/collect
            // Trigger dominant collects. Bumper less dominant releases.
            // TODO: add code so it automatically collects when shoulder is down and releases when up
            //***********************************************************************
            if (gamepad1.right_bumper) {
                robot.spinnerServo.setPower(-.8); // -releases
            } else if (gamepad1.right_trigger == 1) {
                robot.spinnerServo.setPower(+.8); // +collects
            } else {
                robot.spinnerServo.setPower(0); // stop spinning
            }

            //**************** Lifter (Controller A) **********************************
            // Control: left bumper/trigger :: Lift lift up/down
            // bumper = up Trigger = down
            //***********************************************************************
            if (gamepad1.left_trigger == 1) {
                robot.lifter.setPower(-1);
            } else if (gamepad1.left_bumper) {
                robot.lifter.setPower(1);
            } else {
                robot.lifter.setPower(0);
            }

            // keeps elbow up due to dropping weight
            elbowPid();

            // TODO: Buttons for: Fold, Retrieve, Dump
            //**************** Special functions (Controller A) *********************
            // Most of these special functions block other operations until finished
            //***********************************************************************
            // Control: X/B :: shift left/shift right
            //***********************************************************************
            if (gamepad1.b) {
                shiftRobot("right");
            }
            if (gamepad1.x){
                shiftRobot("left");
            }

            //*************** Shift robot left/right ********************************
            // Control: Dpad-left/right :: dump/retrieve
            //     0 Folded :: elbow 169deg :: wrist 0.267
            // -1200 straight up and down :: aligned elbow angle 85deg
            // -3500 parallel to floor
            //***********************************************************************
            if(gamepad1.dpad_left) {
                // setShoulderPos blocks operation until it is set
                robot.setShoulderPos(-1300, .5); // straight up
            }

            /*
            if(gamepad1.dpad_right) {
                robot.setShoulderPos(-1300, .5); // straight up
                robot.setShoulderPos(-2000, .5); // parallel to floor
                robot.wristServo.setPosition(0.47);
            }
            */
        }
    }

    //**************** Elbow PID ********************************************
    // Keep elbow at last set angle.
    // elbow up + / down -
    // 90deg straight up
    // angle gets greater as forearm moves to floor
    // positive error means arm below set pos :: need (+) power
    // negative error means arm above set pos :: need (-) power
    //***********************************************************************
    void elbowPid() {
        double error = robot.setElbowAngle - robot.elbowAngle;
        telemetry.addData("angle Error", error);
        if (!elbowDriven) {
            if (error > 0) { //posivite needs + power arm up
                robot.elbowServo.setPower(pidPower + (error / 11));
            } else if (error < 0) { // negative needs - power arm down
                robot.elbowServo.setPower(-pidPower + (error / 11));
            }
        }
    }

    void shiftRobot(String direction) {
        double heading = robot.getHeading();
        double speed = .2;
        double allowance = 2;
        double angle = robot.driveDirection * ((direction.equals("right")) ? -45 : +45);
        // shifts from current "front" direction
        if (gamepad1.b) {
            robot.move(speed, 4);
            robot.setHeading(heading + angle, speed, allowance);
            robot.move(speed, 2.8);
            robot.setHeading(heading, speed, allowance);
            robot.move(speed, -6);
        }
    }
}