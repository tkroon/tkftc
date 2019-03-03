package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class SixWheelArm_MMC_TJK extends LinearOpMode {
    private Robot robot;
    private double drivePowerScale = 1;
    private double shoulderPowerScale = .60;
    private double elbowPowerScale = 1;
    private double wristIncrement = 0.017;
    //ElapsedTime elapsedTimer;

    @Override
    public void runOpMode() {
        // initialize Robot
        robot = new Robot(hardwareMap, telemetry, this);
        PIDController pid = new PIDController(.2,.03,.03, 20);
        pid.enable();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.readSensors();
            robot.sendTelemetry();

            //**************** Drive direction toggle (Controller A) *********************
            // Control: left_stick_button :: normal forward drive / alternate 180deg drive
            //****************************************************************************
            if (gamepad1.left_stick_button) {
                if (robot.driveDirection == 1) {
                    robot.driveDirection = -1;
                } else {
                    robot.driveDirection = 1;
                }
            }

            //**************** Arm Control (Controller A) ******************************
            // Control: right_stick_button
            // Mode toggle kinematic or manual
            //**************************************************************************
            // Switch between kinematic and manual pushing down right stick
            if (gamepad1.right_stick_button) {
                robot.kinematicEnabled = !robot.kinematicEnabled;
            }

            //**************** Drive (Controller A) *********************************
            // Control: Left Stick
            // Proportional drive with left stick (Arcade Style)
            //***********************************************************************
            double drive = robot.driveDirection * gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            double leftMotorPower = Range.clip((drive - turn) * drivePowerScale, -1.0, 1.0);
            double rightMotorPower = Range.clip((drive + turn) * drivePowerScale, -1.0, 1.0);
            robot.leftMotor.setPower(leftMotorPower);
            robot.rightMotor.setPower(rightMotorPower);

            // Use Arm Control based on Right Stick Choice
            if (robot.kinematicEnabled) { // Drive using Kinematic Control
                robot.armTargetX += gamepad1.right_stick_x / 2;
                robot.armTargetY -= gamepad1.right_stick_y / 2;
                robot.armTargetX = Range.clip(robot.armTargetX, 0, 27);
                robot.armTargetY = Range.clip(robot.armTargetY, 0, 27);
                robot.executeKinematics();
                doPid(pid);
            } else { // Drive Manual Control
                //**************** Shoulder (Controller A) ******************************
                // Control: Right stick left/right :: Shoulder in/out
                // x moves shoulder in and out (left to robot, right away from robot)
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
                // y moves elbow section up and down
                //***********************************************************************
                if (gamepad1.right_stick_y > 0 && robot.elbowAngle < 270 || //Stick up value  +
                        gamepad1.right_stick_y < 0 && robot.elbowAngle > 2) { //Stick down value -
                    robot.elbowServo.setPower(-gamepad1.right_stick_y * elbowPowerScale);
                    robot.setElbowAngle = robot.elbowAngle;
                    robot.elbowDriven = true;
                } else {
                    robot.elbowDriven = false;
                    doPid(pid);
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

            // TODO: Buttons for: Fold, Retrieve, Dump
            /** *************** Special functions (Controller A) ****************************
             * Note: Most of these special functions block other operations until finished
             ***********************************************************************
             * Control: X/B :: shift left/shift right
             ***********************************************************************/
            if (gamepad1.b) {
                robot.shift("right");
            }
            if (gamepad1.x) {
                robot.shift("left");
            }

            //*************** Notes *************************************************
            //     0 Folded :: elbow 169deg :: wrist 0.267
            // -1200 straight up and down :: aligned elbow angle 85deg
            // -3500 parallel to floor
            //***********************************************************************
        }
    }

    void doPid(PIDController pid) {
        pid.setSetpoint(robot.setElbowAngle);
        double correction = pid.performPID(robot.elbowAngle);
        telemetry.addData("elbow power correction: ", correction);
        robot.elbowServo.setPower(correction);
    }
}