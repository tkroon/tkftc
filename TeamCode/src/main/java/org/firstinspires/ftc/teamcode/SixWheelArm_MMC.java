package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp

public class SixWheelArm_MMC extends LinearOpMode {

    private DcMotor leftMotor;
    private DigitalChannel digitalTouch;
    private DigitalChannel digitalTouch2;
    private DistanceSensor sensorColorRange = null;
    //private CRServo elbowServo;
    private CRServo elbowServo;
    private Servo clawServo;
    private Servo wristServo;
    private DcMotor rightMotor;
    private DcMotor lifter;
    private ColorSensor sensorColor;
    private DcMotor shoulderServo;
    private CRServo clawServo2;
    private AnalogInput AngleSensor;
    //tester
    private Servo TEST;

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowServo = hardwareMap.get(CRServo.class, "elbowservo");
        wristServo = hardwareMap.get(Servo.class, "wristservo");
        //elbowServo = hardwareMap.get(Servo.class, "elbowservo");
        clawServo = hardwareMap.get(Servo.class, "clawservo");
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        //sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        shoulderServo = hardwareMap.get(DcMotor.class, "sholderServo");
        clawServo2 = hardwareMap.get(CRServo.class, "clawServo2");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        digitalTouch2 = hardwareMap.get(DigitalChannel.class, "digitalTouch2");
        digitalTouch2.setMode(DigitalChannel.Mode.INPUT);
        AngleSensor = hardwareMap.get(AnalogInput.class, "AngleSensor");
        shoulderServo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //test
        TEST = hardwareMap.get(Servo.class, "Test");

        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");

        waitForStart();
        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        double tgtPower2 = 0;
        double NewIdea = 0;
        double Angle = 0;
        int flipPID = 0;
        int movePIDL = 0;
        int stepLock = 0;
        int step = 0;
        double AngleSnap = 0;
        double Intergral = 0;
        //double tgtPower3 = 0;
        //double shoulder = 1;
        //double ex = 0;

        while (opModeIsActive()) {

            Angle = (AngleSensor.getVoltage()) * 81;

            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Left Motor Power", leftMotor.getPower());
            //telemetry.addData("Status", "Running");
            telemetry.addData("Target Power2", tgtPower2);
            telemetry.addData("Right Motor Power", rightMotor.getPower());
            telemetry.addData("Status", "Running");
            telemetry.addData("Shoulder Servo port 1", shoulderServo.getPower());
            telemetry.addData("Elbow servo port 2", elbowServo.getPower());
            telemetry.addData("Claw servo port 4", clawServo.getPosition());
            telemetry.addData("wrist servo port ", wristServo.getPosition());
            telemetry.addData("volts", AngleSensor.getVoltage());
            telemetry.addData("Angle", Angle);
            telemetry.addData("Encodeer", shoulderServo.getCurrentPosition());
            //telemetry.addData("Distance (cm)", sensorColorRange.getDistance(DistanceUnit.CM));
            //telemetry.addData("Alpha", sensorColor.alpha());
            //telemetry.addData("Red  ", sensorColor.red());
            //telemetry.addData("Green", sensorColor.green());
            //telemetry.addData("Blue ", sensorColor.blue());


            telemetry.update();

            if (!digitalTouch.getState()) {
                telemetry.addData("Button", "PRESSED");

            } else {
                telemetry.addData("Button", "NOT PRESSED");
            }

            // controler 1
            // Raise arm at robot base "shoulder"
            if (gamepad1.a) {
                shoulderServo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (!digitalTouch2.getState()) {
                    shoulderServo.setPower(0);

                } else {
                    shoulderServo.setPower(.5);

                }
            }

            if (gamepad1.b) { // if b is pressed
                shoulderServo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (!digitalTouch.getState()) {
                    shoulderServo.setPower(0);

                } else { //if not pressed
                    shoulderServo.setPower(-.5);

                }
            }
            //if both not pressed
            if (!gamepad1.a && !gamepad1.b) {
                if (gamepad2.dpad_down) {

                } else {
                    shoulderServo.setPower(0);
                }
            }

            // Raise arm at arm joint elbow
            if (gamepad1.x) {
                movePIDL = 0;
                flipPID = 1;
                //ex = 0;

                if (Angle > 250) {
                    elbowServo.setPower(0);

                } else {
                    elbowServo.setPower(-.5);

                }
            }

            if (gamepad1.y) {
                movePIDL = 0;
                flipPID = 1;
                //ex = 0;
                if (Angle < 20) {
                    elbowServo.setPower(0);

                } else {
                    elbowServo.setPower(.5);

                }
            }
            // STAY STILL
            if (!gamepad1.x && !gamepad1.y) {

                if (flipPID == 3) {

                    if ((AngleSnap) > Angle) {
                        Intergral = Angle - AngleSnap;
                        elbowServo.setPower(-.05 + (Intergral / 11));

                    } else if (AngleSnap < Angle) {
                        elbowServo.setPower(.05);

                    }

                } else {
                    if (movePIDL == 0) {
                        AngleSnap = Angle;

                    }
                    flipPID = 3;

                }
            }
            // Open and close claw
            if (gamepad1.right_trigger == 1) {
                clawServo.setPosition(clawServo.getPosition() + 0.1);

            } else if (gamepad1.left_trigger == 1) {
                clawServo.setPosition(clawServo.getPosition() - 0.1);

            } else if (gamepad1.left_trigger == 0 & gamepad1.right_trigger == 0) {
                clawServo.setPosition(0);

            }

            // Open and close claw2
            if (gamepad1.right_trigger == 1) {
                clawServo2.setPower(.8);

            } else if (gamepad1.left_trigger == 1) {
                clawServo2.setPower(-.8);

            } else if (gamepad1.left_trigger == 0 & gamepad1.right_trigger == 0) {
                clawServo2.setPower(0);

            }

            // dunk the wrist bro
            if (gamepad1.dpad_left) {
                wristServo.setPosition(wristServo.getPosition() + 0.015);

            } else if (gamepad1.dpad_right) {
                wristServo.setPosition(wristServo.getPosition() - 0.015);

            }

            // TESTERS
            if (gamepad1.left_bumper) {
                TEST.setPosition(TEST.getPosition() + 0.015);

            } else if (gamepad1.right_bumper) {
                TEST.setPosition(TEST.getPosition() - 0.015);

            }

            // controler 2
            //driving
            if (gamepad2.a) {
                NewIdea = 1;

            }
            if (gamepad2.b) {
                NewIdea = .5;

            }
            if (gamepad2.right_bumper){
                NewIdea = Math.abs (NewIdea )* -1;

            }

            if (gamepad2.left_bumper){
                NewIdea = Math.abs (NewIdea);

            }
            // driving 2
            tgtPower = this.gamepad2.left_stick_y;
            leftMotor.setPower(tgtPower * NewIdea);
            tgtPower2 = -this.gamepad2.right_stick_y;
            rightMotor.setPower(tgtPower2 * NewIdea);

            // left/right
            if ((Math.abs(gamepad2.right_stick_y) > 0.5) || (gamepad2.left_stick_y > -0.5)) {
                stepLock = 1;
                step = -1;
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
            // not moving
            else {
                stepLock = 0;

            }

            // lift up and down
            if (gamepad2.right_trigger == 1) {
                lifter.setPower(1);

            }

            if (gamepad2.left_trigger == 1) {
                lifter.setPower(-1);

            }

            if (gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0) {
                lifter.setPower(0);

            }

            // get position
            if (gamepad2.dpad_up) {
                wristServo.setPosition(0.58);
                shoulderServo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulderServo.setTargetPosition(300);
                shoulderServo.setPower(1);
                flipPID = 3;
                movePIDL = 1;
                AngleSnap = 90;

            }


            //dump position
            if (gamepad2.dpad_down) {
                wristServo.setPosition(0.47);
                shoulderServo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulderServo.setTargetPosition(-2500);
                shoulderServo.setPower(1);

                flipPID = 3;
                movePIDL = 1;
                AngleSnap = 90;

            }

            //fold
            if (gamepad2.dpad_left) {
                wristServo.setPosition(0.);
                shoulderServo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulderServo.setTargetPosition(-100);
                shoulderServo.setPower(1);

                flipPID = 3;
                movePIDL = 1;
                AngleSnap = 300;

            }


            // move right and or left like omni
            if (gamepad2.right_stick_x > .5) {
                if (stepLock == 0) {

                    if (step == -1) {
                        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        step = 0;
                    }
                    if (step == 0) {
                        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftMotor.setTargetPosition(600);
                        rightMotor.setTargetPosition(600);
                        rightMotor.setPower(.5);
                        leftMotor.setPower(.5);
                        if ((leftMotor.getCurrentPosition() < 605) && (leftMotor.getCurrentPosition() > 595) && (rightMotor.getCurrentPosition() < 605) && (rightMotor.getCurrentPosition() > 595)) {
                            step = 1;
                            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        }

                    } else if (step == 1) {
                        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftMotor.setTargetPosition(-100);
                        rightMotor.setTargetPosition(100);
                        rightMotor.setPower(.5);
                        leftMotor.setPower(.5);
                        if ((leftMotor.getCurrentPosition() < -95) && (leftMotor.getCurrentPosition() > -105) && (rightMotor.getCurrentPosition() < 105) && (rightMotor.getCurrentPosition() > 95)) {
                            step = 2;
                            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        }

                    } else if (step == 2) {
                        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftMotor.setTargetPosition(-300);
                        rightMotor.setTargetPosition(-300);
                        rightMotor.setPower(.5);
                        leftMotor.setPower(.5);
                        if ((leftMotor.getCurrentPosition() < -295) && (leftMotor.getCurrentPosition() > -305) && (rightMotor.getCurrentPosition() < -295) && (rightMotor.getCurrentPosition() > -305)) {
                            step = 3;
                            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        }

                    } else if (step == 3) {
                        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftMotor.setTargetPosition(100);
                        rightMotor.setTargetPosition(-100);
                        rightMotor.setPower(.5);
                        leftMotor.setPower(.5);
                        if ((leftMotor.getCurrentPosition() < 105) && (leftMotor.getCurrentPosition() > 95) && (rightMotor.getCurrentPosition() < -95) && (rightMotor.getCurrentPosition() > -105)) {
                            step = 4;
                            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        }

                    } else if (step == 4) {
                        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftMotor.setTargetPosition(-300);
                        rightMotor.setTargetPosition(-300);
                        rightMotor.setPower(.5);
                        leftMotor.setPower(.5);
                        if ((leftMotor.getCurrentPosition() < -295) && (leftMotor.getCurrentPosition() > -305) && (rightMotor.getCurrentPosition() < -295) && (rightMotor.getCurrentPosition() > -305)) {
                            step = 5;
                            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        }

                    }
                }
            }
        }
    }
}
//if(sensorValue[Bumperswitch] == 1)
//motor[frontleft] = 0;
// else
//if(sensorValue[Bumperswitch] == 0);
// motor[frontleft] = 127;


// if (Bumperswitch)
//{
// frontleft = 0;
//}
// else
//{
// frontleft = 127;
//}