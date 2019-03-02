package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRoverRuckus;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus;

import java.util.List;

@Autonomous(name = "Test Vision", group = "")
public class TestVision extends LinearOpMode {
    public VuforiaRoverRuckus vuforiaRoverRuckus;
    public TfodRoverRuckus tfodRoverRuckus;

    @Override
    public void runOpMode() {
        List<Recognition> recognitions;
        vuforiaRoverRuckus = new VuforiaRoverRuckus();
        tfodRoverRuckus = new TfodRoverRuckus();
        vuforiaRoverRuckus.initialize("", VuforiaLocalizer.CameraDirection.BACK,
                true, false, VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES,
                0, 0, 0, 0, 0, 0, true);
        tfodRoverRuckus.initialize(vuforiaRoverRuckus, (float) 0.4, true, true);
        tfodRoverRuckus.activate();

        telemetry.addData("Started:", "now");
        telemetry.update();

        while (opModeIsActive()) {
            recognitions = tfodRoverRuckus.getRecognitions();
            telemetry.addData("# Objects Recognized", recognitions.size());
            for (Recognition recognition : recognitions) {
                telemetry.addData("Recognized", recognition.toString());
            }
            telemetry.update();
        }
        tfodRoverRuckus.deactivate();
        vuforiaRoverRuckus.close();
        tfodRoverRuckus.close();
    }
}