package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.vision.Barcode;
import org.firstinspires.ftc.teamcode.vision.RedScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class redBottom extends LinearOpMode {
    private void spitPixel(double p) {
        in.setPower(p);
        double t = getRuntime();
        while (t + 2 > getRuntime()){
        }
        in.setPower(0);
    }

    private void dropPixel(SampleMecanumDrive d) {
            pA.setTargetPosition(2840);
            pA.setPower(0.6);
            pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            am.setTargetPosition(-120);
            am.setPower(0.6);
            am.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(am.getCurrentPosition() > -110);
            am.setTargetPosition(152);
            am.setPower(0.2);
            am.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (pA.getCurrentPosition() <= 2916 && am.getCurrentPosition() <= 151) {}
            double t = getRuntime();
            while (t + 1 > getRuntime());
            r1.setPosition(0.13);
            r2.setPosition(0.126);
             t = getRuntime();
            while (t + 2 > getRuntime());

            d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                            .forward(2)
                    .build());

            am.setTargetPosition(0);
            am.setPower(0.7);
            am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pA.setTargetPosition(0);
            pA.setPower(0.7);
            pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private DcMotor pA, in, am;

    private Servo r1, r2;

    @Override
    public void runOpMode() {
        in = hardwareMap.get(DcMotor.class, "intake");
        pA = hardwareMap.dcMotor.get("pA");
        am = hardwareMap.get(DcMotor.class, "am");
        r1 = hardwareMap.get(Servo.class, "r1");
        r2 = hardwareMap.get(Servo.class, "r2");

        pA.setDirection(DcMotor.Direction.FORWARD);
        r2.setDirection(Servo.Direction.REVERSE);
        in.setDirection(DcMotor.Direction.REVERSE);


        SampleMecanumDrive d = new SampleMecanumDrive(hardwareMap);
        d.setPoseEstimate(new Pose2d(-36, -72+18, Math.PI/2));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cam"), cameraMonitorViewId);
        RedScanner scanner = new RedScanner(telemetry);
        webcam.setPipeline(scanner);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        Barcode result = null;
        telemetry.setMsTransmissionInterval(5);
        while(!isStarted()) {
            result = scanner.getResult(0.01);
            telemetry.addData("result", result);
        }
        r1.setPosition(0.04);
        r2.setPosition(0.05);
        if (result != null) {
            switch (result) {
                case RIGHT:
                    telemetry.addData("Dectected", "R");

                    d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                            .strafeRight(1)
                            .back(29)
                            .turn(-Math.PI/2)
                                    .back(5)
                                    .forward(4.5)
                            .build());

                    d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                            .strafeRight(11.5)
                            .back(82)
                            .strafeLeft(17.7)
                            .build());

                    dropPixel(d);
                    d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                            .strafeRight(18.2)
                            .back(8)
                            .build());
                    break;
                case MIDDLE:
                    telemetry.addData("Dectected", "M");
                    d.setPoseEstimate(d.getPoseEstimate());
                    d.followTrajectorySequence(
                            d.trajectorySequenceBuilder(d.getPoseEstimate())
                                    .back(30)
                                    .forward(5)
                                    .strafeRight(10)
                                    .turn(-Math.PI/2)
                                    .strafeRight(15)
                                    .build());
                    d.updatePoseEstimate();
                    d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                            .back(81.9+19.2)
                            .strafeLeft(15)
                            .build());
                    dropPixel(d);
                    d.updatePoseEstimate();
                    telemetry.addData("current pos", d.getPoseEstimate());
                    d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                            .strafeRight(16)
                            .back(8)
                            .build());
                    break;
                case LEFT: // red left
                    telemetry.addData("Dectected", "L");
                    d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                                    .back(29)
                                    .turn(Math.PI/2)
                                    .back(5)
                                    .forward(5)
                                    .strafeLeft(15)
                                    .turn(Math.PI)
                            .build());
                    d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                            .back(79.8-1.5)
                            .strafeLeft(11)
                            .build());
                    dropPixel(d);
                    d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                            .strafeRight(15)
                            .back(7)
                            .build());
                    break;
            }
        }
        else {
            while(!isStopRequested()) {
                telemetry.addData("pos", d.getPoseEstimate());
                d.updatePoseEstimate();
            }
        }
    }

}