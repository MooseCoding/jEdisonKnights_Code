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
                            .back(2)
                    .build());

            am.setTargetPosition(0);
            am.setPower(0.7);
            am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pA.setTargetPosition(0);
            pA.setPower(0.7);
            pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*
        int arm = -1;
        boolean armReset = true;
        boolean leftArm = false;
        switch (arm) {
            case -1:
                if (armReset) {
                    r1.setPosition(0.126);
                    r2.setPosition(0.13);
                    pA.setTargetPosition(hPA);
                    pA.setPower(0.6);
                    pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    am.setTargetPosition(hAM);
                    am.setPower(0.1);
                    am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armReset = false;
                }
                break;
            case 0:
                if (!armReset) {
                    am.setTargetPosition(-66);
                    am.setPower(0.5);
                    am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armReset = true;
                }
                if (am.getCurrentPosition() <= -60) {
                    pA.setTargetPosition(60);
                    pA.setPower(0.5);
                    pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (pA.getCurrentPosition() <= 90) {
                    arm++;
                    armReset = false;
                }
                break;
            case 1:
                if (!armReset) {
                    double t = getRuntime();
                    while (getRuntime() < 0.5 + t) {
                    }
                    r1.setPosition(0.04);
                    r2.setPosition(0.05);
                    t = getRuntime();
                    while (getRuntime() < 0.2 + t) {
                    }
                    pA.setTargetPosition(403);
                    pA.setPower(0.4);
                    pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    am.setTargetPosition(s2AM);
                    am.setPower(0.6);
                    am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armReset = true;
                }
                if (pA.getCurrentPosition() >= 400 && am.getCurrentPosition() <= s2PA + 4) {
                    arm++;
                }
                break;
            case 2:
                if (armReset) {

                    pA.setTargetPosition(s3PA);
                    pA.setPower(0.6);
                    pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    am.setTargetPosition(s3AM);
                    am.setPower(0.2);
                    am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (pA.getCurrentPosition() >= s3PA - 1 && am.getCurrentPosition() >= s3AM - 1) {
                    arm++;
                }
                break;
            case 3:

                    r1.setPosition(0.13);


                    r2.setPosition(0.126);

                double t =getRuntime();
                while (t + 3 > getRuntime()) {}
                break;
        }*/
    }




    private DcMotor pA, in, am;

    private Servo r1, r2;
    private Recognition r;

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
        telemetry.setMsTransmissionInterval(5);

        waitForStart();

        r1.setPosition(0.04);
        r2.setPosition(0.05);

        Barcode result = null;

        result = scanner.getResult(3);

        if (result == null)
            result = Barcode.LEFT;



        if (result != null) {
            switch (result) {
                case RIGHT:
                    telemetry.addData("Dectected", result);

                    d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                            .strafeLeft(1)
                            .forward(31.5)
                            .turn(-Math.PI/2)
                            .build());

                    spitPixel(0.4);
                    d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                            .back(5)
                            .turn(Math.PI/2)
                            .strafeLeft(2)
                            .forward(24)
                            .turn(Math.PI/2)
                            .back(90)
                            .strafeLeft(18.5)
                            .build());

                    dropPixel(d);
                    d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                            .strafeRight(17)
                            .back(18)
                            .build());
                case MIDDLE:
                    telemetry.addData("Dectected", result);
                    d.setPoseEstimate(d.getPoseEstimate());
                    d.followTrajectorySequence(
                            d.trajectorySequenceBuilder(d.getPoseEstimate())
                                    .forward(50)
                                    .turn(Math.PI)
                                    .build());
                    spitPixel(0.4);
                    d.updatePoseEstimate();
                    d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                            .back(3)
                            .turn(-Math.PI / 2)
                                    .strafeRight(1)
                            .back(78)
                            .strafeLeft(12)
                            .build());
                    dropPixel(d);
                    d.updatePoseEstimate();
                    telemetry.addData("current pos", d.getPoseEstimate());
                    d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                            .strafeRight(14)
                            .back(8)
                            .build());
                    break;
                case LEFT: // red left
                    telemetry.addData("Dectected", result);

                    d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                                    .forward(28)
                                    .turn(Math.PI/2)
                            .build());

                    spitPixel(0.37);
                    d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                                    .strafeRight(14)
                            .back(78.5)
                            .strafeLeft(9.4)
                            .build());
                    dropPixel(d);
                    d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                            .strafeRight(13)
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
