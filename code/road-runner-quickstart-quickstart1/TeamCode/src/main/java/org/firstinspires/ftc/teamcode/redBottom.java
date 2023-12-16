package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.vision.Barcode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.teamcode.vision.RedScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Collection;
import java.util.Iterator;
import java.util.Queue;

@Autonomous
public class redBottom extends LinearOpMode {
    private int hPA = 250;
    private int s1PA = 0;
    private int s2PA = 403;
    private int s3PA = 2650;
    private int hAM = 80;

    private int s1AM = -64;
    private int s2AM = -70;
    private int s3AM =152;
    private int prop = 1;


    private void spitPixel() {
        in.setPower(-0.5);
        double t = getRuntime();
        while (t + 2 > getRuntime()){
        }
    }

    private void dropPixel() {/*
        pA.setTargetPosition(20);
        pA.setPower(0.5);
        pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        am.setTargetPosition(-19);
        am.setPower(0.2);
        am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (pA.getCurrentPosition() >= 19 && am.getCurrentPosition() >= -18) {}
            double t = getRuntime();
            while (getRuntime() < 0.2 + t) {}
            r1.setPosition(0.04);
            r2.setPosition(0.05);
            t = getRuntime();
            while(getRuntime() < 0.3 + t) {}
            pA.setTargetPosition(403);
            pA.setPower(0.4);
            pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            am.setTargetPosition(-60);
            am.setPower(0.2);
            am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (pA.getCurrentPosition() >= 400 && am.getCurrentPosition() <= -57) {}

            pA.setTargetPosition(2650);
            pA.setPower(0.6);
            pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            am.setTargetPosition(152);
            am.setPower(0.2);
            am.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (pA.getCurrentPosition() >= 2646 && am.getCurrentPosition() >= 150) {}

            r1.setPosition(0.13);
            r2.setPosition(0.126);
            am.setTargetPosition(0);
            am.setPower(0.7);
            am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pA.setTargetPosition(0);
            pA.setPower(0.7);
            pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            */
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
        }
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

        pA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        am.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pA.setDirection(DcMotor.Direction.FORWARD);
        r2.setDirection(Servo.Direction.REVERSE);


        SampleMecanumDrive d = new SampleMecanumDrive(hardwareMap);

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
        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        r1.setPosition(0.04);
        r2.setPosition(0.05);

            Barcode result = scanner.getResult();
            result = Barcode.MIDDLE; 

            switch (result) {
                case RIGHT:
                    telemetry.addData("Dectected", result);
                    d.followTrajectorySequence( d.trajectorySequenceBuilder(new Pose2d(-36, -72, Math.PI/2))
                            .splineTo(new Vector2d(-28, -30), Math.PI/2)
                                    .build());
                    spitPixel();
                    d.followTrajectorySequence( d.trajectorySequenceBuilder(new Pose2d(-28, -30, Math.PI/2))
                            .back(4)
                            .turn(-Math.PI/2)
                            .splineTo(new Vector2d(-16, -34), 0)
                            .splineTo(new Vector2d(65-20, -34), Math.PI)

                            .splineTo(new Vector2d(65-24, -34), Math.PI)
                            .splineTo(new Vector2d(56, -10), Math.PI)
                            .build());

                    break;
                case MIDDLE:
                    telemetry.addData("Dectected", result);
                    d.setPoseEstimate(new Pose2d(-34, -72, Math.PI/2));
                    d.followTrajectorySequence(
                            d.trajectorySequenceBuilder(new Pose2d(-34, -72, Math.PI/2))
                                    .splineTo(new Vector2d(-34, -32), Math.PI/2)
                                    .build());
                    spitPixel();
                    d.followTrajectorySequence(d.trajectorySequenceBuilder(new Pose2d(-34, -32, Math.PI/2))
                                    .splineTo(new Vector2d(-34, -32-15), Math.PI/2)
                                    .splineTo(new Vector2d(-34, -32-15+7), Math.PI/2)
                                    .splineTo(new Vector2d(-16, -34), 0)
                                    .build());
                    dropPixel();
                    d.followTrajectorySequence(d.trajectorySequenceBuilder(new Pose2d(-16, -34, 0))
                                    .splineTo(new Vector2d(65-20, -34), Math.PI)
                                    .splineTo(new Vector2d(65-24, -34), Math.PI)
                                    .splineTo(new Vector2d(56, -10), Math.PI)
                                    .build());
                    break;
                case LEFT: // red left
                    telemetry.addData("Dectected", result);
                    d.followTrajectorySequence(d.trajectorySequenceBuilder(new Pose2d(-36, -72, Math.PI/2))
                            .splineTo(new Vector2d(-48, -33), Math.PI/2)
                                    .build());

                    spitPixel();
                    d.followTrajectorySequence(d.trajectorySequenceBuilder(new Pose2d(-48, -33, Math.PI/2))
                            .back(6)
                            .turn(-Math.PI/2)
                            .splineTo(new Vector2d(-16, -34), 0)
                            .splineTo(new Vector2d(65-20, -34), Math.PI)
                                    .build());
                    dropPixel();
                    d.followTrajectorySequence(d.trajectorySequenceBuilder(new Pose2d(65-20, -34, Math.PI))
                            .forward(4)
                            .splineTo(new Vector2d(56, -10), Math.PI)
                            .build());
                    break;
            }

    }

}
