package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.vision.BlueScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Collection;
import java.util.Iterator;
import java.util.Queue;
@Disabled
@Autonomous
public class blueTop extends LinearOpMode {
    private void spitPixel(double p) {
        in.setPower(p);
        double t = getRuntime();
        while (t + 2 > getRuntime()){
        }
        in.setPower(0);
    }

    private void dropPixel(SampleMecanumDrive d) {
        pA.setTargetPosition(577);
        pA.setPower(0.5);
        pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        am.setTargetPosition(-87);
        am.setPower(0.2);
        am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (pA.getCurrentPosition() <= 575 && am.getCurrentPosition() >= -85) {}


        pA.setTargetPosition(2917);
        pA.setPower(0.6);
        pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        am.setTargetPosition(152);
        am.setPower(0.2);
        am.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (pA.getCurrentPosition() <= 2916 && am.getCurrentPosition() <= 151) {}

        r1.setPosition(0.13);
        r2.setPosition(0.126);
        double t = getRuntime();
        while (t + 2 > getRuntime());

        am.setTargetPosition(0);
        am.setPower(0.7);
        am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pA.setTargetPosition(0);
        pA.setPower(0.7);
        pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        in.setDirection(DcMotor.Direction.REVERSE);


        SampleMecanumDrive d = new SampleMecanumDrive(hardwareMap);
        d.setPoseEstimate(new Pose2d(36, -72+18, Math.PI/2));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cam"), cameraMonitorViewId);
        BlueScanner scanner = new BlueScanner(telemetry);
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

        Barcode result = Barcode.MIDDLE;;
        double cTime = getRuntime();
        while (getRuntime() < cTime + 2) {
            result = scanner.getResult(3);
            telemetry.addData("result", result);
            telemetry.update();
        }

            if (result != null) {
                switch (result) {
                    case RIGHT:
                        telemetry.addData("Dectected", result);
                        d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                                .back(28)
                                .turn(-Math.PI / 2)
                                .build());
                        spitPixel(0.35);
                        d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                                .strafeLeft(13)
                                .back(18)
                                .strafeRight(9.4)
                                .build());
                        dropPixel(d);
                        d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                                .strafeLeft(11)
                                .back(9)
                                .build());
                        break;
                    case MIDDLE:
                        d.setPoseEstimate(d.getPoseEstimate());
                        d.followTrajectorySequence(
                                d.trajectorySequenceBuilder(d.getPoseEstimate())
                                        .back(50)
                                        .build());
                        spitPixel(0.35);
                        d.updatePoseEstimate();
                        d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                                .back(1)
                                .turn(Math.PI / 2)
                                .back(17.5)
                                .strafeRight(14)
                                .build());
                        dropPixel(d);
                        d.updatePoseEstimate();
                        telemetry.addData("current pos", d.getPoseEstimate());
                        d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                                .strafeLeft(14)
                                .back(10)
                                .build());
                        break;

                    case LEFT:
                        telemetry.addData("Dectected", result);
                        d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                                .forward(28)
                                .turn(-Math.PI / 2)
                                .build());

                        spitPixel(0.35);
                        d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                                .back(1)
                                .strafeRight(13)
                                .turn(Math.PI)
                                .back(18)
                                .strafeRight(18)
                                .build());
                        dropPixel(d);
                        d.followTrajectorySequence(d.trajectorySequenceBuilder(d.getPoseEstimate())
                                .strafeLeft(18)
                                .back(9)
                                .build());
                        break;

                }
            }

    }

}
