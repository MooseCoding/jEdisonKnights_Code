package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import org.firstinspires.ftc.teamcode.vision.BlueScanner;
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
import org.firstinspires.ftc.teamcode.vision.RedScanner;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.teamcode.vision.Barcode;
import org.firstinspires.ftc.teamcode.vision.BlueScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Collection;
import java.util.Iterator;
import java.util.Queue;

@Autonomous
public class blueBottom extends LinearOpMode {
    private int prop = 1;
    private double propTheta = 0;

    private void spitPixel() {
        in.setPower(-0.05);
        double t = getRuntime();
        while (t + 0.2 > getRuntime()){}
    }

    private void dropPixel() {
        pA.setTargetPosition(20);
        pA.setPower(0.5);
        pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        am.setTargetPosition(-19);
        am.setPower(0.2);
        am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (pA.getCurrentPosition() >= 20 && am.getCurrentPosition() >= -19) {}
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

        pA.setTargetPosition(2449);
        pA.setPower(0.6);
        pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        am.setTargetPosition(185);
        am.setPower(0.2);
        am.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (pA.getCurrentPosition() >= 2447 && am.getCurrentPosition() >= 182) {}

        r1.setPosition(0.13);
        r2.setPosition(0.126);
        am.setTargetPosition(0);
        am.setPower(0.7);
        am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pA.setTargetPosition(0);
        pA.setPower(0.7);
        pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    private static final String TFOD_MODEL_ASSET = "blueModel.tflite";
    private TfodProcessor tfod;

    private static final String[] LABELS = {"teamProp"};

    private VisionPortal visionPortal;

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default sett ings are available to un-comment and edit as needed to
                // set parameters for custom models.
                //.setModelLabels(new String[] {"Blue team prop"})
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "cam"));

        builder.addProcessor(tfod);

        visionPortal = builder.build();


        tfod.setMinResultConfidence(0.8f);



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

        waitForStart();

        /*
        while(r == null || r.getConfidence() < 0.8 && getRuntime() < 1.5) {
            r = tfod.getRecognitions().get(0);
        }
        if (r == null) {
            prop = 0;
        }
        else {
            propTheta = r.estimateAngleToObject(AngleUnit.DEGREES);

            if (propTheta <= 20 && propTheta >= -20)
                prop = 1;
            else
                prop = 2;
        }*/
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
        Barcode result = Barcode.MIDDLE;
        double cTime = getRuntime();
        while (getRuntime() < cTime + 5) {
            result = scanner.getResult();
        }
        if (result.equals(null)) {
            result = Barcode.LEFT;
        }
        result = Barcode.MIDDLE;
        switch (result) {
            case RIGHT:
                telemetry.addData("Detected", result);
                d.followTrajectorySequence(
                    d.trajectorySequenceBuilder(new Pose2d(-35, 72, -Math.PI/2))
                            .splineTo(new Vector2d(-45, 34), -Math.PI/2)
                            .build());
                spitPixel();
                d.followTrajectorySequence(d.trajectorySequenceBuilder(new Pose2d(-45, 34, -Math.PI/2))
                            .back(2)
                            .turn(Math.PI/2)
                            .splineTo(new Vector2d(-16, 35), Math.toRadians(0))
                            .splineTo(new Vector2d(65-23, 35), Math.PI)
                                .build());
                dropPixel();
                d.followTrajectorySequence(d.trajectorySequenceBuilder(new Pose2d(65-23, 35, Math.PI))
                            .splineTo(new Vector2d(65-30, 35), Math.PI)
                            .splineTo(new Vector2d(56, 10), Math.PI/2)
                            .build());
                break;
           case MIDDLE:
               telemetry.addData("Detected", result);
               d.followTrajectorySequence(
               d.trajectorySequenceBuilder(new Pose2d(-35, 72, 0))
                       .splineTo(new Vector2d(-35, 34), Math.PI/2)
                       .build());
               spitPixel();
               d.followTrajectorySequence(d.trajectorySequenceBuilder(new Pose2d(-35, 34, Math.PI/2))
                       .splineTo(new Vector2d(-16, 35), Math.PI/2)
                       .splineTo(new Vector2d(65-23, 35), 3*Math.PI/2)
                               .build());
               dropPixel();
               d.followTrajectorySequence(d.trajectorySequenceBuilder(new Pose2d(65-23, 35, Math.PI))
                       .splineTo(new Vector2d(65-30, 35), Math.PI)
                       .splineTo(new Vector2d(56, 10), Math.PI/2)
                       .build());
                break;

            case LEFT:
                telemetry.addData("Detected", result);
                d.followTrajectorySequence(
                        d.trajectorySequenceBuilder(new Pose2d(-35, 72, -Math.PI/2))
                                .splineTo(new Vector2d(-25, 34), Math.PI/2)
                                .build());
                spitPixel();
                d.followTrajectorySequence(d.trajectorySequenceBuilder(new Pose2d(-35, 34, Math.PI/2))
                                .turn(Math.PI/2)
                        .splineTo(new Vector2d(-16, 35), Math.toRadians(0))
                        .splineTo(new Vector2d(65-23, 35), Math.PI)
                        .build());
                dropPixel();
                d.followTrajectorySequence(d.trajectorySequenceBuilder(new Pose2d(65-23, 35, Math.PI))
                        .splineTo(new Vector2d(65-30, 35), Math.PI)
                        .splineTo(new Vector2d(56, 10), Math.PI/2)
                        .build());

                break;
        }
    }

}
