package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class mooseTeleopSafe extends LinearOpMode {

    int aprilTheta;
    int aprilDistance;
    //only temporary for code adjustment 
    private int hPA = 250;
    private int s1PA = 0;
    private int s2PA = 403;
    private int s3PA = 2650;
    private int hAM = 80;

    private int s1AM = -64;
    private int s2AM = -70;
    private int s3AM =152;
    


    IMU imu;//x-axis rotation = , y-axis rotation = , z-axis rotation

    private RevColorSensorV3 c1, c2;

    private DcMotor br;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor fl;

    private int pixeR = 0, pixeL = 0;

    private double cValue;

    private DcMotor in; //intake
    private boolean lockedArm, dpadUnlock = false;

    private DcMotor pA;

    private double speedMulti = 1.0; //multiplier for running motors at speed

    private double mult = 1;

    private DcMotor am;

    private Servo air;
    private double turnMult = 0.8;

    private boolean armReset = true;

    boolean leftArm = false, planeActive = true;

    //private Servo e1, e2;
   private Servo r1, r2;

   private Rev2mDistanceSensor d1;

   private int arm = -1;

   private int p1C, p2C; //pixel 1/2 colors (-1 no pixel , 0 white, 1 green, 2 yellow, 3 purple)

    private double iP = 0.6; //intake power

    //private int currentAprilTagID;

    private boolean camOn;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        /*
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "cam"))
                .setCameraResolution(new Size(640, 480))
                .build();

         */
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
                = RevHubOrientationOnRobot.LogoFacingDirection.values();
        RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
                = RevHubOrientationOnRobot.UsbFacingDirection.values();
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoFacingDirections[0], usbFacingDirections[5])));

        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        fl = hardwareMap.get(DcMotor.class, "fl");
        in = hardwareMap.get(DcMotor.class, "intake");
        pA = hardwareMap.dcMotor.get("pA");
        am = hardwareMap.get(DcMotor.class, "am");
        r1 = hardwareMap.get(Servo.class, "r1");
        r2 = hardwareMap.get(Servo.class, "r2");
        air = hardwareMap.get(Servo.class, "air");
        c1 = hardwareMap.get(RevColorSensorV3.class, "c1");
        d1 = hardwareMap.get(Rev2mDistanceSensor.class, "d1");
        c2 = hardwareMap.get(RevColorSensorV3.class, "c2");

        pA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        am.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        pA.setDirection(DcMotor.Direction.FORWARD);
        r2.setDirection(Servo.Direction.REVERSE);


        camOn = true;
        lockedArm = false;

        double time = 0;
        resetRuntime();

        waitForStart();
        while (opModeIsActive()) {
            boolean buttonPress = getRuntime() >= 0.2 + time;
            if (gamepad1.right_bumper && buttonPress) {
                if (mult == 1) {
                    mult = 0.4;
                    turnMult = 0.3;
                    gamepad1.rumble(1, 0, 100);
                } else {
                    mult = 1;
                    turnMult = 0.8;
                    gamepad1.rumble(0, 1, 100);
                }
            }
            if (gamepad1.dpad_right && buttonPress && dpadUnlock) {
                air.setPosition(0.2015);
                planeActive = false;
            }
            if (gamepad1.dpad_left && buttonPress && dpadUnlock) {
                air.setPosition(0.3);
            }
            if (gamepad1.dpad_up && buttonPress && dpadUnlock) {
                arm = -2;
                pA.setTargetPosition(2027);
                pA.setPower(0.3);
                pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                am.setTargetPosition(-96);
                am.setPower(0.3);
                am.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            if (gamepad1.dpad_down && buttonPress && dpadUnlock) {
                pA.setTargetPosition(800);
                pA.setPower(1);
                pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad1.triangle && buttonPress) {
                lockedArm = !lockedArm;
            }
            if (lockedArm && d1.getDistance(DistanceUnit.CM) <= 13) {
                mult=0.4;
            }
            if (lockedArm && d1.getDistance(DistanceUnit.CM) > 13) {
                mult=1;
            }

            //ignore every button input within a 0.2 sec timespan as its neglible to affect results
            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double leftFrontPower = (axial + lateral) + yaw;
            double rightFrontPower = (axial - lateral) - yaw;
            double leftBackPower = (axial - lateral) + yaw;
            double rightBackPower = (axial + lateral) - yaw;


            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            fl.setPower(leftFrontPower * mult);
            bl.setPower(leftBackPower * mult);
            fr.setPower(rightFrontPower * mult);
            br.setPower(rightBackPower * mult);



            /*
            if (gamepad1.left_bumper) {
                if (camOn) {
                    visionPortal.stopStreaming();
                    visionPortal.stopLiveView();
                } else {
                    visionPortal.resumeStreaming();
                    visionPortal.resumeLiveView();
                }
            }
            */

            if (gamepad1.right_trigger > 0 && !lockedArm) {
                in.setPower(iP);
            } else if (gamepad1.left_trigger > 0 && !lockedArm) {
                in.setPower(-iP);
            } else {
                in.setPower(0);
            }


            
            if (gamepad1.right_trigger > 0 && lockedArm && arm == -1) {
                arm = 0;
            }
            
   

            if (gamepad1.left_trigger > 0 && lockedArm && arm == -1) {
                leftArm = true;
                arm=0;
                pixeL = 1;
                pixeR = 1;
            }

            //if (gamepad1.circle) {
            //look for april tags on the left
            //currentAprilTagID = tag.id();
            //telemetry.addData("current tag left", currentAprilTagID);
            //}

            //if (gamepad1.square) {
            //look for april tags on the right
            //while (
            //currentAprilTagID = tag.id();
            //telemetry.addData("current tag right", currentAprilTagID);
            //}

            //if (gamepad1.cross) {
            //go to selected april tag that is on the telemetry output
            //aprilTheta = (int) Math.atan(tag.ftcPose.y / tag.ftcPose.x);
            //aprilDistance = (int) Math.sqrt((tag.ftcPose.z * tag.ftcPose.z) + (tag.ftcPose.x * tag.ftcPose.x));
            //}


            switch(arm) {
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
                    if (am.getCurrentPosition() <= -60 ) {
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
                        while (getRuntime() < 0.5 + t) {}
                        r1.setPosition(0.04);
                        r2.setPosition(0.05);
                        t = getRuntime();
                        while(getRuntime() < 0.2 + t) {}
                        pA.setTargetPosition(403);
                        pA.setPower(0.4);
                        pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        am.setTargetPosition(s2AM);
                        am.setPower(0.6);
                        am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armReset = true;
                    }
                    if (pA.getCurrentPosition() >= 400 && am.getCurrentPosition() <= s2PA+4) {
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
                    if (pA.getCurrentPosition() >= s3PA-1 && am.getCurrentPosition() >= s3AM-1 ) {
                        arm++;
                    }
                    break;
                case 3:
                        if (leftArm && gamepad1.circle && buttonPress) {
                            r1.setPosition(0.13);

                        }
                        if (leftArm && gamepad1.square && buttonPress) {
                            r2.setPosition(0.126);
                            leftArm = !leftArm;
                        }

                    if (!leftArm) {
                        leftArm=false;
                        arm = -1;
                    }
                    break;
            }

                dpadUnlock = true;



            telemetry.addData("multi", mult);
            telemetry.addData("locked arm", lockedArm);
            telemetry.addData("dpad unlocked", dpadUnlock);
            telemetry.addData("arm location", arm);
            telemetry.addData("arm pos", pA.getCurrentPosition())
;            telemetry.addData("distance (cm)", d1.getDistance(DistanceUnit.CM));
            telemetry.addData("right trigger", gamepad1.right_trigger);
            telemetry.addData("right trigger speed", pA.getPower());
            telemetry.addData("pixel count", pixeR+pixeL);
            telemetry.addData("pixel in the right", p1C);
            telemetry.addData("pixel in the left", p2C);
            telemetry.addData("pA pos", pA.getCurrentPosition());
            telemetry.addData("Time", getRuntime());
            telemetry.update();
            if (buttonPress) {
                time = getRuntime();
            }
        }

    }
}