package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class mooseTeleopSafeTopher extends LinearOpMode {

    String color1;
    String color2;

    private int armPos;

    boolean rightReady = false;
    boolean leftReady = false;

    IMU imu;//x-axis rotation = , y-axis rotation = , z-axis rotation

    private RevColorSensorV3 c1, c2;

    private DcMotor br;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor fl;

    private int pixeR = 0, pixeL = 0;
    private int p1, p2;

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
        VisionPortal visionPortal = new VisionPortal.Bu10ilder()
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


        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        pA.setDirection(DcMotor.Direction.FORWARD);
        r2.setDirection(Servo.Direction.REVERSE);
        in.setDirection(DcMotor.Direction.REVERSE);



        lockedArm = false;

        double time = 0;
        resetRuntime();

        waitForStart();
        while (opModeIsActive()) {
            boolean buttonPress = getRuntime() >= 0.2 + time;
            if (buttonPress) {
                if (gamepad1.right_bumper) {
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
                if (gamepad1.dpad_right && dpadUnlock) {
                    air.setPosition(0.2015);
                    planeActive = false;
                }

                if (gamepad1.dpad_left && dpadUnlock)
                    air.setPosition(0.3);

                if (gamepad1.dpad_up && dpadUnlock) {
                    arm = -2;
                    pA.setTargetPosition(2200);
                    pA.setPower(0.3);
                    pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    am.setTargetPosition(-40);
                    am.setPower(0.3);
                    am.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                }
                if (gamepad1.dpad_down && dpadUnlock) {
                    pA.setTargetPosition(400);
                    pA.setPower(1);
                    pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (gamepad1.triangle)
                    lockedArm = !lockedArm;

                if (gamepad1.circle)  //right
                    r1.setPosition(0.04);


                    if (gamepad1.square) //left
                        r2.setPosition(0.05);

                if (gamepad1.dpad_up && !dpadUnlock)
                    arm--;

                if (gamepad1.dpad_down && !dpadUnlock)
                    arm++;


                if (gamepad1.dpad_left && !dpadUnlock)
                    arm = -1;


                time = getRuntime();
            }

            if (lockedArm) {
                if (d1.getDistance(DistanceUnit.CM) <= 15)
                    mult = 0.4;

                if (d1.getDistance(DistanceUnit.CM) > 15)
                    mult = 1;

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


            if (gamepad1.right_trigger > 0 && !lockedArm)
                in.setPower(-iP);
            else if (gamepad1.left_trigger > 0 && !lockedArm)
                in.setPower(iP);
            else
                in.setPower(0);

            if (gamepad1.right_trigger > 0 && lockedArm && arm == -1) {
                leftArm = true;
                arm=0;
            }

            if (gamepad1.left_stick_button) {
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
                        am.setTargetPosition(-120);
                        am.setPower(0.6);
                        am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pA.setTargetPosition(490);
                    pA.setPower(0.6);
                    pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    armReset = false;
                    }
                    break;
                case 0:
                    if (p1C > -1)
                        pixeR = 1;
                    if (p2C > -1)
                        pixeL = 1;
                    if (!armReset) {
                            am.setTargetPosition(-50);
                        am.setPower(1);
                        am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                    if (am.getCurrentPosition() >= -52) {
                        pA.setTargetPosition(0);
                        pA.setPower(0.3);
                        pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    }
                    if (pA.getCurrentPosition() <= 15) {
                        arm++;
                    }
                    break;
                case 1:
                    if (pixeR == 1 && r1.getPosition() == 0.04)
                       rightReady = true;
                    if (pixeL == 1 && r2.getPosition() == 0.05)
                        leftReady = true;
                    if (pixeL == 0)
                        leftReady = true;
                    if (pixeR == 0)
                        rightReady = true;

                    if (leftReady && rightReady) {
                        double t = getRuntime();
                        pA.setTargetPosition(458);
                        pA.setPower(1);
                        pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        am.setTargetPosition(-100);
                        am.setPower(0.6);
                        am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armReset = true;
                    }
                    if (pA.getCurrentPosition() >= 450 && am.getCurrentPosition() <= -98) {
                        arm++;
                    }
                    break;
                case 2:
                    if (armReset) {

                        pA.setTargetPosition(2382);
                        pA.setPower(0.6);
                        pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                        am.setTargetPosition(186);
                        am.setPower(0.2);
                        am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armPos = 0;
                    }
                    if (pA.getCurrentPosition() >= 2380 && am.getCurrentPosition() >= 150) {
                        arm++;
                    }
                    break;
                case 3:
                    if (gamepad1.cross && buttonPress && lockedArm) {
                        if(armPos == 2) {
                            pA.setTargetPosition(2382);
                            pA.setPower(0.8);
                            pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            am.setTargetPosition(190);
                            am.setPower(0.4);
                            am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            armPos = 0;
                        }
                        else if (armPos == 1){
                            pA.setTargetPosition(2582);
                            pA.setPower(0.6);
                            pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            armPos = 2;
                        }
                        else {
                            pA.setTargetPosition(2782);
                            pA.setPower(0.5);
                            pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            armPos = 1;
                        }
                    }

                        if (leftArm && gamepad1.circle && buttonPress && p1C > -1 && lockedArm) {
                            r1.setPosition(0.13);
                            pixeR = 0;
                        }

                        if (leftArm && gamepad1.square && buttonPress && p2C > -1 && lockedArm) {
                            r2.setPosition(0.126);
                            pixeL = 0;
                        }

                    if (pixeR == 0 && pixeL == 0) {
                        double t = getRuntime();
                        while (t + 1 > getRuntime());
                        leftArm=false;
                        arm = -1;
                    }
                    break;
            }
                if (c1.getRawLightDetected() > 400) {
                    if (c1.getLightDetected() == 1)
                        p1C = 0;

                    else if (c1.getLightDetected() > 0.9)
                        p1C = 3;

                    else if (c1.green() >= c1.red() + c1.blue() || c1.getLightDetected() > 0.7)
                        p1C = 1;

                    else
                        p1C = 2;

                }

                if (c2.getRawLightDetected() > 300) {
                    if (c2.green() >= c2.red() + c2.blue() || c2.getLightDetected() >= 0.25)
                        p2C = 1;
                    else if (c2.getRawLightDetected() > 750)
                        p2C = 0;
                    else if (c2.getRawLightDetected() > 600)
                        p2C = 3;
                    else
                        p2C = 2;
                }



            if (getRuntime() > 90 || gamepad1.dpad_right)
                dpadUnlock = true;


            p1 = 1;
            p2 = 1;

            switch (p1C) {
                case -1:
                    color1 = "no pixel";
                    p1 = 0;
                    break;
                case 0:
                    color1 = "white";
                    break;
                case 1:
                    color1 = "green";
                    break;
                case 2:
                    color1 = "yellow";
                    break;
                case 3:
                    color1 = "purple";
                    break;
            }

            switch (p2C) {
                case -1:
                    color2 = "no pixel";
                    p2 = 0;
                    break;
                case 0:
                    color2 = "white";
                    break;
                case 1:
                    color2 = "green";
                    break;
                case 2:
                    color2 = "yellow";
                    break;
                case 3:
                    color2 = "purple";
                    break;
            }

            telemetry.addData("is slow mode", mult==0.4);
            telemetry.addData("locked arm", lockedArm);
            telemetry.addData("dpad unlocked", dpadUnlock);
            telemetry.addData("arm location", arm);
;            telemetry.addData("distance (cm)", d1.getDistance(DistanceUnit.CM));
            telemetry.addData("pixel count", p1+p2);
            telemetry.addData("right pixel", color1);
            telemetry.addData("left pixel", color2);
            telemetry.addData("Time", getRuntime());

            telemetry.update();

        }

    }
}