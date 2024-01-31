package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class mooseTeleopSafeJoey extends LinearOpMode {
    private String color1, color2;
    private int armPos, arm = -1, p1C, p2C, p1, p2, pixeL = 0, pixeR = 0; //pixel 1/2 colors (-1 no pixel , 0 white, 1 green, 2 yellow, 3 purple)
    private RevColorSensorV3 c1, c2;
    private DcMotor br, fr, bl, fl, in, pA, am;
    private double speedMulti = 1.0, mult = 1, iP = 0.6, bellT = 0; //multiplier for running motors at speed
    private boolean leftArm = false, planeActive = true, armReset = true, rightReady = false, leftReady = false,  lockedArm, dpadUnlock = false;
    private Servo r1, r2, air, bell;
    private Rev2mDistanceSensor d1;

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
        bell = hardwareMap.get(Servo.class, "bell");


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
                        gamepad1.rumble(1, 0, 100);
                    } else {
                        mult = 1;
                        gamepad1.rumble(0, 1, 100);
                    }
                }
                if (gamepad1.dpad_left) {
                    //change bell
                }

                if (gamepad1.triangle && arm == -1) {
                    arm++;
                }

                if (gamepad1.triangle && arm == 1) {
                    arm++;
                }
                else if (gamepad1.cross && (arm == 1 || arm == 0)) {
                    arm = -1;
                    armReset = true;
                }

                if (gamepad1.dpad_right && dpadUnlock) {
                    air.setPosition(0.3);
                }

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

                if (gamepad1.circle && arm == 1) { //right
                    r1.setPosition(0.04);

                    rightReady = true;
                }

                if (gamepad1.square && arm == 1) {//left
                    r2.setPosition(0.05);
                    leftReady = true;
                }

                time = getRuntime();

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


            if (gamepad1.right_trigger > 0 && (p1C == -1 || p2C == -1))
                in.setPower(-iP);
            else if (gamepad1.left_trigger > 0 )
                in.setPower(iP);
            else
                in.setPower(0);


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
                        am.setTargetPosition(-100);
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
                        am.setTargetPosition(-45);
                        am.setPower(0.8);
                        am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    if (am.getCurrentPosition() >= -47) {
                        pA.setTargetPosition(100);
                        pA.setPower(0.2);
                        pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    }
                    if (pA.getCurrentPosition() <= 110) {
                        arm++;
                    }
                    break;
                case 1:
                    if (pixeL == 0)
                        leftReady = true;
                    if (pixeR == 0)
                        rightReady = true;

                    if (leftReady && rightReady) {
                        armReset = true;
                    }
                    break;
                case 2:
                    if (armReset && pA.getCurrentPosition() < 400) {
                        pA.setTargetPosition(458);
                        pA.setPower(1);
                        pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        am.setTargetPosition(-100);
                        am.setPower(0.6);
                        am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armReset = false;
                    }
                    if (pA.getCurrentPosition() > 450 && am.getCurrentPosition() < -90 && !armReset) {
                        pA.setTargetPosition(2382);
                        pA.setPower(0.6);
                        pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        am.setTargetPosition(220);
                        am.setPower(0.2);
                        am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armPos = 0;
                        armReset = true;
                    }

                    if (pA.getCurrentPosition() >= 2380 && am.getCurrentPosition() >= 160) {
                        arm++;
                    }
                    break;
                case 3:
                    if (gamepad1.circle && buttonPress && pixeR > -1) {
                        r1.setPosition(0.13);
                        pixeR = 0;
                    }

                    if (gamepad1.square && buttonPress && pixeL > -1) {
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

                else if (c1.getRawLightDetected() > 1900 || c1.getLightDetected() < 0.5)
                    p1C = 3;

                else if (c1.green() >= c1.red() + c1.blue() || c1.getLightDetected() > 0.7)
                    p1C = 1;

                else
                    p1C = 2;

            }
            else
                p1C = -1;

            if (c2.getRawLightDetected() > 300) {
                if((c2.green() >= c2.red() + c2.blue() || c2.getLightDetected() >= 0.25)&& (c2.getRawLightDetected() < 750 && c2.getRawLightDetected() > 730) )
                    p2C = 1;
                else if (c2.getRawLightDetected() > 850)
                    p2C = 0;
                else if (c2.getRawLightDetected() > 600)
                    p2C = 3;
                else
                    p2C = 2;
            }
            else
                p2C =-1;


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
            if (gamepad1.touchpad_finger_1 || gamepad1.touchpad_finger_2 || gamepad1.left_stick_button || gamepad1.right_stick_button) {
                if (bell.getPosition() == 0) {
                    bellT = getRuntime();
                }
                bell.setPosition(0);
                if (bellT + 0.5 > getRuntime()) {
                    bell.setPosition(1);
                }

            }


            telemetry.addData("am pos", am.getCurrentPosition());
            telemetry.addData("trues", leftReady);
            telemetry.addData("right", rightReady);
            telemetry.addData("is slow mode", mult==0.4);
            telemetry.addData("locked arm", lockedArm);
            telemetry.addData("dpad unlocked", dpadUnlock);
            telemetry.addData("arm location", arm);
            telemetry.addData("distance (cm)", d1.getDistance(DistanceUnit.CM));
            telemetry.addData("pixel count", p1+p2);
            telemetry.addData("right pixel", color1);
            telemetry.addData("left pixel", color2);
            telemetry.addData("Time", getRuntime());

            telemetry.update();

        }

    }
}
