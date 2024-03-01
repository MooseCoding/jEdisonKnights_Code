package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.wolfpackmachina.bettersensors.Drivers.ColorSensorV3;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp
@Photon
public class MOOSE_TOPHER extends LinearOpMode {
    private enum ARM_STATE {
        HOLD,
        GRAB,
        WAIT,
        MOVE,
        DEPLOY,
        HANG,
        RISE,
        ZERO_ARM,
        ZERO_CLAW
    }

    private enum AIRPLANE_STATE {
        HOLD,
        LAUNCH,
        DEPLOYED
    }

    private enum INTAKE_STATE {
        HOLD,
        INTAKE,
        OUTTAKE
    }

    private enum SPEED_STATE {
        NORMAL,
        SLOW,
        BOARD
    }

    private enum DRIVE_MODE {
        INIT,
        LOCKED_ARM,
        UNLOCKED_ARM,
    }

    private enum DPAD_STATE {
        DPAD_STANDARD,
        ENDGAME
    }

    private ARM_STATE arm_current_state = ARM_STATE.HOLD;

    private AIRPLANE_STATE airplane_current_state = AIRPLANE_STATE.HOLD;

    private INTAKE_STATE intake_current_state = INTAKE_STATE.HOLD;

    private SPEED_STATE speed_current_state = SPEED_STATE.NORMAL;

    private DRIVE_MODE drive_mode_current = DRIVE_MODE.INIT;

    private DPAD_STATE dpad_current_state = DPAD_STATE.DPAD_STANDARD;

    private String color1, color2;
    private int armPos, arm = -1, p1C, p2C, p1, p2, pixeL = 0, pixeR = 0, pid = -1; //pixel 1/2 colors (-1 no pixel , 0 white, 1 green, 2 yellow, 3 purple)
    private ColorSensorV3 c1, c2;

    //private MotorEx br, fr, bl, fl, in, PA, am;
    private DcMotorEx BR, FR, BL, FL, IN, PA, AM;
    private double mult = 1, t = 0, airTime = 0, PA_power = 0, am_power = 0;; //multiplier for running motors at speed
    private boolean leftArm = false, planeActive = true, armReset = true, rightReady = false, leftReady = false,  lockedArm, dpadUnlock = false, isTime = false;
    private Servo r1, r2, air;
    private Rev2mDistanceSensor d1;
    private IMU imu;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        GamepadEx g = new GamepadEx(gamepad1);
        Gamepad g_copy = new Gamepad();

        PhotonLynxVoltageSensor vS = hardwareMap.getAll(PhotonLynxVoltageSensor.class).iterator().next();


        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        /*
        br = new MotorEx(hardwareMap, "br");
        bl = new MotorEx(hardwareMap, "bl");
        fr = new MotorEx(hardwareMap, "fr");
        fl = new MotorEx(hardwareMap, "fl");
        in = new MotorEx(hardwareMap, "intake");
        pA = new MotorEx(hardwareMap, "pA");
        am = new MotorEx(hardwareMap, "am");*/

        BR = hardwareMap.get(DcMotorEx.class, "br");
        FR = hardwareMap.get(DcMotorEx.class, "br");
        BL = hardwareMap.get(DcMotorEx.class, "br");
        FL = hardwareMap.get(DcMotorEx.class, "br");
        IN = hardwareMap.get(DcMotorEx.class, "br");
        AM = hardwareMap.get(DcMotorEx.class, "br");
        PA = hardwareMap.get(DcMotorEx.class, "pA");

        r1 = hardwareMap.get(Servo.class, "r1");
        r2 = hardwareMap.get(Servo.class, "r2");
        air = hardwareMap.get(Servo.class, "air");
        c1 = hardwareMap.get(ColorSensorV3.class, "c1");
        d1 = hardwareMap.get(Rev2mDistanceSensor.class, "d1");
        c2 = hardwareMap.get(ColorSensorV3.class, "c2");

        FL.setDirection(DcMotorSimple.Direction.REVERSE); //reverse
        BL.setDirection(DcMotorSimple.Direction.REVERSE); //reverse
        r2.setDirection(Servo.Direction.REVERSE);
        IN.setDirection(DcMotorSimple.Direction.REVERSE); //reverse
        PA.setDirection(DcMotorSimple.Direction.REVERSE);

        lockedArm = false;

        resetRuntime();



        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            g_copy.copy(gamepad1);
                switch (drive_mode_current) {
                    case INIT:
                        while(gamepad1.right_trigger == 0) {
                            if (gamepad1.left_stick_button && !g_copy.left_stick_button) {
                                PA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                AM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            }
                            telemetry.addData("PA pos", PA.getCurrentPosition());
                            telemetry.addData("am pos", AM.getCurrentPosition());
                            telemetry.update();
                        }
                        drive_mode_current = DRIVE_MODE.UNLOCKED_ARM;
                        break;
                    case UNLOCKED_ARM:
                        if (gamepad1.right_trigger > 0 && !lockedArm)
                            intake_current_state = INTAKE_STATE.INTAKE;
                        else if (gamepad1.left_trigger > 0 && !lockedArm)
                            intake_current_state = INTAKE_STATE.OUTTAKE;
                        else
                            intake_current_state = INTAKE_STATE.HOLD;
                        if (gamepad1.triangle && !gamepad1.triangle)
                            drive_mode_current = DRIVE_MODE.LOCKED_ARM;
                        break;
                    case LOCKED_ARM:
                        if (gamepad1.left_stick_button && !g_copy.left_stick_button) {
                            pixeL = 1;
                            pixeR = 1;
                        }
                        if (d1.getDistance(DistanceUnit.CM)/10.0 <= 15)
                            speed_current_state = SPEED_STATE.BOARD;

                        if (d1.getDistance(DistanceUnit.CM)/10.0 > 15)
                            speed_current_state = SPEED_STATE.NORMAL;

                        if (gamepad1.triangle && !g_copy.triangle) {
                            drive_mode_current = DRIVE_MODE.UNLOCKED_ARM;
                        }
                        break;
                }

            switch(dpad_current_state) {
                case ENDGAME:
                    if (gamepad1.dpad_right && !g_copy.dpad_right) {
                        airplane_current_state = AIRPLANE_STATE.LAUNCH;
                    }
                    if (gamepad1.dpad_up && !g_copy.dpad_up) {
                        arm_current_state = ARM_STATE.HANG;
                    }
                    if (gamepad1.dpad_down && !g_copy.dpad_down) {
                        arm_current_state = ARM_STATE.RISE;
                    }
                    if (gamepad1.dpad_left && !g_copy.dpad_left) {
                        dpad_current_state = DPAD_STATE.DPAD_STANDARD;
                    }
                    break;
                case DPAD_STANDARD:
                    if (gamepad1.dpad_up && !g_copy.dpad_up) {
                        arm_current_state = ARM_STATE.HOLD;
                    }
                    if (gamepad1.dpad_left && !g_copy.dpad_left) {
                        dpad_current_state = DPAD_STATE.ENDGAME;
                    }
                    break;
            }

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

            FL.setPower(leftFrontPower);
            BL.setPower(leftBackPower);
            FR.setPower(rightFrontPower);
            BR.setPower(rightBackPower);


            switch(arm_current_state) {
                case HOLD:
                    if (armReset) {
                        r1.setPosition(0.126);
                        r2.setPosition(0.13);
                        AM.setTargetPosition(-120);
                        AM.setPower(0.6);

                    PA.setTargetPosition(490);
                    PA.setPower(0.6);

                        PA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        AM.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    armReset = false;

                    }
                    if (gamepad1.right_trigger > 0 && lockedArm) {
                        leftArm = true;
                        arm_current_state = ARM_STATE.GRAB;
                    }
                    break;
                case GRAB:
                    if (p1C > -1)
                        pixeR = 1;
                    if (p2C > -1)
                        pixeL = 1;
                    if (!armReset) {

                            AM.setTargetPosition(-60);
                        AM.setPower(0.6);

                        AM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                }
                    if (AM.getCurrentPosition() >= -80) {
                        PA.setTargetPosition(10);
                        PA.setPower(0.3);
                        PA.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    }
                    if (PA.getCurrentPosition() <= 30) {
                        arm++;
                    }
                    break;
                case WAIT:
                    if (gamepad1.circle && !g_copy.circle) { //right
                        r1.setPosition(0.04);
                        rightReady = true;

                    }

                    if (gamepad1.square && !g_copy.square) {//left
                        r2.setPosition(0.05);
                        leftReady = true;

                    }

                    if (pixeL == 0)
                        leftReady = true;
                    if (pixeR == 0)
                        rightReady = true;

                   if (leftReady && rightReady) {
                       if (!isTime) {
                           t = getRuntime();
                           isTime = true;
                       }
                       if (t + 0.7 < getRuntime() && t != 0) {
                           PA.setTargetPosition(458);
                           PA.setPower(1);

                           AM.setTargetPosition(-100);
                           AM.setPower(1);

                           armReset = true;
                           leftReady = false;
                           rightReady = false;
                           t = 0;
                       }
                   }
                    if (PA.getCurrentPosition() >= 450 && AM.getCurrentPosition() <= -98) {
                        arm++;
                    }
                    break;
                case MOVE:
                    if (armReset) {
                        PA.setTargetPosition(2532);
                        PA.setPower(0.6);


                        AM.setTargetPosition(190);
                        AM.setPower(0.6);

                        PA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        AM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        armPos = 2;
                    }
                    if (PA.getCurrentPosition() >= 2180 && AM.getCurrentPosition() >= 160) {
                        arm++;
                    }
                    break;
                case DEPLOY:
                    if (gamepad1.cross && !g_copy.cross && lockedArm) {
                        if(armPos == 2) {
                            PA.setTargetPosition(2182);
                            PA.setPower(0.8);

                            AM.setTargetPosition(190);
                            AM.setPower(0.8);

                            PA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            AM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            armPos = 0;
                        }
                        else if (armPos == 1){
                            PA.setTargetPosition(2582);
                            PA.setPower(0.6);
                            PA.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                        }
                        else {
                            PA.setTargetPosition(2582);
                            PA.setPower(0.5);
                            PA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            armPos = 1;
                        }
                      
                    }

                        if (leftArm && gamepad1.circle && !g_copy.circle && pixeR > -1 && lockedArm) {
                            r1.setPosition(0.13);
                            pixeR = 0;
                           
                        }

                        if (leftArm && gamepad1.square && !g_copy.square && pixeL > -1 && lockedArm) {
                            r2.setPosition(0.126);
                            pixeL = 0;
                         
                        }

                    if (pixeR == 0 && pixeL == 0) {
                        if (isTime) {
                            t = getRuntime();
                            isTime = false;
                        }
                        if (t + 1 < getRuntime() && t != 0) {
                            leftArm = false;
                            arm = -1;
                        }
                    }

                    break;
                case HANG:
                    arm = -2;
                    PA.setTargetPosition(2200);
                    PA.setPower(0.3);

                    AM.setTargetPosition(-40);
                    AM.setPower(0.3);

                    if (gamepad1.dpad_down && !g_copy.dpad_down) {
                        arm_current_state = ARM_STATE.RISE;
                    }
                    break;
                case RISE:
                    if (vS.getCachedVoltage() < 7) {
                        FL.setPower(0);
                        FR.setPower(0);
                        BL.setPower(0);
                        BR.setPower(0);
                        IN.setPower(0);
                    }
                    PA.setTargetPosition(400);
                    PA.setPower(1);
                    PA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    break;
                case ZERO_ARM:
                    if (gamepad1.right_trigger > 0 && !(g_copy.right_trigger>0)) {
                        PA.setTargetPosition(PA.getCurrentPosition() + 10);
                        PA.setPower(1);
                        PA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    else if (gamepad1.left_trigger > 0 && !(g_copy.left_trigger>0)) {
                        PA.setTargetPosition(PA.getCurrentPosition() - 10);
                        PA.setPower(1);
                        PA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    }
                    if (gamepad1.cross && !g_copy.cross) {
                        PA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        AM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }
                    if (gamepad1.left_stick_button && !g_copy.left_stick_button && gamepad1.left_bumper && !g_copy.left_bumper)
                        arm_current_state = ARM_STATE.ZERO_CLAW;
                    else if (gamepad1.right_stick_button && !g_copy.right_stick_button && gamepad1.left_bumper && !g_copy.left_bumper)
                        arm_current_state = ARM_STATE.HOLD;
                    break;
                case ZERO_CLAW:
                    if (gamepad1.right_trigger > 0 && !(g_copy.right_trigger>0)) {
                        AM.setTargetPosition(AM.getCurrentPosition() + 10);
                        AM.setPower(1);
                        AM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    else if (gamepad1.left_trigger > 0 && !(g_copy.left_trigger>0)) {
                        AM.setTargetPosition(AM.getCurrentPosition() - 10);
                        AM.setPower(1);
                        AM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    if (gamepad1.cross && !g_copy.cross) {
                        AM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        PA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }
                    if (gamepad1.left_stick_button && !g_copy.left_stick_button && gamepad1.left_bumper && !g_copy.left_bumper)
                        arm_current_state = ARM_STATE.ZERO_ARM;
                    else if (gamepad1.right_stick_button && !g_copy.right_stick_button && gamepad1.left_bumper && !g_copy.left_bumper)
                        arm_current_state = ARM_STATE.HOLD;
            }

            switch(airplane_current_state) {
                case HOLD: //does nothing but improves readability
                    break;
                case LAUNCH:
                    if (air.getPosition() != 0.3) {
                        air.setPosition(0.3);
                        airTime = getRuntime();
                    }
                    if (getRuntime() - 3 > airTime)
                        airplane_current_state = AIRPLANE_STATE.DEPLOYED;
                    break;
                case DEPLOYED:
                    if (air.getPosition() != 0.2)
                        air.setPosition(0.2);
                    break;
            }

            switch (speed_current_state) {
                case NORMAL:
                    if (gamepad1.right_bumper && !g_copy.right_bumper)
                        speed_current_state = SPEED_STATE.SLOW;
                    FL.setPower(leftFrontPower);
                    BL.setPower(leftBackPower);
                    FR.setPower(rightFrontPower);
                    BR.setPower(rightBackPower);
                    break;
                case SLOW:
                    if (gamepad1.right_bumper && !g_copy.right_bumper)
                        speed_current_state = SPEED_STATE.NORMAL;
                    FL.setPower(leftFrontPower * mult);
                    BL.setPower(leftBackPower * mult);
                    FR.setPower(rightFrontPower * mult);
                    BR.setPower(rightBackPower * mult);
                    break;
                case BOARD:
                    FL.setPower(leftFrontPower * 0.2);
                    BL.setPower(leftBackPower * 0.2);
                    FR.setPower(rightFrontPower * 0.2);
                    BR.setPower(rightBackPower * 0.2);
                    break;
            }

            switch (intake_current_state) {
                case HOLD:
                    if (gamepad1.right_trigger > 0) {
                        intake_current_state = INTAKE_STATE.INTAKE;
                    }
                    else if (gamepad1.left_trigger > 0) {
                        intake_current_state = INTAKE_STATE.OUTTAKE;
                    }

                    else if (IN.getPower() != 0) {
                        IN.setPower(0);
                    }
                    break;
                case INTAKE:
                    if (IN.getPower() != 0.45) {
                        IN.setPower(0.45);
                    }
                    if (gamepad1.right_trigger == 0) {
                        intake_current_state = INTAKE_STATE.HOLD;
                    }

                    break;
                case OUTTAKE:
                    if (IN.getPower() != -0.45) {
                        IN.setPower(-0.45);
                    }
                    if (gamepad1.left_trigger == 0) {
                        intake_current_state = INTAKE_STATE.HOLD;
                    }
                    break;
            }


            if (getRuntime() > 90)
                dpad_current_state = DPAD_STATE.ENDGAME;

            p1 = 1;
            p2 = 1;
            p1C = 0;
            p2C = 0;



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

            telemetry.addData("distance (cm)", d1.getDistance(DistanceUnit.CM)/10.0);
            telemetry.addData("pixel count", p1+p2);
            telemetry.addData("right pixel", color1);
            telemetry.addData("left pixel", color2);
            telemetry.addData("Voltage", vS.getCachedVoltage());
            telemetry.addData("Time", getRuntime());
            telemetry.addData("PA pos", PA.getCurrentPosition());
            telemetry.addData("am pos", AM.getCurrentPosition());
            telemetry.addData("driver ", drive_mode_current);
            telemetry.addData("arm ", armReset);
            telemetry.addData("fl power", leftBackPower);
            telemetry.addData("fl power now", BL.getPower());
            telemetry.update();
            

        }

    }
}