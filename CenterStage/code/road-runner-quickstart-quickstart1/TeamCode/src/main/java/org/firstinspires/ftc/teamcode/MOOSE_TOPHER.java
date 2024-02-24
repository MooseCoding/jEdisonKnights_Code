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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.wolfpackmachina.bettersensors.Drivers.ColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.position;

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

    private MotorEx br, fr, bl, fl, in, pA, am;
    private DcMotorEx BR, FR, BL, FL, IN, PA, AM;
    private double mult = 1, t = 0, airTime = 0; //multiplier for running motors at speed
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
         ButtonReader TRIANGLE = new ButtonReader(g, GamepadKeys.Button.Y),
                SQUARE = new ButtonReader(g, GamepadKeys.Button.X),
                CIRCLE = new ButtonReader(g, GamepadKeys.Button.B),
                CROSS = new ButtonReader(g, GamepadKeys.Button.A),
                DPAD_UP = new ButtonReader(g, GamepadKeys.Button.DPAD_UP),
                DPAD_DOWN = new ButtonReader(g, GamepadKeys.Button.DPAD_DOWN),
                DPAD_LEFT = new ButtonReader(g, GamepadKeys.Button.DPAD_LEFT),
                DPAD_RIGHT = new ButtonReader(g, GamepadKeys.Button.DPAD_RIGHT),
                LEFT_STICK_BUTTON = new ButtonReader(g, GamepadKeys.Button.LEFT_STICK_BUTTON),
                RIGHT_STICK_BUTTON = new ButtonReader(g, GamepadKeys.Button.RIGHT_STICK_BUTTON),
                LEFT_BUMPER = new ButtonReader(g, GamepadKeys.Button.LEFT_BUMPER),
                RIGHT_BUMPER = new ButtonReader(g, GamepadKeys.Button.RIGHT_BUMPER);

         TriggerReader RIGHT_TRIGGER = new TriggerReader(g, GamepadKeys.Trigger.RIGHT_TRIGGER),
                LEFT_TRIGGER = new TriggerReader(g, GamepadKeys.Trigger.LEFT_TRIGGER);

        PhotonLynxVoltageSensor vS = hardwareMap.getAll(PhotonLynxVoltageSensor.class).iterator().next();


        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        br = new MotorEx(hardwareMap, "br");
        bl = new MotorEx(hardwareMap, "bl");
        fr = new MotorEx(hardwareMap, "fr");
        fl = new MotorEx(hardwareMap, "fl");
        in = new MotorEx(hardwareMap, "intake");
        pA = new MotorEx(hardwareMap, "pA");
        am = new MotorEx(hardwareMap, "am");
        BR = hardwareMap.get(DcMotorEx.class, "br");
        FR = hardwareMap.get(DcMotorEx.class, "br");
        BL = hardwareMap.get(DcMotorEx.class, "br");
        FL = hardwareMap.get(DcMotorEx.class, "br");
        IN = hardwareMap.get(DcMotorEx.class, "br");
        AM = hardwareMap.get(DcMotorEx.class, "br");
        PA = hardwareMap.get(DcMotorEx.class, "br");

        r1 = hardwareMap.get(Servo.class, "r1");
        r2 = hardwareMap.get(Servo.class, "r2");
        air = hardwareMap.get(Servo.class, "air");
        c1 = hardwareMap.get(ColorSensorV3.class, "c1");
        d1 = hardwareMap.get(Rev2mDistanceSensor.class, "d1");
        c2 = hardwareMap.get(ColorSensorV3.class, "c2");

        fl.setInverted(true); //reverse
        bl.setInverted(true); //reverse
        r2.setDirection(Servo.Direction.REVERSE);
        in.setInverted(true); //reverse

        lockedArm = false;

        resetRuntime();

        pA.setRunMode(MotorEx.RunMode.PositionControl);
        am.setRunMode(Motor.RunMode.PositionControl);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
                switch (drive_mode_current) {
                    case INIT:
                        while (!RIGHT_TRIGGER.wasJustPressed()) {
                            RIGHT_TRIGGER.readValue();
                        }
                        drive_mode_current = DRIVE_MODE.UNLOCKED_ARM;
                        break;
                    case UNLOCKED_ARM:
                        RIGHT_TRIGGER.readValue();
                        LEFT_TRIGGER.readValue();
                        TRIANGLE.readValue();

                        if (RIGHT_TRIGGER.isDown() && !lockedArm)
                            intake_current_state = INTAKE_STATE.INTAKE;
                        else if (LEFT_TRIGGER.isDown() && !lockedArm)
                            intake_current_state = INTAKE_STATE.OUTTAKE;
                        else
                            intake_current_state = INTAKE_STATE.HOLD;
                        if (TRIANGLE.wasJustPressed())
                            drive_mode_current = DRIVE_MODE.LOCKED_ARM;
                        break;
                    case LOCKED_ARM:
                        LEFT_STICK_BUTTON.readValue();
                        TRIANGLE.readValue();

                        if (LEFT_STICK_BUTTON.wasJustPressed()) {
                            pixeL = 1;
                            pixeR = 1;
                        }
                        if (d1.getDistance(DistanceUnit.CM)/10.0 <= 15)
                            speed_current_state = SPEED_STATE.BOARD;

                        if (d1.getDistance(DistanceUnit.CM)/10.0 > 15)
                            speed_current_state = SPEED_STATE.NORMAL;

                        if (TRIANGLE.wasJustPressed()) {
                            drive_mode_current = DRIVE_MODE.UNLOCKED_ARM;
                        }
                        break;
                }

            switch(dpad_current_state) {
                case ENDGAME:
                    DPAD_RIGHT.readValue();
                    DPAD_LEFT.readValue();
                    DPAD_UP.readValue();
                    DPAD_DOWN.readValue();

                    if (DPAD_RIGHT.wasJustPressed()) {
                        airplane_current_state = AIRPLANE_STATE.LAUNCH;
                    }
                    if (DPAD_UP.wasJustPressed()) {
                        arm_current_state = ARM_STATE.HANG;
                    }
                    if (DPAD_DOWN.wasJustPressed()) {
                        arm_current_state = ARM_STATE.RISE;
                    }
                    if (DPAD_LEFT.wasJustPressed()) {
                        dpad_current_state = DPAD_STATE.DPAD_STANDARD;
                    }
                    break;
                case DPAD_STANDARD:
                    DPAD_UP.readValue();
                    DPAD_LEFT.readValue();

                    if (DPAD_UP.wasJustPressed()) {
                        arm_current_state = ARM_STATE.HOLD;
                    }
                    if (DPAD_LEFT.wasJustPressed()) {
                        dpad_current_state = DPAD_STATE.ENDGAME;
                    }
                    break;
            }


            double max;

            double axial = -g.getLeftY();
            double lateral = -g.getLeftX();
            double yaw = -g.getRightX();

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

            /*
            if (axial == 1 && (lateral < 0.1 && lateral > -0.1) && (yaw < 0.1 && yaw > -0.1) && wantedAngle == 69420) {
                wantedAngle = position.getHeading();
                pid = 0;
            }
            else if (axial == -1 && (lateral < 0.1 && lateral > -0.1) && (yaw < 0.1 && yaw > -0.1) && wantedAngle == 69420) {
                wantedAngle = position.getHeading();
                pid = 1;
            }
            else if (lateral == -1 && (axial < 0.1 && axial > -0.1) && (yaw < 0.1 && yaw > -0.1) && wantedAngle == 69420) {
                wantedAngle = position.getHeading();
                pid = 2;
            }
            else if (lateral == 1 && (axial < 0.1 && axial > -0.1) && (yaw < 0.1 && yaw > -0.1) && wantedAngle == 69420) {
                wantedAngle = position.getHeading();
                pid = 3;
            }
            else {
                */ 
                //wantedAngle = 69420;

            
            /*error = wantedAngle - position.getHeading();
            switch (pid) {
                case 0:
                    if (error < 0.009) {
                        fl.set(1);
                        fr.set(1-kC*error);
                        bl.set(1);
                        br.set(1-kC*error);
                    }
                    else if (error > 0.009) {
                        fl.set(1-kC*error);
                        bl.set(1-kC*error);
                        fr.set(1);
                        br.set(1);

                    }
                    else {
                        fl.set(1);
                        fr.set(1);
                        br.set(1);
                        bl.set(1);
                    }
                    break;
                case 1:
                    if (error < 0.009) {
                        fl.set(-1);
                        fr.set(-1+kC*error);
                        bl.set(-1);
                        br.set(-1+kC*error);
                    }
                    else if (error > 0.009) {
                        fl.set(-1+kC*error);
                        bl.set(-1+kC*error);
                        fr.set(-1);
                        br.set(-1);

                    }
                    else {
                        fl.set(-1);
                        fr.set(-1);
                        br.set(-1);
                        bl.set(-1);
                    }

            }
            */ 


            switch(arm_current_state) {
                case HOLD:
                    RIGHT_TRIGGER.readValue();

                    if (armReset) {
                        r1.setPosition(0.126);
                        r2.setPosition(0.13);
                        am.setTargetPosition(-120);
                        am.set(0.6);

                    pA.setTargetPosition(490);
                    pA.set(0.6);


                    armReset = false;
                    }
                    if (RIGHT_TRIGGER.isDown() && lockedArm) {
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
                            am.setTargetPosition(-60);
                        am.set(0.6);

                }
                    if (am.getCurrentPosition() >= -80) {
                        pA.setTargetPosition(10);
                        pA.set(0.3);


                    }
                    if (pA.getCurrentPosition() <= 30) {
                        arm++;
                    }
                    break;
                case WAIT:
                    CIRCLE.readValue();

                    if (CIRCLE.wasJustPressed()) { //right
                        r1.setPosition(0.04);
                        rightReady = true;

                    }

                    SQUARE.readValue();

                    if (SQUARE.wasJustPressed()) {//left
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
                           pA.setTargetPosition(458);
                           pA.set(1);

                           am.setTargetPosition(-100);
                           am.set(0.6);

                           armReset = true;
                           leftReady = false;
                           rightReady = false;
                           t = 0;
                       }
                   }
                    if (pA.getCurrentPosition() >= 450 && am.getCurrentPosition() <= -98) {
                        arm++;
                    }
                    break;
                case MOVE:
                    if (armReset) {
                        pA.setTargetPosition(2532);
                        pA.set(0.6);


                        am.setTargetPosition(190);
                        am.set(0.2);

                        armPos = 2;
                    }
                    if (pA.getCurrentPosition() >= 2180 && am.getCurrentPosition() >= 160) {
                        arm++;
                    }
                    break;
                case DEPLOY:
                    CROSS.readValue();
                    CIRCLE.readValue();
                    SQUARE.readValue();

                    if (CROSS.wasJustPressed() && lockedArm) {
                        if(armPos == 2) {
                            pA.setTargetPosition(2182);
                            pA.set(0.8);

                            am.setTargetPosition(190);
                            am.set(0.4);

                            armPos = 0;
                        }
                        else if (armPos == 1){
                            pA.setTargetPosition(2582);
                            pA.set(0.6);

                        }
                        else {
                            pA.setTargetPosition(2582);
                            pA.set(0.5);

                            armPos = 1;
                        }
                      
                    }

                        if (leftArm && CIRCLE.wasJustPressed() && pixeR > -1 && lockedArm) {
                            r1.setPosition(0.13);
                            pixeR = 0;
                           
                        }

                        if (leftArm && SQUARE.wasJustPressed() && pixeL > -1 && lockedArm) {
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
                    DPAD_DOWN.readValue();

                    arm = -2;
                    pA.setTargetPosition(2200);
                    pA.set(0.3);

                    am.setTargetPosition(-40);
                    am.set(0.3);

                    if (DPAD_DOWN.wasJustPressed()) {
                        arm_current_state = ARM_STATE.RISE;
                    }
                    break;
                case RISE:
                    if (vS.getCachedVoltage() < 7) {
                        fl.set(0);
                        fr.set(0);
                        bl.set(0);
                        br.set(0);
                        in.set(0);
                    }
                    pA.setTargetPosition(400);
                    pA.set(1);
                    break;
                case ZERO_ARM:
                    RIGHT_TRIGGER.readValue();
                    LEFT_TRIGGER.readValue();
                    CROSS.readValue();
                    LEFT_BUMPER.readValue();
                    LEFT_STICK_BUTTON.readValue();
                    RIGHT_STICK_BUTTON.readValue();

                    if (RIGHT_TRIGGER.wasJustPressed()) {
                        pA.setTargetPosition(pA.getCurrentPosition() + 10);
                        pA.set(1);
                    }
                    else if (LEFT_TRIGGER.wasJustPressed()) {
                        pA.setTargetPosition(pA.getCurrentPosition() - 10);
                        pA.set(1);
                    }
                    if (CROSS.wasJustPressed()) {
                        pA.resetEncoder();
                        am.resetEncoder();
                    }
                    if (RIGHT_STICK_BUTTON.wasJustPressed() && LEFT_BUMPER.isDown())
                        arm_current_state = ARM_STATE.ZERO_CLAW;
                    else if (LEFT_STICK_BUTTON.wasJustPressed() && LEFT_BUMPER.isDown())
                        arm_current_state = ARM_STATE.HOLD;
                    break;
                case ZERO_CLAW:
                    RIGHT_TRIGGER.readValue();
                    LEFT_TRIGGER.readValue();
                    CROSS.readValue();
                    LEFT_BUMPER.readValue();
                    LEFT_STICK_BUTTON.readValue();
                    RIGHT_STICK_BUTTON.readValue();

                    if (RIGHT_TRIGGER.wasJustPressed()) {
                        am.setTargetPosition(am.getCurrentPosition() + 10);
                        am.set(1);
                    }
                    else if (LEFT_TRIGGER.wasJustPressed()) {
                        am.setTargetPosition(am.getCurrentPosition() - 10);
                        am.set(1);
                    }
                    if (CROSS.wasJustPressed()) {
                        am.resetEncoder();
                        am.resetEncoder();
                    }
                    if (RIGHT_STICK_BUTTON.wasJustPressed() && LEFT_BUMPER.isDown())
                        arm_current_state = ARM_STATE.ZERO_ARM;
                    else if (LEFT_STICK_BUTTON.wasJustPressed() && LEFT_BUMPER.isDown())
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
                    RIGHT_BUMPER.readValue();

                    if (RIGHT_BUMPER.wasJustPressed())
                        speed_current_state = SPEED_STATE.SLOW;
                    fl.set(leftFrontPower);
                    bl.set(leftBackPower);
                    fr.set(rightFrontPower);
                    br.set(rightBackPower);
                    break;
                case SLOW:
                    RIGHT_BUMPER.readValue();

                    if (RIGHT_BUMPER.wasJustPressed())
                        speed_current_state = SPEED_STATE.NORMAL;
                    fl.set(leftFrontPower * mult);
                    bl.set(leftBackPower * mult);
                    fr.set(rightFrontPower * mult);
                    br.set(rightBackPower * mult);
                    break;
                case BOARD:
                    fl.set(leftFrontPower * 0.2);
                    bl.set(leftBackPower * 0.2);
                    fr.set(rightFrontPower * 0.2);
                    br.set(rightBackPower * 0.2);
                    break;
            }

            switch (intake_current_state) {
                case HOLD:
                    if (in.get() != 0) {
                        in.set(0);
                    }
                    break;
                case INTAKE:
                    if (in.get() != 0.45) {
                        in.set(0.45);
                    }
                    break;
                case OUTTAKE:
                    if (in.get() != -0.45) {
                        in.set(-0.45);
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

            TRIANGLE.readValue();
            RIGHT_BUMPER.readValue();

            telemetry.addData("distance (cm)", d1.getDistance(DistanceUnit.CM)/10.0);
            telemetry.addData("pixel count", p1+p2);
            telemetry.addData("right pixel", color1);
            telemetry.addData("left pixel", color2);
            telemetry.addData("y", TRIANGLE.wasJustPressed());
            telemetry.addData("right bumper", RIGHT_BUMPER.wasJustPressed());
            telemetry.addData("Voltage", vS.getCachedVoltage());
            telemetry.addData("Time", getRuntime());

            telemetry.update();
            

        }

    }
}