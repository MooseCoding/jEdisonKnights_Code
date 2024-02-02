package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.position;

@TeleOp
@Photon
public class MOOSE_TOPHER extends LinearOpMode {
    private String color1, color2;
    private int armPos, arm = -1, p1C, p2C, p1, p2, pixeL = 0, pixeR = 0, pid = -1; //pixel 1/2 colors (-1 no pixel , 0 white, 1 green, 2 yellow, 3 purple)
    private RevColorSensorV3 c1, c2;

    private final int kC = 0, kI = 0, kD = 0;

    private DcMotor br, fr, bl, fl, in, pA, am;
    private double speedMulti = 1.0, mult = 1, iP = 0.45, t = 0, wantedAngle = 69420, error; //multiplier for running motors at speed
    private boolean leftArm = false, planeActive = true, armReset = true, rightReady = false, leftReady = false,  lockedArm, dpadUnlock = false, isTime = false;
    private Servo r1, r2, air;
    private Rev2mDistanceSensor d1;

    private Gamepad g = new Gamepad(), gC = new Gamepad();
    private IMU imu;

    private PhotonLynxVoltageSensor vS = hardwareMap.getAll(PhotonLynxVoltageSensor.class).iterator().next();

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

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

        SampleMecanumDrive d = new SampleMecanumDrive(hardwareMap);
        d.setPoseEstimate(position.getPosition());

        resetRuntime();

        waitForStart();
        while (opModeIsActive()) {
            g.copy(gC);
            gC.copy(gamepad1);

                if (gC.right_bumper && !g.right_bumper) {
                    if (mult == 1) {
                        mult = 0.4;
                        gC.rumble(1, 0, 100);
                    } else {
                        mult = 1;
                        gC.rumble(0, 1, 100);
                    }
            
                }
                if (gC.dpad_right && dpadUnlock && !g.dpad_right) {
                    air.setPosition(0.2);
            
                }

                if (gC.dpad_left && dpadUnlock && !g.dpad_left) {
                    air.setPosition(0.3);
                
                }

                if (gC.dpad_up && dpadUnlock && !g.dpad_up) {
                    arm = -2;
                    pA.setTargetPosition(2200);
                    pA.setPower(0.3);
                    pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    am.setTargetPosition(-40);
                    am.setPower(0.3);
                    am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                }
                if (gC.dpad_down && dpadUnlock && !g.dpad_down) {
                    pA.setTargetPosition(400);
                    pA.setPower(1);
                    pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                  
                }
                if (gC.triangle && !g.triangle) {
                    lockedArm = !lockedArm;
                    
        }

                if (gC.circle && arm == 1 && !g.circle ) { //right
                    r1.setPosition(0.04);

                    rightReady = true;
                    
                }

                if (gC.square && arm == 1 && !g.square) {//left
                    r2.setPosition(0.05);
                leftReady = true;
                  
            }

                if (gC.dpad_up && !dpadUnlock && !g.dpad_up) {
                    arm++;
                   
                }

                if (gC.dpad_down && !dpadUnlock && !g.dpad_down) {
                    arm--;
                  
                }


                if (gC.dpad_left && !dpadUnlock && !g.dpad_left) {
                    arm = -1;
                 
                }

                if (gC.left_bumper && gC.right_stick_button && !g.left_bumper) {
                    arm = -3;
                    am.setTargetPosition(0);
                    am.setPower(1);
                    am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pA.setTargetPosition(0);
                    pA.setPower(1);
                    pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    
                }

                if (gC.left_stick_button && gC.left_bumper && !g.left_stick_button) {
                    pA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    am.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                
                }

        
            if (lockedArm) {
                if (d1.getDistance(DistanceUnit.CM) <= 15)
                    mult = 0.4;

                if (d1.getDistance(DistanceUnit.CM) > 15)
                    mult = 1;

            }


            //ignore every button input within a 0.2 sec timespan as its neglible to affect results
            double max;

            double axial = -gC.left_stick_y;
            double lateral = gC.left_stick_x;
            double yaw = gC.right_stick_x;

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
                fl.setPower(leftFrontPower * mult);
                bl.setPower(leftBackPower * mult);
                fr.setPower(rightFrontPower * mult);
                br.setPower(rightBackPower * mult);
            
            /*error = wantedAngle - position.getHeading();
            switch (pid) {
                case 0:
                    if (error < 0.009) {
                        fl.setPower(1);
                        fr.setPower(1-kC*error);
                        bl.setPower(1);
                        br.setPower(1-kC*error);
                    }
                    else if (error > 0.009) {
                        fl.setPower(1-kC*error);
                        bl.setPower(1-kC*error);
                        fr.setPower(1);
                        br.setPower(1);

                    }
                    else {
                        fl.setPower(1);
                        fr.setPower(1);
                        br.setPower(1);
                        bl.setPower(1);
                    }
                    break;
                case 1:
                    if (error < 0.009) {
                        fl.setPower(-1);
                        fr.setPower(-1+kC*error);
                        bl.setPower(-1);
                        br.setPower(-1+kC*error);
                    }
                    else if (error > 0.009) {
                        fl.setPower(-1+kC*error);
                        bl.setPower(-1+kC*error);
                        fr.setPower(-1);
                        br.setPower(-1);

                    }
                    else {
                        fl.setPower(-1);
                        fr.setPower(-1);
                        br.setPower(-1);
                        bl.setPower(-1);
                    }

            }
            */ 




            if (gC.right_trigger > 0 && !lockedArm)
                in.setPower(-iP);
            else if (gC.left_trigger > 0 && !lockedArm)
                in.setPower(iP);
            else
                in.setPower(0);

            if (gC.right_trigger > 0 && lockedArm && arm == -1) {
                leftArm = true;
                arm=0;
            }

            if (gC.left_stick_button) {
                pixeL = 1;
                pixeR = 1;
            }
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
                            am.setTargetPosition(-60);
                        am.setPower(0.6);
                        am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                    if (am.getCurrentPosition() >= -80) {
                        pA.setTargetPosition(10);
                        pA.setPower(0.3);
                        pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    }
                    if (pA.getCurrentPosition() <= 30) {
                        arm++;
                    }
                    break;
                case 1:
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
                           pA.setPower(1);
                           pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                           am.setTargetPosition(-100);
                           am.setPower(0.6);
                           am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
                case 2:
                    if (armReset) {
                        pA.setTargetPosition(2532);
                        pA.setPower(0.6);
                        pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        am.setTargetPosition(190);
                        am.setPower(0.2);
                        am.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armPos = 2;
                    }
                    if (pA.getCurrentPosition() >= 2180 && am.getCurrentPosition() >= 160) {
                        arm++;
                    }
                    break;
                case 3:
                    if (gC.cross && !g.cross && lockedArm) {
                        if(armPos == 2) {
                            pA.setTargetPosition(2182);
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
                        }
                        else {
                            pA.setTargetPosition(2582);
                            pA.setPower(0.5);
                            pA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            armPos = 1;
                        }
                      
                    }

                        if (leftArm && gC.circle && !g.circle && pixeR > -1 && lockedArm) {
                            r1.setPosition(0.13);
                            pixeR = 0;
                           
                        }

                        if (leftArm && gC.square && !g.square && pixeL > -1 && lockedArm) {
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


            if (getRuntime() > 90 || gC.dpad_right)
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

            telemetry.addData("am pos", am.getCurrentPosition());
            telemetry.addData("trues", leftReady);
            telemetry.addData("right", rightReady);
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
