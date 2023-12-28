package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp

public class motorTester extends LinearOpMode {

    int aprilTheta;
    int aprilDistance;

    IMU imu;//x-axis rotation = , y-axis rotation = , z-axis rotation

    private DcMotor br;

    private DcMotor i;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor fl;

    private DcMotor pA;

    private DcMotor am;
    private Servo r1, r2;

    private RevColorSensorV3 c1, c2;
    boolean speedMultiOn;

    double speedMulti;



    public void runOpMode() {

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
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
                = RevHubOrientationOnRobot.LogoFacingDirection.values();
        RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
                = RevHubOrientationOnRobot.UsbFacingDirection.values();
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoFacingDirections[4], usbFacingDirections[0])));

        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        fl = hardwareMap.get(DcMotor.class, "fl");
        i = hardwareMap.get(DcMotor.class, "intake");
        c1 = hardwareMap.get(RevColorSensorV3.class, "c1");
        c2 = hardwareMap.get(RevColorSensorV3.class, "c2");


        am = hardwareMap.get(DcMotor.class, "am");

        r1 = hardwareMap.get(Servo.class, "r1");
        r2 = hardwareMap.get(Servo.class, "r2");

        r2.setDirection(Servo.Direction.REVERSE);

        pA = hardwareMap.get(DcMotor.class, "pA");
        //pA2 = hardwareMap.get(DcMotor.class, "pA2"); //must be run at a negative power relative to pA1
        pA.setDirection(DcMotorSimple.Direction.FORWARD);
        //pA2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        speedMulti = 1;
        speedMultiOn = false;

        waitForStart();
        //servo 0 = bl
        //servo 1 = br
        //servo 2 = fr
        //servo 3 = f
        while (opModeIsActive()) {
            if(gamepad1.left_stick_button) {
                pA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                am.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if(this.gamepad1.triangle) {
                br.setPower(70);
                for (int i = 0; i < 4000; i++) {
                    telemetry.update();
                }
                br.setPower(0);
            }
            if(this.gamepad1.cross) {
                fl.setPower(70);
                for (int i = 0; i < 4000; i++) {
                    telemetry.update();
                }
                fl.setPower(0);
            }
            if(this.gamepad1.square) {
                fl.setPower(-70);
                for (int i = 0; i < 4000; i++) {
                    telemetry.update();
                }
                fl.setPower(0);
            }
            if(this.gamepad1.circle) {
                bl.setPower(70);
                for (int i = 0; i < 4000; i++) {
                    telemetry.update();
                }
                bl.setPower(0);
            }

            if (gamepad1.dpad_up) {
                r1.setPosition(0.17);
                r2.setPosition(0.87);
            }
            if (gamepad1.dpad_down) {
                r1.setPosition(0.18);
                r2.setPosition(0.86);
            }
            if(gamepad1.dpad_right) {
                r1.setPosition(0.042);
                r2.setPosition(0.052);
            }
            if (gamepad1.dpad_left) {
                r1.setPosition(0.126);
                r2.setPosition(0.13);
            }
            if (gamepad1.right_bumper) {
               pA.setPower(1);
            }
            else {
                pA.setPower(0);
            }
            if(gamepad1.left_bumper) {
                pA.setPower(-1);
            }
            else {
                pA.setPower(0);
            }
            telemetry.addData("right trigger", gamepad1.right_trigger);
            telemetry.addData("left trigger", gamepad1.left_trigger);
            telemetry.addData(" motor speed", pA.getPower());
            telemetry.addData(" encoder pos", pA.getCurrentPosition());
            telemetry.addData("am pos", am.getCurrentPosition());
            telemetry.addData("c1 raw", c1.getRawLightDetected());
            telemetry.addData("c1 unraw", c1.getLightDetected());
            telemetry.addData("c1 green", c1.green());
            telemetry.addData("c1 red", c1.red());
            telemetry.addData("c1 blue", c1.blue());
            telemetry.addData("c2 raw", c2.getRawLightDetected());
            telemetry.addData("c2 unraw", c2.getLightDetected());
            telemetry.addData("c2 green", c2.green());
            telemetry.addData("c2 red", c2.red());
            telemetry.addData("c2 blue", c2.blue());
            int p1C= 0, p2C = 0;
            String color1 = "", color2 = "";
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
            else
                p1C = -1;

            if (c2.getRawLightDetected() > 300) {
                if ((c2.green() >= c2.red() + c2.blue() || c2.getLightDetected() >= 0.257) && c2.red() != c2.green())
                    p2C = 1;
                else if (c2.getRawLightDetected() > 750)
                    p2C = 0;
                else if (c2.getRawLightDetected() > 600)
                    p2C = 3;
                else
                    p2C = 2;
            }
            else
                p2C =-1;

            switch (p1C) {
                case -1:
                    color1 = "no pixel";
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
            telemetry.addLine();
            telemetry.addData("right pixel", color1);
            telemetry.addData("left pixel", color2);
            telemetry.update();
        }
    }
}
