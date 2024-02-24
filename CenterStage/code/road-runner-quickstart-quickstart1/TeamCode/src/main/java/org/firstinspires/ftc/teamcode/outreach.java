package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
@TeleOp
@Photon
public class outreach extends LinearOpMode {
    private MotorEx br, fr, bl, fl;
    private GamepadEx g;
    @Override
    public void runOpMode() {
        g = new GamepadEx(gamepad1);
        br = new MotorEx(hardwareMap, "br");
        bl = new MotorEx(hardwareMap, "bl");
        fr = new MotorEx(hardwareMap, "fr");
        fl = new MotorEx(hardwareMap, "fl");
        fl.setInverted(true); //reverse
        bl.setInverted(true); //reverse
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
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

            fl.set(0.4 * leftFrontPower);
            bl.set(0.4*leftBackPower);
            fr.set(0.4*rightFrontPower);
            br.set(0.4*rightBackPower);
        }
    }
}
