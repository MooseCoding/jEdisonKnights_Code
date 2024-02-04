package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.rev.RevColorSensorV3;

public class ColorSensors {
    private RevColorSensorV3 c1;
    private RevColorSensorV3 c2;

    public ColorSensors(RevColorSensorV3 c1, RevColorSensorV3 c2) {
        this.c1 = c1;
        this.c2 = c2;
    }

    public String[] returnColor() {
        String cr1;
        String cr2;

        return new String[] {"hi"} ;
    }
}
