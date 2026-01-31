package org.firstinspires.ftc.teamcode.Core.Algorithms;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Selectors implements InterfaceSimple {
    public DigitalChannel Alliance_B, Alliance_R, Auto1, Auto2, Auto3, Auto4;

    public void start(HardwareMap hardware) {
        Alliance_B = hardware.get(DigitalChannel.class, "Alliance_B");
        Alliance_R = hardware.get(DigitalChannel.class, "Alliance_R");
        Auto1 = hardware.get(DigitalChannel.class, "Auto1");
        Auto2 = hardware.get(DigitalChannel.class, "Auto2");
        Auto3 = hardware.get(DigitalChannel.class, "Auto3");
        Auto4 = hardware.get(DigitalChannel.class, "Auto4");

        Alliance_B.setMode(DigitalChannel.Mode.INPUT);
        Alliance_R.setMode(DigitalChannel.Mode.INPUT);
        Auto1.setMode(DigitalChannel.Mode.INPUT);
        Auto2.setMode(DigitalChannel.Mode.INPUT);
        Auto3.setMode(DigitalChannel.Mode.INPUT);
        Auto4.setMode(DigitalChannel.Mode.INPUT);

    }


    public boolean isBlueAlliance() {
        return Alliance_B.getState();
    }

    public boolean isRedAlliance() {
        return Alliance_R.getState();
    }

    public int getAuto() {
        if (!Auto1.getState()) return 1;
        else if (!Auto2.getState()) return 2;
        else if (!Auto3.getState()) return 3;
        else if (!Auto4.getState()) return 4;
        else return 0;
    }



    @Override
    public void update() {

    }
}
