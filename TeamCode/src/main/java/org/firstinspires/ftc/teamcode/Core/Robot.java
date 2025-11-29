package org.firstinspires.ftc.teamcode.Core;

import org.firstinspires.ftc.teamcode.Core.Hardware.InterfaceSimple;
import org.firstinspires.ftc.teamcode.Core.Parts.Drivetrain;

public class Robot implements InterfaceSimple {


    Drivetrain drivetrain;
    @Override
    public void update() {
        drivetrain.update();
    }
}
