package org.firstinspires.ftc.teamcode.Core;

import org.firstinspires.ftc.teamcode.Core.Algorithms.PIDFOController;
import org.firstinspires.ftc.teamcode.Core.Hardware.InterfaceSimple;
import org.firstinspires.ftc.teamcode.Core.Parts.Drivetrain;

public class Robot implements InterfaceSimple {

   public PIDFOController PID = new PIDFOController();
    public Drivetrain drivetrain = new Drivetrain();
    @Override
    public void update() {
        drivetrain.update();
    }
}
