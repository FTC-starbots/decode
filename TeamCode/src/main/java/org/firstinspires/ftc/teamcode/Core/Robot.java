package org.firstinspires.ftc.teamcode.Core;

import org.firstinspires.ftc.teamcode.Core.Algorithms.PIDFOController;
import org.firstinspires.ftc.teamcode.Core.Algorithms.InterfaceSimple;
import org.firstinspires.ftc.teamcode.Core.Parts.Drivetrain;
import org.firstinspires.ftc.teamcode.Core.Parts.Shooter;

public class Robot implements InterfaceSimple {

   public PIDFOController PID = new PIDFOController();
   public Shooter shooter = new Shooter();
    public Drivetrain drivetrain = new Drivetrain();
    @Override
    public void update() {

        shooter.update();
        drivetrain.update();
    }
}
