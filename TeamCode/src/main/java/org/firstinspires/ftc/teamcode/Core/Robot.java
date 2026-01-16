package org.firstinspires.ftc.teamcode.Core;

import org.firstinspires.ftc.teamcode.Core.Algorithms.PIDFOController;
import org.firstinspires.ftc.teamcode.Core.Algorithms.InterfaceSimple;
import org.firstinspires.ftc.teamcode.Core.Hardware.AprilTagCam;
import org.firstinspires.ftc.teamcode.Core.Hardware.Lime;
import org.firstinspires.ftc.teamcode.Core.Parts.Drivetrain;
import org.firstinspires.ftc.teamcode.Core.Parts.Shooter;

public class Robot implements InterfaceSimple {
public Lime limelight = new Lime();
   public PIDFOController PID = new PIDFOController();
   public Shooter shooter = new Shooter();
    public Drivetrain drivetrain = new Drivetrain();

    public AprilTagCam aptag = new AprilTagCam();

    @Override
    public void update() {
        //limelight.update();
        //aptag.update();
        //shooter.update();
        drivetrain.update();
    }
}
