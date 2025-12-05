package org.firstinspires.ftc.teamcode.OpModes.TeleOp;





import org.firstinspires.ftc.teamcode.Core.Algorithms.Launching;
import org.firstinspires.ftc.teamcode.Core.Hardware.AprilTag;
import org.firstinspires.ftc.teamcode.Core.Parts.Intake;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Parts.Drivetrain;
import org.firstinspires.ftc.teamcode.Core.Parts.Shooter;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Configurable

@TeleOp
public class TeleOperado extends OpMode{

    Pose startingPose = new Pose(105.19626168224299,32.971962616822424, Math.toRadians(90));
    Follower followerteleop;

    Robot robot = new Robot();

    Drivetrain drivetrain = robot.drivetrain;
    TelemetryManager telemetryM;

    TelemetryManager telemetryGraph;

Launching launching = new Launching();
public static double kp = 0.003, ki = 0.0000001, kd = 0.003, kf = 0.05;
     ElapsedTime Ciclo;

     double tempoCiclo;

    double CorrAng;

    double volts;

    public  VoltageSensor myControlHubVoltageSensor;

     Shooter Shooter = robot.shooter;

     Intake Intake = new Intake();

   //  AprilTag AprilTag = new AprilTag();





    @Override
    public void init() {
        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        followerteleop = Constants.createFollower(hardwareMap);
        followerteleop.setStartingPose(startingPose);
        followerteleop.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        followerteleop.startTeleopDrive();

        Intake.start(hardwareMap);
        Shooter.start(hardwareMap);



    }


    public void start(){
        Ciclo = new ElapsedTime();
        Ciclo.reset();
        Shooter.setCoefficients(kp,ki,kd,kf);
       // AprilTag.initAprilTag();
    }

    @Override
    public void loop() {
        Ciclo.reset();

       // AprilTag.AprilTagLocalization();

        launching.teleop(gamepad1);

        Shooter.teleop(gamepad1);
        Intake.teleop(gamepad1);
        drivetrain.Drive(gamepad1, followerteleop, false);

        robot.update();
        telemetry();
        tempoCiclo = Ciclo.milliseconds();
    }

public void telemetry(){


telemetryM.addData("Velocidade desejada" , Shooter.velocidade_desejada);
    telemetryM.addData("Velocidade atual" , Shooter.Atirador_1.getVelocity());
    telemetryM.addData("erro " , Shooter.erro);




    telemetry.addData("Heading error", followerteleop.getHeadingError());
    telemetry.addData("Heading Vector", followerteleop.getTeleopHeadingVector());
    telemetry.addLine();

    telemetryM.update(telemetry);
    telemetry.update();

}
}
