package org.firstinspires.ftc.teamcode.OpModes.TeleOp;



import static org.firstinspires.ftc.teamcode.Core.Algorithms.PIDFOController.pidSystem.CorAng;

import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Algorithms.PIDFOController;
import org.firstinspires.ftc.teamcode.Core.Parts.Drivetrain;
import org.firstinspires.ftc.teamcode.Core.Robot;

import java.time.temporal.Temporal;
import java.util.function.Supplier;

public class TeleOperado extends OpMode{

    Pose startingPose = new Pose(0,0,0);
    Follower followerteleop;
    Drivetrain drivetrain;
    Robot robot;
     TelemetryManager telemetryM;
     PIDFOController PID;

     ElapsedTime Ciclo;

     double tempoCiclo;

    double CorrAng;

    public  VoltageSensor myControlHubVoltageSensor;

     HardwareMap hardwareMapteleop;


    @Override
    public void init() {
        drivetrain.StartFollower(followerteleop,startingPose,hardwareMapteleop);
    }


    public void start(){
        Ciclo = new ElapsedTime();
        Ciclo.reset();
    }

    @Override
    public void loop() {
        Ciclo.reset();

        CorrAng = PID.PIDF(CorAng,drivetrain.headingError,
            0.014 ,0.4, 1.3, 0.15,-0.5,0.5,
            PID.integral_CorAng,PID.lastheading_CorAng,
            tempoCiclo,myControlHubVoltageSensor);


        drivetrain.Drive(gamepad1, followerteleop, false, 0, CorrAng);

        robot.update();

        tempoCiclo = Ciclo.milliseconds();
    }


}
