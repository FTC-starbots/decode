package org.firstinspires.ftc.teamcode.OpModes.TeleOp;



import static org.firstinspires.ftc.teamcode.Core.Algorithms.PIDFOController.pidSystem.CorAng;


import com.pedropathing.control.PIDFCoefficients;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.Algorithms.PIDFOController;
import org.firstinspires.ftc.teamcode.Core.Parts.Drivetrain;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.robotcore.external.Telemetry;



import java.time.temporal.Temporal;
import java.util.function.Supplier;
@TeleOp
public class TeleOperado extends OpMode{

    Pose startingPose = new Pose(105.19626168224299,32.971962616822424, Math.toRadians(90));
    Follower followerteleop;

    Robot robot = new Robot();

    Drivetrain drivetrain = robot.drivetrain;
    TelemetryManager telemetryM;



     ElapsedTime Ciclo;

     double tempoCiclo;

    double CorrAng;

    double volts;

    public  VoltageSensor myControlHubVoltageSensor;

     HardwareMap hardwareMapteleop;




    @Override
    public void init() {
        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        followerteleop = Constants.createFollower(hardwareMap);
        followerteleop.setStartingPose(startingPose);
        followerteleop.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        followerteleop.startTeleopDrive();
    }


    public void start(){
        Ciclo = new ElapsedTime();
        Ciclo.reset();
        followerteleop.activateHeading();
    }

    @Override
    public void loop() {
        Ciclo.reset();

        drivetrain.Drive(gamepad1, followerteleop, false);

        robot.update();
        telemetry();
        tempoCiclo = Ciclo.milliseconds();
    }

public void telemetry(){
    telemetry.addData("Heading error", followerteleop.getHeadingError());
        telemetry.addData("Heading Vector", followerteleop.getTeleopHeadingVector());
    telemetry.addLine();
    telemetry.addData("a", followerteleop.getVectorCalculator().getTeleopHeadingVector());

    telemetry.update();

}
}
