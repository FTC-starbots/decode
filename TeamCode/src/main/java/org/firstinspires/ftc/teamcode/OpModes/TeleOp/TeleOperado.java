package org.firstinspires.ftc.teamcode.OpModes.TeleOp;



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

import org.firstinspires.ftc.teamcode.Core.Parts.Drivetrain;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Core.Robot;

import java.util.function.Supplier;

public class TeleOperado extends OpMode{

    Pose startingPose = new Pose(0,0,0);
    Follower followerteleop;
    Drivetrain drivetrain;
    Robot robot;
     TelemetryManager telemetryM;

     HardwareMap hardwareMapteleop;


    @Override
    public void init() {
        drivetrain.StartFollower(followerteleop,startingPose,hardwareMapteleop);
    }

    @Override
    public void loop() {

    drivetrain.Drive(gamepad1, followerteleop, false, 0);
    robot.update();
    }
}
