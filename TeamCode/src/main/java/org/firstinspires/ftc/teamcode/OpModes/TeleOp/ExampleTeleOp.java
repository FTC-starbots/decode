package org.firstinspires.ftc.teamcode.OpModes.TeleOp;
import com.acmerobotics.roadrunner.Pose2d;
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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;


@TeleOp
public class ExampleTeleOp extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(105.19626168224299, 32.971962616822424, Math.toRadians(90)); //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
private Pose lastpose;
    private double rotation, lastheading, realheading, atualheading;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(lastheading), 0.8))
                .build();

    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        rotation = (gamepad1.left_trigger - gamepad1.right_trigger);



        follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, rotation*0.5, false);

        //This is how it looks with slowMode on
        if(rotation != 0) lastheading = Math.toDegrees(follower.getHeading());
        atualheading = Math.toDegrees(follower.getHeading()) % 360;
        if (atualheading < 0) atualheading += 360;

        //Automated PathFollowing
        telemetryM.debug("gamepad1.left_stick_y", gamepad1.left_stick_y);
        telemetryM.debug("gamepad1.left_stick_x", gamepad1.left_stick_x);
        telemetryM.debug("rotation", atualheading);


        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}