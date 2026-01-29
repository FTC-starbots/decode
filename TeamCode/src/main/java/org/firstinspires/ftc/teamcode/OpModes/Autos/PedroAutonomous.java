package org.firstinspires.ftc.teamcode.OpModes.Autos;



import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.teamcode.OpModes.TeleOp.TeleOperado.servoA;
import static org.firstinspires.ftc.teamcode.OpModes.TeleOp.TeleOperado.servoB;
import static org.firstinspires.ftc.teamcode.OpModes.TeleOp.TeleOperado.servoC;
import static org.firstinspires.ftc.teamcode.OpModes.TeleOp.TeleOperado.velA;
import static org.firstinspires.ftc.teamcode.OpModes.TeleOp.TeleOperado.velB;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.Algorithms.Launching;
import org.firstinspires.ftc.teamcode.Core.Algorithms.PoseIdentifier;
import org.firstinspires.ftc.teamcode.Core.Hardware.Lime;
import org.firstinspires.ftc.teamcode.Core.Parts.AimAssisters;
import org.firstinspires.ftc.teamcode.Core.Parts.Intake;
import org.firstinspires.ftc.teamcode.Core.Parts.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class


    public double servoIdeal, velIdeal;


    Shooter shooter = new Shooter();
    Intake intake = new Intake();
    PoseIdentifier poseidentifier = new PoseIdentifier();
     AimAssisters Capota = new AimAssisters();

     Launching launching = new Launching();

     Lime limelight = new Lime();

boolean toShoot;
int sequence;

int timesShoot;

public double id;
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(23.102803738317753, 124.48598130841118, Math.toRadians(324)));

        paths = new Paths(follower); // Build paths

        shooter.start(hardwareMap);
        intake.start(hardwareMap);
        Capota.start(hardwareMap);
        launching.start(hardwareMap);
        limelight.setup(hardwareMap);
        limelight.start();

        limelight.setPipeline(5);




        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {

        double d = poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.DISTANCE);

        servoIdeal = servoA * d * d - servoB * d + servoC;

        velIdeal = (int) (Math.round((velA * d + velB) / 20.0) * 20);

        Capota.setValue(servoIdeal);



        sequenceAnalyzer(id);
        launching.auto(0.5, sequence, toShoot);

        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path11;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.103, 124.486),

                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(324), Math.toRadians(317))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 96.000),

                                    new Pose(23.776, 106.093)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(317), Math.toRadians(-90))
                    .setReversed()
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.776, 106.093),

                                    new Pose(24.234, 76.467)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.234, 76.467),

                                    new Pose(13.009, 70.206)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(13.009, 70.206),

                                    new Pose(53.981, 89.729)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(317))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(53.981, 89.729),

                                    new Pose(24.121, 72.028)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(317), Math.toRadians(-90))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.121, 72.028),

                                    new Pose(23.879, 52.140)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.879, 52.140),

                                    new Pose(59.636, 83.364)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(317))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.636, 83.364),

                                    new Pose(24.000, 48.224)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(317), Math.toRadians(-90))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.000, 48.224),

                                    new Pose(23.579, 26.935)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.579, 26.935),

                                    new Pose(61.533, 81.458)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(317))

                    .build();
        }
    }



    public void sequenceAnalyzer(double id){
        if (id == 21){
            if (timesShoot == 0) sequence = 1;
            else if (timesShoot == 1) sequence = 1;
            else if(timesShoot == 2) sequence = 2;
            else if(timesShoot == 3) sequence = 3;


        } else if (id == 22) {
            if (timesShoot == 0) sequence = 2;
            else if (timesShoot == 1) sequence = 2;
            else if(timesShoot == 2) sequence = 1;
            else if(timesShoot == 3) sequence = 4;
        }else if (id == 23) {
            if (timesShoot == 0) sequence = 3;
            else if (timesShoot == 1) sequence = 3;
            else if(timesShoot == 2) sequence = 5;
            else if(timesShoot == 3) sequence = 1;

        }

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1, false);
                pathState = 1;

                break;
            case 1:

                if(!follower.isBusy()){
                    toShoot = true;
                    sleep(2001);
                    toShoot = false;
                        follower.followPath(paths.Path2, true);
                        pathState = 2;
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(paths.Path3, true);
                    pathState = 3;
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.STOP);
                    follower.followPath(paths.Path4, true);
                    pathState = 4;
                }
                break;
            case 4:
                if(!follower.isBusy()) {

                    follower.followPath(paths.Path5, false);
                    pathState = 5;
                }
                break;

            case 5:
                if(!follower.isBusy()) {
                    toShoot = true;
                    sleep(2001);
                    toShoot = false;
                    follower.followPath(paths.Path6, true);
                    pathState = 6;
                }
                break;

            case 6:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(paths.Path7, true);
                    pathState = 7;
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.STOP);
                    follower.followPath(paths.Path8, false);
                    pathState = 8;
                }
                break;

            case 8:

                if(!follower.isBusy()) {
                    toShoot = true;
                    sleep(2001);
                    toShoot = false;
                    follower.followPath(paths.Path9, true);
                    pathState = 9;
                }
                break;

            case 9:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(paths.Path10, true);
                    pathState = 10;
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.STOP);
                    follower.followPath(paths.Path11, false);
                    pathState = 11;
                }
                break;

            case 11:
            if(!follower.isBusy()) {
                toShoot = true;
                sleep(2001);
                toShoot = false;
                pathState = -1;
            }
            break;
        }
    }
}