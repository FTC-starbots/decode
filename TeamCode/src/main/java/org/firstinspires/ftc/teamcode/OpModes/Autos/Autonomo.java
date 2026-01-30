package org.firstinspires.ftc.teamcode.OpModes.Autos;



import static android.os.SystemClock.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Algorithms.Drawing;
import org.firstinspires.ftc.teamcode.Core.Algorithms.Launching;
import org.firstinspires.ftc.teamcode.Core.Algorithms.PoseIdentifier;
import org.firstinspires.ftc.teamcode.Core.Hardware.Lime;
import org.firstinspires.ftc.teamcode.Core.Parts.AimAssisters;
import org.firstinspires.ftc.teamcode.Core.Parts.Intake;
import org.firstinspires.ftc.teamcode.Core.Parts.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Configurable // Panels
public class Autonomo extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    ElapsedTime timerShooter = new ElapsedTime();

    public double servoIdeal;
    public int velIdeal;


    Shooter shooter = new Shooter();
    Intake intake = new Intake();
    PoseIdentifier poseidentifier = new PoseIdentifier();
     AimAssisters Capota = new AimAssisters();

     Launching launching = new Launching();

     Lime limelight = new Lime();



boolean toShoot;
int sequence;

int timesShoot = 0;

public double id;

boolean timerReset;
boolean atrightvel;

int timesReset;
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(32.29906542056075, 134.5794392523364, Math.toRadians(-90)));

        paths = new Paths(follower); // Build paths

        shooter.start(hardwareMap);
        intake.start(hardwareMap);
        Capota.start(hardwareMap);
        launching.start(hardwareMap);
        limelight.setup(hardwareMap);
        limelight.start();

        limelight.setPipeline(5);

        shooter.setCoefficients(0.0002, 0.00009, 0, 0.05);

        id = 21;

        launching.reservatorio.resetServos();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {

        poseidentifier.infoPINPOINT(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());


        double d = poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.DISTANCE);

        servoIdeal = 0.000022 * d * d - 0.0079 * d + 1.55;

        velIdeal = (int) (Math.round((2.1 * d + 980) / 20.0) * 20);

        Capota.setValue(servoIdeal);

        if (toShoot) {
            shooter.isShooting(true);
            shooter.isPreShoot(false);
            shooter.setVelDesejada(velIdeal);
        } else {
            shooter.isShooting(false);
            shooter.isPreShoot(true);
            shooter.setVelDesejada(980);
        }


        if (timerReset && timesReset == 0) {
            timerShooter.reset();
            timesReset = 1;
        }


        if (!toShoot) launching.reservatorio.resetServos();

        launching.isShooting(shooter.shooting);
        launching.isAtRightVel(shooter.right_vel);
        sequenceAnalyzer(id);
        launching.auto(0.5, sequence, toShoot);


        shooter.update();
        limelight.update();


        Drawing.draw(follower);
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine
        {
            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            telemetry.addLine();
            telemetry.addData("RightVel", shooter.right_vel);
            telemetry.addData("ToShoot", toShoot);
            telemetry.addData("shooting", shooter.shooting);
            telemetry.addData("ToShoot", shooter.pre_shoot);


            telemetry.addLine();
            telemetry.addData("Velocidade desejada", shooter.velocidade_desejada);
            telemetry.addData("Velocidade atual", shooter.Atirador_1.getVelocity());
            telemetry.addData("shooter erro", shooter.erro);
            telemetry.addData("power ", shooter.Atirador_1.getPower());
            telemetry.addLine();
            panelsTelemetry.addData("Velocidade desejada", shooter.velocidade_desejada);
            panelsTelemetry.addData("Velocidade atual", shooter.Atirador_1.getVelocity());
            panelsTelemetry.addData("shooter erro", shooter.erro);
            panelsTelemetry.addData("power ", shooter.Atirador_1.getPower());

            panelsTelemetry.update(telemetry);
        }
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
                                    new Pose(32.299, 134.579),

                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(317))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 96.000),

                                    new Pose(23.776, 102.280)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(317), Math.toRadians(-90))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.776, 102.280),

                                    new Pose(23.785, 80.729)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.785, 80.729),

                                    new Pose(15.252, 75.364)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.252, 75.364),

                                    new Pose(53.981, 89.953)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(317))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(53.981, 89.953),

                                    new Pose(24.121, 72.028)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(317), Math.toRadians(-90))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.121, 72.028),

                                    new Pose(23.879, 56.178)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.879, 56.178),

                                    new Pose(59.636, 83.364)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(317))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.636, 83.364),

                                    new Pose(24.000, 48.224)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(317), Math.toRadians(-90))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.000, 48.224),

                                    new Pose(24.028, 30.972)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.028, 30.972),

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
                follower.followPath(paths.Path1,0.8, true);
                pathState = 1;

                break;
            case 1:

                if(!follower.isBusy()){
                 //   id = limelight.id;
                    toShoot = true;

                    if (launching.post_shoot) {
                        timerReset = false;
                        toShoot = false;
                        follower.followPath(paths.Path2,0.7, true);
                        pathState = 2;
                        timesShoot = 1;
                    }
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(paths.Path3,0.65, true);
                    pathState = 3;
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(paths.Path4,0.5, true);
                    pathState = 4;
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.STOP);
                    follower.followPath(paths.Path5,0.65, true);
                    pathState = 5;

                }
                break;

        }
    }
}