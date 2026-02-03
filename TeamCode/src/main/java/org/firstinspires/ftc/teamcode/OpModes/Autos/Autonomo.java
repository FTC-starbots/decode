package org.firstinspires.ftc.teamcode.OpModes.Autos;




import android.annotation.SuppressLint;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Algorithms.Drawing;
import org.firstinspires.ftc.teamcode.Core.Algorithms.Launching;
import org.firstinspires.ftc.teamcode.Core.Algorithms.PoseIdentifier;
import org.firstinspires.ftc.teamcode.Core.Algorithms.Selectors;
import org.firstinspires.ftc.teamcode.Core.Hardware.Lime;
import org.firstinspires.ftc.teamcode.Core.Parts.AimAssisters;
import org.firstinspires.ftc.teamcode.Core.Parts.Intake;
import org.firstinspires.ftc.teamcode.Core.Parts.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Configurable // Panels
public class Autonomo extends OpMode {

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
    public PathChain Path12;

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)

    ElapsedTime timerShooter = new ElapsedTime();

    public double servoIdeal;
    public int velIdeal;


    Shooter shooter = new Shooter();
    Intake intake = new Intake();
    PoseIdentifier poseidentifier = new PoseIdentifier();
     AimAssisters Capota = new AimAssisters();

     Launching launching = new Launching();

     Lime limelight = new Lime();

    Selectors seletores = new Selectors();


boolean toShoot = false;
int sequence = 1;

int timesShoot = 0;

public double id = 21;

boolean timerReset;

int timesReset;
    boolean manualSelectAuto = false, manualSelectAlliance = false,isBlue;

    int auto;
    @SuppressLint("SuspiciousIndentation")
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);


        shooter.start(hardwareMap);
        intake.start(hardwareMap);
        Capota.start(hardwareMap);
        launching.start(hardwareMap);
        limelight.setup(hardwareMap);
        seletores.start(hardwareMap);

        limelight.start();

        limelight.setPipeline(5);

        shooter.setCoefficients(0.0004, 0, 0, 0.05);

        id = 21;




        launching.reservatorio.resetServos();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }


    @Override
    public void init_loop(){


        if (!manualSelectAuto){
            auto = seletores.getAuto();
        }

        if (gamepad2.aWasPressed() || gamepad2.crossWasPressed()) {
            manualSelectAuto = true;
            auto =1;
        }
        else if (gamepad2.bWasPressed() || gamepad2.circleWasPressed()){
            manualSelectAuto = true;
            auto =2;
        }
        else if (gamepad2.yWasPressed() || gamepad2.triangleWasPressed()){
            manualSelectAuto = true;
            auto =3;
        }
        else if (gamepad2.xWasPressed() || gamepad2.squareWasPressed()) {
            manualSelectAuto = true;
            auto = 4;
        }



        if (!manualSelectAlliance){
            isBlue = seletores.isBlueAlliance();
        }

        if (gamepad2.dpadLeftWasPressed()) {
            manualSelectAlliance = true;
            isBlue = true;
        }
        else if (gamepad2.dpadRightWasPressed()){
            manualSelectAlliance = true;
            isBlue = false;
        }

        telemetry.addData("auto", auto);
        telemetry.addData("isBlue", isBlue);
        telemetry.update();


    }


    @Override
    public void start(){

        if (auto == 1 || auto == 2){
            if (isBlue) {
                follower.setStartingPose(new Pose(32.29906542056075, 134.5794392523364, Math.toRadians(-90)));
            }else {
                follower.setStartingPose(new Pose(111.70093457943925, 134.5794392523364, Math.toRadians(270)));
            }
        }
        else if (auto == 3 || auto == 4){
            if (isBlue) {
                follower.setStartingPose(new Pose(55.55140186915888, 8.448598130841138, Math.toRadians(-90)));
            }else {
                follower.setStartingPose(new Pose(88.44859813084112, 8.448598130841138, Math.toRadians(270)));
            }
        }


        if (isBlue) {

            if (auto == 1){
                build12GateBLUE();
            }
            else if (auto == 2){
                build12GateBLUE();
            }
            else if (auto == 3){
                buildSmall_Human_PlayerBLUE();
            }
            else if (auto == 4){
                buildSmall_Human_PlayerBLUE();
            }

        }else {
            if (auto == 1){
                build12GateRED();
            }
            else if (auto == 2){
                build12GateRED();
            }
            else if (auto == 3) {
                buildSmall_Human_PlayerRED();
            }
            else if (auto == 4) {
                buildSmall_Human_PlayerRED();
            }
        }
    }


    @Override
    public void loop() {
        {
            poseidentifier.infoPINPOINT(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());


            double d = poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.DISTANCE);

            servoIdeal = 0.000022 * d * d - 0.0079 * d + 1.55;


            if (auto == 1 || auto == 2){
                velIdeal = (int) (Math.round((2.1 * d + 940) / 20.0) * 20);
            }
            else if (auto == 3 || auto == 4){
                velIdeal = (int) (Math.round((2.1 * d + 980) / 20.0) * 20);
            }


            Capota.setValue(servoIdeal);

            if (toShoot) {
                shooter.isShooting(true);
                shooter.isPreShoot(false);
                shooter.setVelDesejada(velIdeal);
            } else {
                shooter.isShooting(false);
                shooter.isPreShoot(true);
                shooter.setVelDesejada(940);
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
        }

        Drawing.draw(follower);
        follower.update();

        if (auto == 1) Update12GatePath();
        else if (auto == 2) Update9GatePath();
        else if (auto == 3) UpdateSmall_Human_Player();
        else if (auto == 4)UpdateJUST_Human_Player();

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
            telemetry.addData("pre", shooter.pre_shoot);
            telemetry.addData("pos", launching.post_shoot);



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

    public void sequenceAnalyzer(double id){
       if ((auto == 1 || auto == 2 && isBlue) || (auto == 3 || auto == 4 && !isBlue)) {

           if (id == 21) {
               if (timesShoot == 0) sequence = 1;
               else if (timesShoot == 1) sequence = 1;
               else if (timesShoot == 2) sequence = 2;
               else if (timesShoot == 3) sequence = 3;
               else sequence = 1;


           } else if (id == 22) {
               if (timesShoot == 0) sequence = 2;
               else if (timesShoot == 1) sequence = 2;
               else if (timesShoot == 2) sequence = 1;
               else if (timesShoot == 3) sequence = 4;
               else sequence = 1;
           } else if (id == 23) {
               if (timesShoot == 0) sequence = 3;
               else if (timesShoot == 1) sequence = 3;
               else if (timesShoot == 2) sequence = 5;
               else if (timesShoot == 3) sequence = 1;
               else sequence = 1;
           }
       } else if (((auto == 3 || auto == 4 && isBlue) || (auto == 1 || auto == 2 && !isBlue))) {
           if (id == 21) {
               if (timesShoot == 0) sequence = 1;
               else if (timesShoot == 1) sequence = 3;
               else if (timesShoot == 2) sequence = 2;
               else if (timesShoot == 3) sequence = 1;
               else sequence = 1;


           } else if (id == 22) {
               if (timesShoot == 0) sequence = 2;
               else if (timesShoot == 1) sequence = 4;
               else if (timesShoot == 2) sequence = 1;
               else if (timesShoot == 3) sequence = 2;
               else sequence = 1;
           } else if (id == 23) {
               if (timesShoot == 0) sequence = 3;
               else if (timesShoot == 1) sequence = 1;
               else if (timesShoot == 2) sequence = 5;
               else if (timesShoot == 3) sequence = 3;
               else sequence = 1;
           }
       }
    }

    public void build12GateBLUE() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(32.299, 134.579),
                                new Pose(34.467, 119.009),
                                new Pose(57.421, 100.299)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(323))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57.421, 100.299),

                                new Pose(25.570, 102.505)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(317), Math.toRadians(-90))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.570, 102.505),

                                new Pose(25.579, 85.215)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.579, 85.215),

                                new Pose(15.252, 76.935)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.252, 76.935),

                                new Pose(52.860, 90.402)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(317))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(52.860, 90.402),

                                new Pose(25.243, 74.943)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(317), Math.toRadians(-90))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.243, 74.943),

                                new Pose(25.224, 63.355)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.224, 63.355),

                                new Pose(52.907, 90.318)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(317))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(52.907, 90.318),

                                new Pose(25.346, 50.692)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(317), Math.toRadians(-90))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.346, 50.692),

                                new Pose(25.150, 38.374)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.150, 38.374),

                                new Pose(57.421, 100.037)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(323))

                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.224, 63.355),

                                new Pose(57.421, 100.037)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(323))

                .build();
        }
    public void build12GateRED() {
        // ========== PATH 1 ==========
        Path1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(111.701, 134.579),
                        new Pose(109.533, 119.009),
                        new Pose(86.579, 100.299)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(270),
                Math.toRadians(-143)
        ).build();


// ========== PATH 2 ==========
        Path2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(86.579, 100.299),
                        new Pose(118.430, 102.505)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(-137),
                Math.toRadians(270)
        ).build();


// ========== PATH 3 ==========
        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(118.430, 102.505),
                                new Pose(118.421, 85.215)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(270))
                .build();


// ========== PATH 4 ==========
        Path4 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(118.421, 85.215),
                        new Pose(128.748, 76.935)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(270),
                Math.toRadians(270)
        ).build();


// ========== PATH 5 ==========
        Path5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(128.748, 76.935),
                        new Pose(91.140, 90.402)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(270),
                Math.toRadians(-137)
        ).build();


// ========== PATH 6 ==========
        Path6 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(91.140, 90.402),
                        new Pose(118.757, 74.943)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(-137),
                Math.toRadians(270)
        ).build();


// ========== PATH 7 ==========
        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(118.757, 74.943),
                                new Pose(118.776, 63.355)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(270))
                .build();


// ========== PATH 8 ==========
        Path8 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(118.776, 63.355),
                        new Pose(91.093, 90.318)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(270),
                Math.toRadians(-137)
        ).build();


// ========== PATH 9 ==========
        Path9 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(91.093, 90.318),
                        new Pose(118.654, 50.692)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(-137),
                Math.toRadians(270)
        ).build();


// ========== PATH 10 ==========
        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(118.654, 50.692),
                                new Pose(118.850, 38.374)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(270))
                .build();


// ========== PATH 11 ==========
        Path11 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(118.850, 38.374),
                        new Pose(86.579, 100.037)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(270),
                Math.toRadians(-143)
        ).build();


// ========== PATH 12 ==========
        Path12 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(118.776, 63.355),
                        new Pose(86.579, 100.037)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(270),
                Math.toRadians(-143)
        ).build();

    }



    public void buildSmall_Human_PlayerBLUE(){
        Path1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.551, 8.449),
                                new Pose(54.927, 12.215),
                                new Pose(58.900, 10.813),
                                new Pose(56.075, 18.168)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.075, 18.168),

                                new Pose(25.346, 19.290)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(293), Math.toRadians(90))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.346, 19.290),

                                new Pose(25.486, 33.776)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.486, 33.776),

                                new Pose(55.897, 18.308)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(293))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.897, 18.308),

                                new Pose(34.963, 9.467)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(293), Math.toRadians(180))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(34.963, 9.467),

                                new Pose(15.533, 7.234)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.533, 7.234),

                                new Pose(55.972, 18.486)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(293))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.972, 18.486),

                                new Pose(34.944, 9.150)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(293), Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(34.944, 9.150),

                                new Pose(19.738, 7.682)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(19.738, 7.682),

                                new Pose(35.888, 7.981)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
    }

    public void buildSmall_Human_PlayerRED(){
        // ========== PATH 1 ==========
        Path1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(88.449, 8.449),
                                new Pose(89.073, 12.215),
                                new Pose(85.100, 10.813),
                                new Pose(87.925, 18.168)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();


// ========== PATH 2 ==========
        Path2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(87.925, 18.168),
                        new Pose(118.654, 19.290)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(-113),
                Math.toRadians(90)
        ).build();


// ========== PATH 3 ==========
        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(118.654, 19.290),
                                new Pose(118.514, 33.776)
                        )
                ).setTangentHeadingInterpolation()
                .build();


// ========== PATH 4 ==========
        Path4 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(118.514, 33.776),
                        new Pose(88.103, 18.308)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(90),
                Math.toRadians(-113)
        ).build();


// ========== PATH 5 ==========
        Path5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(88.103, 18.308),
                        new Pose(109.037, 9.467)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(-113),
                Math.toRadians(0)
        ).build();


// ========== PATH 6 ==========
        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(109.037, 9.467),
                                new Pose(128.467, 7.234)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();


// ========== PATH 7 ==========
        Path7 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(128.467, 7.234),
                        new Pose(88.028, 18.486)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(0),
                Math.toRadians(-113)
        ).build();


// ========== PATH 8 ==========
        Path8 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(88.028, 18.486),
                        new Pose(109.056, 9.150)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(-113),
                Math.toRadians(0)
        ).build();


// ========== PATH 9 ==========
        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(109.056, 9.150),
                                new Pose(124.262, 7.682)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();


// ========== PATH 10 ==========
        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.262, 7.682),
                                new Pose(108.112, 7.981)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

    }

//=================================================================================================
    public void Update12GatePath() {
        switch (pathState) {
            case 0:
                follower.followPath(Path1,0.8, true);
                pathState = 1;
                timesShoot = 0;
                break;
            case 1:

                if(!follower.isBusy()){
                    toShoot = true;

                    if (launching.post_shoot) {
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        follower.followPath(Path2,0.9, true);
                        pathState = 2;
                        timesShoot = 1;
                    }
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(Path3,0.85, true);
                    pathState = 3;
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.STOP);
                    follower.followPath(Path4,0.75, true);
                    pathState = 4;
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(Path5,0.9, true);
                    pathState = 5;
                    timesShoot = 0;

                }
                break;
            case 5:

                if(!follower.isBusy()){

                    toShoot = true;

                    if (launching.post_shoot) {
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        follower.followPath(Path6,0.9, true);
                        pathState = 6;
                        timesShoot = 1;
                    }
                }
                break;

            case 6:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(Path7,0.85, true);
                    pathState = 7;
                }
            case 7:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.STOP);
                    follower.followPath(Path8,0.9, true);
                    pathState = 8;
                }
                break;

            case 8:

                if(!follower.isBusy()){
                    toShoot = true;

                    if (launching.post_shoot) {
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        follower.followPath(Path9,0.9, true);
                        pathState = 9;
                        timesShoot = 1;
                    }
                }
                break;

            case 9:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(Path10,0.85, true);
                    pathState = 10;
                }
            case 10:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.STOP);
                    follower.followPath(Path11,0.9, true);
                    pathState = 11;
                }
                break;
            case 11:
                if(!follower.isBusy()){
                    toShoot = true;

                    if (launching.post_shoot) {
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        pathState = -1;
                        timesShoot = 1;
                    }
                }
                break;
        }
    }
    public void Update9GatePath() {
        switch (pathState) {
            case 0:
                follower.followPath(Path1,0.8, true);
                pathState = 1;
                timesShoot = 0;
                break;
            case 1:

                if(!follower.isBusy()){
                    toShoot = true;

                    if (launching.post_shoot) {
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        follower.followPath(Path2,0.9, true);
                        pathState = 2;
                        timesShoot = 1;
                    }
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(Path3,0.85, true);
                    pathState = 3;
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.STOP);
                    follower.followPath(Path4,0.75, true);
                    pathState = 4;
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(Path5,0.9, true);
                    pathState = 5;
                    timesShoot = 0;

                }
                break;
            case 5:

                if(!follower.isBusy()){

                    toShoot = true;

                    if (launching.post_shoot) {
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        follower.followPath(Path6,0.9, true);
                        pathState = 6;
                        timesShoot = 1;
                    }
                }
                break;

            case 6:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(Path7,0.85, true);
                    pathState = 7;
                }
            case 7:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.STOP);
                    follower.followPath(Path12,0.9, true);
                    pathState = 8;
                }
                break;

            case 8:

                if(!follower.isBusy()){
                    toShoot = true;

                    if (launching.post_shoot) {
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        pathState = -1;
                        timesShoot = 1;
                    }
                }
                break;



        }
    }


    public void UpdateSmall_Human_Player() {
        switch (pathState) {
            case 0:
                follower.followPath(Path1,0.75, true);
                pathState = 1;
                timesShoot = 0;
                break;
            case 1:

                if(!follower.isBusy()){
                    toShoot = true;

                    if (launching.post_shoot) {
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        follower.followPath(Path2,0.9, true);
                        pathState = 2;
                        timesShoot = 1;
                    }
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(Path3,0.85, true);
                    pathState = 3;
                }
                break;
            case 3:
                if(!follower.isBusy()) {

                    follower.followPath(Path4,0.75, true);
                    pathState = 4;
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.STOP);
                    toShoot = true;

                    if (launching.post_shoot) {
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        follower.followPath(Path5,0.9, true);
                        pathState = 5;
                        timesShoot = 1;
                    }

                }
                break;
            case 5:

                if(!follower.isBusy()){
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(Path6,0.6, true);
                    pathState = 6;

                }
                break;

            case 6:
                if(!follower.isBusy()) {

                    follower.followPath(Path7,0.8, true);
                    pathState = 7;
                }
            case 7:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.STOP);
                    toShoot = true;

                    if (launching.post_shoot) {
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        follower.followPath(Path8,0.9, true);
                        pathState = 8;
                        timesShoot = 1;
                    }
                }
                break;

            case 8:

                if(!follower.isBusy()){
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(Path9,1, true);
                    pathState = 9;

                }
                break;

            case 9:
                if(!follower.isBusy()) {

                    follower.followPath(Path10,0.8, true);
                    pathState = 10;
                }
            case 10:
                if(!follower.isBusy()){
                    intake.setMode(Intake.Mode.STOP);
                    pathState = -1;
                    timesShoot = 0;
                }
                break;
        }
    }

    public void UpdateJUST_Human_Player() {
        switch (pathState) {
            case 0:
                follower.followPath(Path1,0.75, true);
                pathState = 1;
                timesShoot = 0;
                break;
            case 1:

                if(!follower.isBusy()){
                    toShoot = true;

                    if (launching.post_shoot) {
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        follower.followPath(Path2,0.9, true);
                        pathState = 2;
                        timesShoot = 1;
                    }
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(Path3,0.85, true);
                    pathState = 3;
                }
                break;
            case 3:
                if(!follower.isBusy()) {

                    follower.followPath(Path4,0.75, true);
                    pathState = 4;
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.STOP);
                    toShoot = true;

                    if (launching.post_shoot) {
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        follower.followPath(Path5,0.9, true);
                        pathState = 5;
                        timesShoot = 1;
                    }

                }
                break;
            case 5:

                if(!follower.isBusy()){
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(Path6,0.6, true);
                    pathState = 6;

                }
                break;

            case 6:
                if(!follower.isBusy()) {

                    follower.followPath(Path7,0.8, true);
                    pathState = 7;
                }
            case 7:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.STOP);
                    toShoot = true;

                    if (launching.post_shoot) {
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        follower.followPath(Path8,0.9, true);
                        pathState = 8;
                        timesShoot = 1;
                    }
                }
                break;

            case 8:

                if(!follower.isBusy()){
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(Path9,1, true);
                    pathState = 9;

                }
                break;

            case 9:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.STOP);
                    toShoot = true;

                    if (launching.post_shoot) {
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        follower.followPath(Path8,0.9, true);
                        pathState = 10;
                        timesShoot = 1;
                    }
                }
                break;

            case 10:

                if(!follower.isBusy()){
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(Path9,1, true);
                    pathState = 11;

                }
                break;
            case 11:
                if(!follower.isBusy()) {

                    follower.followPath(Path10,0.8, true);
                    pathState = 12;
                }
            case 12:
                if(!follower.isBusy()){
                    intake.setMode(Intake.Mode.STOP);
                    pathState = -1;
                    timesShoot = 0;
                }
                break;
        }
    }




}