package org.firstinspires.ftc.teamcode.OpModes.Autos;




import static android.os.SystemClock.sleep;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Algorithms.Drawing;
import org.firstinspires.ftc.teamcode.Core.Algorithms.Launching;
import org.firstinspires.ftc.teamcode.Core.Algorithms.PoseIdentifier;
import org.firstinspires.ftc.teamcode.Core.Algorithms.SequenceAlgorithm;
import org.firstinspires.ftc.teamcode.Core.Algorithms.ShooterControl;
import org.firstinspires.ftc.teamcode.Core.Hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Core.Hardware.Leds;
import org.firstinspires.ftc.teamcode.Core.Hardware.Lime;
import org.firstinspires.ftc.teamcode.Core.Parts.AimAssisters;
import org.firstinspires.ftc.teamcode.Core.Parts.Intake;
import org.firstinspires.ftc.teamcode.Core.Parts.LimeServo;
import org.firstinspires.ftc.teamcode.Core.Parts.Reservoirs;
import org.firstinspires.ftc.teamcode.Core.Parts.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(preselectTeleOp = "TeleOperado")
@Configurable // Panels
public class Autonomo extends OpMode {

    public PathChain Path1;
    public PathChain Path1a;
    public PathChain Path2;
    public PathChain Path2a;
    public PathChain Path3;
    public PathChain Path3a;
    public PathChain Path11;
    public PathChain Path11a;
    public PathChain Path4;
    public PathChain Path4a;
    public PathChain Path5;
    public PathChain Path5a;
    public PathChain Path6;
    public PathChain Path6a;
    public PathChain Path7;
    public PathChain Path7a;
    public PathChain Path8;
    public PathChain Path8a;
    public PathChain Path9;
    public PathChain Path9a;
    public PathChain Path10;
    public PathChain Path10a;
    public PathChain Path12;
    public PathChain Path12a;
    public PathChain Path13;
    public PathChain Path13a;

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)

    ElapsedTime timerShooter = new ElapsedTime();

    public double servoIdeal;
    public int velIdeal;

    int setGateOrder = 0;
    Shooter shooter = new Shooter();
    public Intake intake = new Intake();
    PoseIdentifier poseidentifier = new PoseIdentifier();
    AimAssisters Capota = new AimAssisters();

    Launching launching = new Launching();
    LimeServo servolime = new LimeServo();
    Lime limelight = servolime.limelight;

    Leds LED = new Leds();
    ElapsedTime ejectTimer = new ElapsedTime();

   ColorSensor colorSensor = new ColorSensor();
    ShooterControl ControleShooter = new ShooterControl();
    SequenceAlgorithm algoritmoSequencia = new SequenceAlgorithm();

     double kpShootera = 0.0017, kiShootera = 0.000125, kdShootera = 0, kfShootera = 0.05;

    boolean toShoot = false;
int sequence = 1;
int angleCor;
int timesShoot = 0;

public static double id = 21;

boolean timerReset;

int shoot = 0;

int timesReset;
boolean isReservoirsFull;
    boolean manualSelectAuto = false, manualSelectAlliance = false;

    public static boolean isBlue;

    public static Pose finalPose = new Pose(0, 0, 0);

    public static int auto;
    public static ColorSensor.BALL[] gateOrder = new ColorSensor.BALL[9];
    @SuppressLint("SuspiciousIndentation")
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        colorSensor.start(hardwareMap);
        shooter.start(hardwareMap);
        intake.start(hardwareMap);
        Capota.start(hardwareMap);
        launching.start(hardwareMap);
        limelight.setup(hardwareMap);
        LED.start(hardwareMap);


        limelight.start();

        limelight.setPipeline(5);


        servolime.start(hardwareMap);
        shooter.setCoefficients(kpShootera, kiShootera, kdShootera, kfShootera);

        id = 0;




        launching.reservatorio.resetServos();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }


    @Override
    public void init_loop(){


        if (!manualSelectAuto){
            auto = 1;
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
        }else if (gamepad2.leftBumperWasPressed()) {
            manualSelectAuto = true;
            auto = 5;
        }



        if (!manualSelectAlliance){
            isBlue = true;
        }

        if (gamepad2.dpadLeftWasPressed()) {
            manualSelectAlliance = true;
            isBlue = true;
        }
        else if (gamepad2.dpadRightWasPressed()){
            manualSelectAlliance = true;
            isBlue = false;
        }

        follower.update();
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("auto", auto);
        telemetry.addData("isBlue", isBlue);
        telemetry.update();


    }


    @Override
    public void start(){
shooter.isAuto(true, auto);
        servolime.updateDesiredReference(LimeServo.Mode.OBELISK);
        if (auto == 1 || auto == 2){
            if (isBlue) {
                follower.setStartingPose(new Pose(32.299, 134.579, Math.toRadians(-90)));
            }else {
                follower.setStartingPose(new Pose(111.70093457943925, 134.5794392523364, Math.toRadians(270)));
            }
        }
        else if (auto == 3 || auto == 4){
            if (isBlue) {
                follower.setStartingPose(new Pose(55.55140186915888, 10.448598130841138, Math.toRadians(-90)));
            }else {
                follower.setStartingPose(new Pose(88.44859813084112, 10.448598130841138, Math.toRadians(270)));
            }
        }else{
            if (isBlue) {
                follower.setStartingPose(new Pose(55.55140186915888, 10.448598130841138, Math.toRadians(-90)));
            }else {
                follower.setStartingPose(new Pose(88.44859813084112, 10.448598130841138, Math.toRadians(270)));
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
            if(intake.IntakeMode == Intake.Mode.INTAKE)colorSensor.readPresence(ejectTimer);
            if (colorSensor.ball1 != ColorSensor.BALL.NONE && colorSensor.ball2 != ColorSensor.BALL.NONE && colorSensor.ball3!= ColorSensor.BALL.NONE) isReservoirsFull =true;
            else isReservoirsFull =false;


            if (intake.IntakeMode == Intake.Mode.STOP ||intake.IntakeMode == Intake.Mode.EJECT) ejectTimer.reset();


            if (auto ==1 || auto == 2) {
                if (pathState >1 && pathState < 6) {
                    if (ejectTimer.seconds() > 1.75 && ejectTimer.seconds() < 2.75) intake.setMode(Intake.Mode.EJECT);
                    else if (ejectTimer.seconds() > 2.75) intake.setMode(Intake.Mode.STOP);
                }else if (pathState >6 && pathState < 9) {
                    if (ejectTimer.seconds() > 2.75 && ejectTimer.seconds() < 3.75) intake.setMode(Intake.Mode.EJECT);
                    else if (ejectTimer.seconds() > 2.75) intake.setMode(Intake.Mode.STOP);
                }else if (pathState > 9) {
                    if (ejectTimer.seconds() > 3.2 && ejectTimer.seconds() < 4.2) intake.setMode(Intake.Mode.EJECT);
                    else if (ejectTimer.seconds() > 2.75) intake.setMode(Intake.Mode.STOP);
                }

                }else {
                if (ejectTimer.seconds() > 2 && ejectTimer.seconds() < 2.85) intake.setMode(Intake.Mode.EJECT);
                else if (ejectTimer.seconds() > 2.85 || shooter.shooting) intake.setMode(Intake.Mode.STOP);
            }
            if (!shooter.shooting) {
                launching.reservatorio.servoPosition(launching.reservatorio.Reservoir_1, Reservoirs.ReservoirsState.DOWN);
                launching.reservatorio.servoPosition(launching.reservatorio.Reservoir_2, Reservoirs.ReservoirsState.DOWN);
                launching.reservatorio.servoPosition(launching.reservatorio.Reservoir_3, Reservoirs.ReservoirsState.DOWN);
            }

            if (id == 21){
                LED.Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (id == 22) {
                LED.Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else if (id == 23) {
                LED.Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

            }

            ControleShooter.isBlue(isBlue);
            poseidentifier.infoPINPOINT(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());

            servolime.setCurrentPosition(poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.X), poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.Y), poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.YAW));

            poseidentifier.isblue(isBlue);


            double d = poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.DISTANCE);

            ControleShooter.setDistance(d);

            if (d < 90){
                shooter.setMargin(40,40);
                ControleShooter.setVel(3.9,800, 160);
                if (isBlue)ControleShooter.setServo(0.0098,1.5);
                else ControleShooter.setServo(0.0093,1.54);

            }
            else{
                shooter.setMargin(60,40);
                /*if (isBlue)*/ ControleShooter.setVel(3.7,860, 160);
                //else ControleShooter.setVel(3.35,885, 160);
                ControleShooter.setServo(0.0093,1.2);
            }


            shooter.setlimeInfo(limelight.tpixel, limelight.id, limelight.tagdetected);
            shooter.isBlue(isBlue);
            if (auto == 3 || auto ==4){
                velIdeal = Math.min(1340, ControleShooter.getVel());
            }else{
                velIdeal = Math.min(1020, ControleShooter.getVel());
            }
            servoIdeal = ControleShooter.getServo();

            ControleShooter.update();

            Capota.IntakeMode(intake.IntakeMode);
            Capota.setValue(servoIdeal);

            if (toShoot) {
                shooter.isShooting(true);
                shooter.isPreShoot(false);
                shooter.setVelDesejada(velIdeal);
            } else {
                shooter.isShooting(false);
                shooter.isPreShoot(true);
                shooter.setVelDesejada(velIdeal);
            }


            if (timerReset && timesReset == 0) {
                timerShooter.reset();
                timesReset = 1;
            }


            if (!toShoot) launching.reservatorio.resetServos();

            launching.isShooting(shooter.shooting);
            launching.isAtRightVel(shooter.right_vel);
            sequenceAnalyzer(id);
           // algoritmoSequencia.setMOTIF(id);
            launching.auto(0.45, sequence, toShoot);

            if ((id == 21 || id ==22 || id==23) && setGateOrder < 3 ){
               // gateOrder = algoritmoSequencia.gateOrder();
                setGateOrder++;
            }

            if (auto == 3 || auto == 4){
                if (pathState > 0) servolime.setPosition(0.5);
                else servolime.update();
            }else {
                servolime.update();
            }
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


            telemetry.addData("ID", id);
            telemetry.addData("SEQUENCE", sequence);
            telemetry.addLine();
            telemetry.addData("TX", limelight.tpixel);
            telemetry.addLine();
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

    @Override
    public void stop(){
        finalPose = follower.getPose();
    }
    public void sequenceAnalyzer(double id){
       if ((auto == 1 && isBlue)|| (auto == 2 && isBlue) || (auto == 3 && !isBlue) || (auto == 4 && !isBlue)) {

           if (id == 21) {
               if (timesShoot == 0) sequence = 1;
               else if (timesShoot == 1) sequence = 1;
               else if (timesShoot == 2) sequence = 2;
               else if (timesShoot == 3) sequence = 3;
               else sequence = 1;


           } else if (id == 22) {
               if (timesShoot == 0) sequence = 1;
               else if (timesShoot == 1) sequence = 2;
               else if (timesShoot == 2) sequence = 1;
               else if (timesShoot == 3) sequence = 4;
               else sequence = 1;
           } else {
               if (timesShoot == 0) sequence = 1;
               else if (timesShoot == 1) sequence = 3;
               else if (timesShoot == 2) sequence = 5;
               else if (timesShoot == 3) sequence = 1;
               else sequence = 1;
           }
       } else if (((auto == 3 && isBlue)|| (auto == 4 && isBlue) || (auto == 1 && !isBlue)|| (auto == 2 && !isBlue))) {
           if (id == 21) {
               if (timesShoot == 0) sequence = 1;
               else if (timesShoot == 1) sequence = 3;
               else if (timesShoot == 2) sequence = 2;
               else if (timesShoot == 3) sequence = 1;
               else sequence = 1;


           } else if (id == 22) {
               if (timesShoot == 0) sequence = 1;
               else if (timesShoot == 1) sequence = 4;
               else if (timesShoot == 2) sequence = 1;
               else if (timesShoot == 3) sequence = 2;
               else sequence = 1;
           } else {
               if (timesShoot == 0) sequence = 1;
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
                                new Pose(32.748, 135.476),
                                new Pose(31.401, 127.981),
                                new Pose(36.859, 117.350)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(335))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(36.859, 117.350),

                                new Pose(24.5, 105.645)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(335), Math.toRadians(-90))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.5, 105.645),

                                new Pose(24.5, 83.756)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.5, 83.756),

                                new Pose(16.803, 77.692)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(14.803, 74.692),

                                new Pose(37.308, 118.068)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(339))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(37.308, 118.068),

                                new Pose(24.5, 105.672)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(-90))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.5, 105.672),

                                new Pose(24.5, 61.233)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.5, 61.233),

                                new Pose(37.305, 118.034)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(340))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(37.305, 118.034),

                                new Pose(24.5, 105.421)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(-90))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.5, 105.421),

                                new Pose(24.5, 37.796)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.5, 37.796),

                                new Pose(39.276, 118.356)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(340))

                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.5, 63.355),

                                new Pose(57.421, 100.037)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(340))

                .build();
        }
    public void build12GateRED() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(111.252, 135.476),
                                new Pose(112.599, 127.981),
                                new Pose(107.141, 117.350)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-160))
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(107.141, 117.350),
                                new Pose(120.776, 105.645)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-160), Math.toRadians(-90))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(120.776, 105.645),
                                new Pose(120.776, 83.756)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(120.776, 83.756),
                                new Pose(128.197, 77.692)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.197, 74.692),
                                new Pose(106.692, 118.068)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-159))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(106.692, 118.068),
                                new Pose(119.5, 105.672)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-160), Math.toRadians(-90))
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(119.5, 105.672),
                                new Pose(119.5, 61.233)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(119.5, 61.233),
                                new Pose(106.695, 118.034)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-160))
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(106.695, 118.034),
                                new Pose(119.5, 105.421)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-160), Math.toRadians(-90))
                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(119.5, 105.421),
                                new Pose(119.5, 37.796)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(119.5, 37.796),
                                new Pose(104.724, 118.356)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-160))
                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(119.5 , 63.355),
                                new Pose(86.579, 100.037)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-160))
                .build();

    }



    public void buildSmall_Human_PlayerBLUE(){
        Path1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.551, 8.449),
                                new Pose(57.009, 13.972),
                                new Pose(56.299, 17.944)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(289))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.299, 19.5),

                                new Pose(25.346, 17)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(289), Math.toRadians(90))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.346, 17),

                                new Pose(25.486, 33.776)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.486, 33.776),

                                new Pose(56.299, 17.944)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(291))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.299, 19.5),

                                new Pose(34.963, 9.467)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(291), Math.toRadians(180))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(34.963, 9.467),

                                new Pose(13.066, 6.785)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.066, 6.785),

                                new Pose(56.299, 17.944)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(294))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.299, 19.5),

                                new Pose(34.944, 9.150)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(294), Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(34.944, 9.150),

                                new Pose(14, 7.233)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.785, 7.233),

                                new Pose(56.299, 17.944)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(294))

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.299, 19.5),

                                new Pose(34.925, 8.963)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(294), Math.toRadians(180))

                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(34.925, 8.963),

                                new Pose(13.009, 7.196)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.009, 7.196),

                                new Pose(56.299, 19.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(294))
                .build();

        Path1a = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.299, 17.944),

                                new Pose(56.299, 19.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(289), Math.toRadians(289))

                .build();

        Path4a = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.299, 17.944),

                                new Pose(56.299, 19.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(291), Math.toRadians(291))

                .build();
        Path7a = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.299, 17.944),

                                new Pose(56.299, 19.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(294), Math.toRadians(294))

                .build();
        Path10a = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.299, 17.944),

                                new Pose(56.299, 19.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(294), Math.toRadians(294))

                .build();

    }

    public void buildSmall_Human_PlayerRED(){
        Path1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(88.449, 8.449),
                                new Pose(86.991, 13.972),
                                new Pose(87.701, 17.944)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-110))
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(87.701, 19.5),
                                new Pose(118.654, 17)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-110), Math.toRadians(90))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(118.654, 17),
                                new Pose(118.514, 33.776)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(118.514, 33.776),
                                new Pose(87.701, 17.944)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-113))
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(87.701, 19.5),
                                new Pose(109.037, 9.467)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-113), Math.toRadians(0))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(109.037, 9.467),
                                new Pose(130.934, 6.785)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(130.934, 6.785),
                                new Pose(87.701, 17.944)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-113))
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(87.701, 19.5),
                                new Pose(109.056, 9.150)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-113), Math.toRadians(0))
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(109.056, 9.150),
                                new Pose(125, 7.233)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125, 7.233),
                                new Pose(87.701, 17.944)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-113))
                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(87.701, 19.5),
                                new Pose(109.075, 8.963)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-113), Math.toRadians(0))
                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(109.075, 8.963),
                                new Pose(130.991, 7.196)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(130.991, 7.196),
                                new Pose(87.701, 20)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-113))
                .build();

        Path1a = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(87.701, 17.944),
                                new Pose(87.701, 19.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-110), Math.toRadians(-110))
                .build();

        Path4a = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(87.701, 17.944),
                                new Pose(87.701, 19.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-113), Math.toRadians(-113))
                .build();

        Path7a = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(87.701, 17.944),
                                new Pose(87.701, 19.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-113), Math.toRadians(-113))
                .build();

        Path10a = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(87.701, 17.944),
                                new Pose(87.701, 19.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-113), Math.toRadians(-113))
                .build();

    }

//=================================================================================================
    public void Update12GatePath() {
        switch (pathState) {
            case 0:
                follower.followPath(Path1,0.65, true);
                pathState = 1;
                timesShoot = 0;

                break;
            case 1:

                if(!follower.isBusy()){
                    if (!toShoot) sleep(200);

                    if (!follower.isTurning()) {

                        toShoot = true;
                    }


                        if (launching.post_shoot) {

                            timerReset = false;
                            toShoot = false;
                            launching.ispost_shoot(false);
                            follower.followPath(Path2, 0.9, true);
                            pathState = 2;
                            timesShoot = 1;
                        }

                }
                break;

            case 2:
                if(!follower.isBusy()) {

                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(Path3,0.65, true);
                    pathState = 3;
                }
                break;
            case 3:
                if(!follower.isBusy()) {

                    //sleep(200);
                    //intake.setMode(Intake.Mode.EJECT);
                    follower.followPath(Path4,0.85, true);
                    pathState = 4;
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    if (id != 21  && id != 22 && id != 23){
                        id = limelight.id;
                    }
                    intake.setMode(Intake.Mode.STOP);
                    sleep(1500);
                    follower.followPath(Path5,0.9, true);
                    pathState = 5;
                    timesShoot = 1;

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

                        timesShoot = 2;
                        pathState = 6;


                    }
                }
                break;

            case 6:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(Path7,0.65, true);
                    pathState = 7;
                }
            case 7:
                if(!follower.isBusy()) {
                    //sleep(200);
                    //intake.setMode(Intake.Mode.EJECT);
                    follower.followPath(Path8,0.9, true);
                    pathState = 8;
                }
                break;

            case 8:

                if(!follower.isBusy()){
                    toShoot = true;
                    intake.setMode(Intake.Mode.STOP);
                    if (launching.post_shoot) {
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        follower.followPath(Path9,0.9, true);
                        pathState = 9;
                        timesShoot = 3;
                    }
                }
                break;

            case 9:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(Path10,0.65, true);
                    pathState = 10;
                }
            case 10:
                if(!follower.isBusy()) {
                    //sleep(200);
                    //intake.setMode(Intake.Mode.EJECT);
                    follower.followPath(Path11,0.9, true);
                    pathState = 11;
                }
                break;
            case 11:
                if(!follower.isBusy()){
                    toShoot = true;
                    intake.setMode(Intake.Mode.STOP);
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
                follower.followPath(Path1,0.65, true);
                pathState = 1;
                timesShoot = 0;
                break;
            case 1:

                if(!follower.isBusy()){
                    toShoot = true;

                    if (id != 21  && id != 22 && id != 23){
                        id = limelight.id;
                    }

                    if (launching.post_shoot) {
//                        if(isBlue){
//                            servolime.updateDesiredReference(LimeServo.Mode.BLUE_TAG);
//                        }else{
//                            servolime.updateDesiredReference(LimeServo.Mode.RED_TAG);
//                        }
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
                    follower.followPath(Path3,0.65, true);
                    pathState = 3;
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(Path4,0.85, true);
                    pathState = 4;
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.STOP);
                    sleep(2500);
                    follower.followPath(Path5,0.9, true);
                    pathState = 5;
                    timesShoot = 1;

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
                        timesShoot = 2;
                    }
                }
                break;

            case 6:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(Path7,0.65, true);
                    pathState = 7;
                }
            case 7:
                if(!follower.isBusy()) {

                    follower.followPath(Path12,0.9, true);
                    pathState = 8;
                }
                break;

            case 8:

                if(!follower.isBusy()){
                    toShoot = true;
                    intake.setMode(Intake.Mode.STOP);
                    if (launching.post_shoot) {
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        pathState = -1;
                        timesShoot =3;
                    }
                }
                break;



        }
    }


    public void UpdateSmall_Human_Player() {
        switch (pathState) {
            case 0:
                if(angleCor == 0) {
                    follower.followPath(Path1, 0.9, true);
                    angleCor = 1;
                }else {
                    follower.followPath(Path1a, 0.85, true);
                    pathState = 1;
                }

                if (id != 21  && id != 22 && id != 23){
                    id = limelight.id;
                }


                break;
            case 1:

                if(!follower.isBusy()){
                    if (!toShoot) sleep(400);

                    follower.breakFollowing();
                    shoot = 1;
                    toShoot = true;
                    angleCor = 0;


                    if (launching.post_shoot) {
                        shoot = 0;
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        follower.followPath(Path2,1, true);
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
                    if(angleCor == 0) {
                        follower.followPath(Path4, 0.9, true);
                        angleCor = 1;
                    }else {
                        follower.followPath(Path4a, 0.85, true);
                        pathState = 4;
                    }
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.breakFollowing();
                    if (!toShoot) sleep(200);

                    shoot = 1;
                    intake.setMode(Intake.Mode.EJECT);
                    toShoot = true;
                    angleCor = 0;
                    if (launching.post_shoot) {
                        shoot = 0;
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        follower.followPath(Path5,1, true);
                        pathState = 5;
                        timesShoot = 2;
                    }

                }
                break;
            case 5:

                if(!follower.isBusy()){
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(Path6,0.7, true);
                    pathState = 6;

                }
                break;

            case 6:
                if(!follower.isBusy()) {

                    if(angleCor == 0) {
                        follower.followPath(Path7, 0.9, true);
                        angleCor = 1;
                    }else {
                        follower.followPath(Path7a, 0.85, true);
                        pathState = 7;
                    }
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.breakFollowing();
                    if (!toShoot) sleep(200);

                    angleCor = 0;
                    toShoot = true;
                    intake.setMode(Intake.Mode.EJECT);
                    if (launching.post_shoot) {
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        follower.followPath(Path8,1, true);
                        pathState = 8;
                        timesShoot = 3;
                    }
                }
                break;

            case 8:
                if(!follower.isBusy()){
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(Path9,0.7, true);
                    pathState = 9;

                }
                break;

            case 9:
                if(!follower.isBusy()) {

                    if(angleCor == 0) {
                        follower.followPath(Path10, 0.9, true);
                        angleCor = 1;
                    }else {
                        follower.followPath(Path10a, 0.85, true);
                        pathState = 10;
                    }
                }
                break;

            case 10:
                if(!follower.isBusy()) {
                    if (!toShoot) sleep(200);

                    follower.breakFollowing();
                    angleCor = 0;
                    toShoot = true;
                    intake.setMode(Intake.Mode.EJECT);
                    if (launching.post_shoot) {
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        follower.followPath(Path11,1, true);
                        pathState = 11;
                        timesShoot = 4;
                    }
                }
                break;
            case 11:

                if(!follower.isBusy()){

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
                    shoot = 1;
                    if (id != 21  && id != 22 && id != 23){
                        id = limelight.id;
                    }

                    if (launching.post_shoot) {
                        shoot = 0;
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        follower.followPath(Path5,0.9, true);
                        pathState = 2;
                        timesShoot = 1;
                    }
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    shoot = 0;
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(Path6,0.85, true);
                    pathState = 3;
                }
                break;
            case 3:
                if(!follower.isBusy()) {

                    follower.followPath(Path7,0.75, true);
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
                        timesShoot = 2;
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
                        follower.followPath(Path5,0.9, true);
                        pathState = 8;
                        timesShoot = 3;
                    }
                }
                break;

            case 8:

                if(!follower.isBusy()){
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(Path6,1, true);
                    pathState = 9;

                }
                break;

            case 9:
                if(!follower.isBusy()) {

                    follower.followPath(Path7,0.8, true);
                    pathState = 7;
                }
            case 10:
                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.EJECT);
                    toShoot = true;
                    intake.setMode(Intake.Mode.EJECT);
                    if (launching.post_shoot) {
                        intake.setMode(Intake.Mode.EJECT);
                        timerReset = false;
                        toShoot = false;
                        launching.ispost_shoot(false);
                        follower.followPath(Path5,0.9, true);
                        pathState = 11;
                        timesShoot = 4;
                    }
                }
                break;

            case 11:

                if(!follower.isBusy()) {
                    intake.setMode(Intake.Mode.STOP);
                    pathState = -1;
                    timesShoot = 0;
                }
                break;

        }
    }




}