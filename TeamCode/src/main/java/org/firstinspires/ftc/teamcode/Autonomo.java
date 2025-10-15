package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Config
@Autonomous


public class Autonomo extends LinearOpMode {

    public class PIDs implements Action {
        public PIDs() {

            //inicializa o braço
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            //executa o movimento do braço em loop
            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) fr.getFiducialId();


            return PIDs_state;
        }
    }


    private Limelight3A limelight;

    private DcMotorEx leftBack, leftFront, rightBack, rightFront, Intake, Atirador_1, Atirador_2, Indexer;

    private Servo Reservoir_2, Reservoir_1, Reservoir_3;

    private RevBlinkinLedDriver Led_Strip;

    int posicao_desejada_indexer = 0, reservatorio = 0;


    private VoltageSensor ControlHub_VoltageSensor;

    boolean PIDs_state;

    public int id_ap;

    ElapsedTime  timer_shooter;



    private Action rota1;
    private Action rota2;
    private Action rota3;
    private Action rota4;
    private Action rota5;
    private Action rota6;
    private Action rota7;
    private Action rota8;
    private Action rota9;

    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Indexer = hardwareMap.get(DcMotorEx.class, "Indexer");
        Atirador_1 = hardwareMap.get(DcMotorEx.class, "Atirador_1");
        Atirador_2 = hardwareMap.get(DcMotorEx.class, "Atirador_2");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Reservoir_2 = hardwareMap.get(Servo.class, "Reservoir_2");
        Reservoir_1 = hardwareMap.get(Servo.class, "Reservoir_1");
        Reservoir_3 = hardwareMap.get(Servo.class, "Reservoir_3");
        Led_Strip = hardwareMap.get(RevBlinkinLedDriver.class, "Led_Strip");

        timer_shooter   = new ElapsedTime();
        timer_shooter.reset();

        //--------Calcula Rotas e parametriza drivetrain---------------------
        Rotas();
        //------------------------------------------
        waitForStart();
        PIDs_state = true;
        Actions.runBlocking(new ParallelAction(rota1, new PIDs()));

    }

    public void Rotas() {

        MecanumDrive.PARAMS.axialGain = 7.5;
        MecanumDrive.PARAMS.lateralGain = 40;
        MecanumDrive.PARAMS.headingGain = 36;
        MecanumDrive.PARAMS.maxWheelVel = 35;


        drive = new MecanumDrive(hardwareMap, new Pose2d(35, -45,
                Math.toRadians(180)));

        rota1 = drive.actionBuilder(new Pose2d(-35, -45, Math.toRadians(180)))

                .strafeToLinearHeading(new Vector2d(30, -40), Math.toRadians(90))
                .afterTime(0, () -> {
                    LLResult result = limelight.getLatestResult();
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

                    if (fiducialResults.get(0).getFiducialId() == 21 || fiducialResults.get(1).getFiducialId() == 21 || fiducialResults.get(2).getFiducialId() == 21)
                        id_ap = 21;
                    if (fiducialResults.get(0).getFiducialId() == 22 || fiducialResults.get(1).getFiducialId() == 22 || fiducialResults.get(2).getFiducialId() == 22)
                        id_ap = 22;
                    if (fiducialResults.get(0).getFiducialId() == 23 || fiducialResults.get(1).getFiducialId() == 23 || fiducialResults.get(2).getFiducialId() == 23)
                        id_ap = 23;

                })
                .strafeTo(new Vector2d(30, -10))

                .strafeTo(new Vector2d(-32.5, -35))

                .build();
    }

    private void Shoot_single() {

        if (0 < timer_shooter.seconds() && 0.9 >= timer_shooter.seconds()){
            if(reservatorio == 1)  posicao_desejada_indexer= 0;
            if(reservatorio == 2)  posicao_desejada_indexer= 210;
            if(reservatorio == 3)  posicao_desejada_indexer= 440;

        }else if (0.9 < timer_shooter.seconds() && 1.7 >= timer_shooter.seconds()){

            if(reservatorio == 1)Reservoir_1.setPosition(1);
            if(reservatorio == 2) Reservoir_2.setPosition(1);
            if(reservatorio == 3)Reservoir_3.setPosition(1);

        }else if (1.7 < timer_shooter.seconds() && 1.8 >= timer_shooter.seconds()){
            if(reservatorio == 1)Reservoir_1.setPosition(0);
            if(reservatorio == 2) Reservoir_2.setPosition(0);
            if(reservatorio == 3)Reservoir_3.setPosition(0);
        }else if (1.8 < timer_shooter.seconds() && 1.9 >= timer_shooter.seconds()){
            Shoot = false;
        }

    }
}

        /*//clipa o segundo clipe
        rota4 = drive.actionBuilder(new Pose2d(37, -70.5, Math.toRadians(90)))
                .afterTime(0,()->{

                    RotacaoOuttake.setPosition(1);
                    posicaoDesejada_Braco = 950;
                    posicaoDesejada_Slide = 1900;
                })
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(1.1,-34),Math.toRadians(90), new TranslationalVelConstraint(30))
                .splineTo(new Vector2d(1.1,-28.5),Math.toRadians(90), new TranslationalVelConstraint(25),new ProfileAccelConstraint(-53, 63))
                .afterTime(0,()->{
                    posicaoDesejada_Braco = 0;
                })
                .waitSeconds(0.25)
                .afterTime(0,()->{
                    GarraOuttake.setPosition(0.3);
                })
                .afterTime(0,()->{
                    PIDs_state = false;
                })
                .build();

        //coleta o terceiro clipe
        rota5 = drive.actionBuilder(new Pose2d(1.1, -27, Math.toRadians(90)))

                .afterTime(0,()->{
                    posicaoDesejada_Braco = 0;
                    RotacaoOuttake.setPosition(0.5);
                })
                .afterTime(0,()->{
                    posicaoDesejada_Slide = 100;
                })
                .setTangent(Math.toRadians(270))
                .splineTo(new Vector2d(37,-65),Math.toRadians(270), new TranslationalVelConstraint(30))
                .splineTo(new Vector2d(37,-68),Math.toRadians(270), new TranslationalVelConstraint(10))
                .afterTime(0,()->{
                    // GarraOuttake.setPosition(0.9);
                    PIDs_state = false;
                })
                .splineTo(new Vector2d(37,-70.5),Math.toRadians(270), new TranslationalVelConstraint(10))
                .build();

        //clipa o terceiro clipe
        rota6 = drive.actionBuilder(new Pose2d(37, -70.5, Math.toRadians(90)))
                .afterTime(0,()->{

                    RotacaoOuttake.setPosition(1);
                    posicaoDesejada_Braco = 950;
                    posicaoDesejada_Slide = 1900;
                })
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(2.5,-34),Math.toRadians(90), new TranslationalVelConstraint(30))
                .splineTo(new Vector2d(2.5,-28.5),Math.toRadians(90), new TranslationalVelConstraint(25),new ProfileAccelConstraint(-53, 63))
                .afterTime(0,()
                        ->{
                    posicaoDesejada_Braco = 0;
                })
                .waitSeconds(0.25)
                .afterTime(0,()->{
                    GarraOuttake.setPosition(0.3);
                })
                .afterTime(0,()->{
                    PIDs_state = false;
                })
                .build();
//----------------------------------------------------------------------------------------------------------
        //coleta o quarto sample
        rota7 = drive.actionBuilder(new Pose2d(2.5, -27, Math.toRadians(90)))
                .afterTime(0,()->{
                    posicaoDesejada_Braco = 0;
                    RotacaoOuttake.setPosition(0.5);
                })
                .afterTime(0,()->{
                    posicaoDesejada_Slide = 100;
                })
                .setTangent(Math.toRadians(270))
                .splineTo(new Vector2d(37,-65),Math.toRadians(270), new TranslationalVelConstraint(30))
                .splineTo(new Vector2d(37,-68),Math.toRadians(270), new TranslationalVelConstraint(10))
                .afterTime(0,()->{
                    // GarraOuttake.setPosition(0.9);
                    PIDs_state = false;
                })
                .splineTo(new Vector2d(37,-70.5),Math.toRadians(270), new TranslationalVelConstraint(10))
                .build();

        //clipa o quarto sample
        rota8 = drive.actionBuilder(new Pose2d(37, -70.5, Math.toRadians(90)))
                .afterTime(0,()->{

                    RotacaoOuttake.setPosition(1);
                    posicaoDesejada_Braco = 950;
                    posicaoDesejada_Slide = 1900;
                })
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(-3,-34),Math.toRadians(90), new TranslationalVelConstraint(30))
                .splineTo(new Vector2d(-3,-28.5),Math.toRadians(90), new TranslationalVelConstraint(25),new ProfileAccelConstraint(-53, 63))
                .afterTime(0,()->{
                    posicaoDesejada_Braco = 0;
                })
                .waitSeconds(0.25)
                .afterTime(0,()->{
                    GarraOuttake.setPosition(0.3);
                })
                .afterTime(0,()->{
                    PIDs_state = false;
                })
                .build();

        //estaciona
        rota9 = drive.actionBuilder(new Pose2d(-3, -27, Math.toRadians(90)))
                .afterTime(0,()->{
                    posicaoDesejada_Slide = 0;
                    posicaoDesejada_Braco = 0;
                    RotacaoOuttake.setPosition(0.5);
                })
                .setTangent(Math.toRadians(270))
                .splineTo(new Vector2d(37,-70),Math.toRadians(270), new TranslationalVelConstraint(60),new ProfileAccelConstraint(-53, 63))
                .afterTime(0,()->{
                    PIDs_state = false;
                })
                .build();*/


