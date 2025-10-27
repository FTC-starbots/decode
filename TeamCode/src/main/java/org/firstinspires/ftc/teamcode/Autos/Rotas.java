package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@Config
@Autonomous


public class Rotas extends LinearOpMode {
    public VoltageSensor myControlHubVoltageSensor;
    int posicao_desejada_indexer = 0, reservatorio = 0, reservatorio_1, reservatorio_2, reservatorio_3;

    double power_correction;
    double potencia_indexer, integral_PiDF_Shooter, lasterror_PiDF_Shooter, integral_Indexer, lasterror_PiDF_Indexer;
    double[] media_speed = new double[10];
    public VoltageSensor ControlHub_VoltageSensor;


    //variáveis dos controladores PiDF____________________________
    double integral_PiDF_Correcao_Angular = 0;
    double lasterror_PiDF_Correcao_Angular = 0;

    //____________________________________________________________

    double cicle_time = 0;



    public Limelight3A limelight;

    public DcMotorEx leftBack, leftFront, rightBack, rightFront, Intake, Atirador_1, Atirador_2, Indexer;

    public Servo Reservoir_2, Reservoir_1, Reservoir_3;

    public RevBlinkinLedDriver Led_Strip;

    boolean PIDs_state;

    public int id_ap;

    ElapsedTime timer_shooter, deltaTime;

    public double velocidade_desejada = 1165, Ai_shoot;

    public Action rota1;
    public Action rota2;
    public Action rota3;
    public Action rota4;

    public Action rota5;

    public Action rota6;

    public Action rota7;
    public Action rota8;
    public Action rota9;
    public Action rota10;

    public Action rota11;



    MecanumDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {
    }

    public void Rotas_APequeno() {

        MecanumDrive.PARAMS.axialGain = 7.5;
        MecanumDrive.PARAMS.lateralGain = 40;
        MecanumDrive.PARAMS.headingGain = 36;
        MecanumDrive.PARAMS.maxWheelVel = 60;


        drive = new MecanumDrive(hardwareMap, new Pose2d(10, 55,
                Math.toRadians(90)));

        rota1 = drive.actionBuilder(new Pose2d(10, 55, Math.toRadians(90)))

                .strafeToLinearHeading(new Vector2d(10, -25),Math.toRadians(85),new TranslationalVelConstraint(80))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota2 = drive.actionBuilder(new Pose2d(10, -25, Math.toRadians(85)))

                .turnTo( Math.toRadians(148))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })

                .build();

        rota3 = drive.actionBuilder(new Pose2d(10, -25, Math.toRadians(148)))
                .afterTime(0, ()->{
                    velocidade_desejada = 1165;

                })

                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })

                .waitSeconds(1.2)

                .afterTime(0, ()->{
                    if (id_ap == 21) Reservoir_1.setPosition(1);
                    if (id_ap == 22) Reservoir_2.setPosition(0);
                    if (id_ap == 23) Reservoir_2.setPosition(0);

                    PIDs_state = true;
                })

                .waitSeconds(0.8)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 218;
                    if (id_ap == 22) posicao_desejada_indexer = 0;
                    if (id_ap == 23) posicao_desejada_indexer = 440;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    if (id_ap == 21) Reservoir_2.setPosition(0);
                    if (id_ap == 22) Reservoir_1.setPosition(1);
                    if (id_ap == 23) Reservoir_3.setPosition(1);

                    PIDs_state = true;
                })

                .waitSeconds(0.8)


                .afterTime(0, ()->{

                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 440;
                    if (id_ap == 22) posicao_desejada_indexer = 440;
                    if (id_ap == 23) posicao_desejada_indexer = 0;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    if (id_ap == 21) Reservoir_3.setPosition(1);
                    if (id_ap == 22) Reservoir_3.setPosition(1);
                    if (id_ap == 23) Reservoir_1.setPosition(1);

                    PIDs_state = true;
                })

                .waitSeconds(1)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);
                    posicao_desejada_indexer = 0;
                })

                .afterTime(0, ()->{
                    PIDs_state = false;
                })

                .build();
//
        rota4 = drive.actionBuilder(new Pose2d(10, -25, Math.toRadians(147)))

                .turnTo(Math.toRadians(165))

                .lineToX(44.5)

                .turnTo(Math.toRadians(90))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota5 = drive.actionBuilder(new Pose2d(44.5, -25, Math.toRadians(90)))

                .strafeToLinearHeading(new Vector2d(44.5, -11.5),Math.toRadians(90),new TranslationalVelConstraint(30))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota6 = drive.actionBuilder(new Pose2d(44.5, -11.5, Math.toRadians(90)))

                .strafeToLinearHeading(new Vector2d(10, -25),Math.toRadians(145))
                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota7 = drive.actionBuilder(new Pose2d(10, -25, Math.toRadians(145)))
                .afterTime(0, ()->{
                    velocidade_desejada = 1165;

                })
                .turnTo(Math.toRadians(143))
                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })

                .waitSeconds(1.2)

                .afterTime(0, ()->{
                    if (id_ap == 21) Reservoir_1.setPosition(1);
                    if (id_ap == 22) Reservoir_2.setPosition(0);
                    if (id_ap == 23) Reservoir_2.setPosition(0);

                    PIDs_state = true;
                })

                .waitSeconds(0.8)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 218;
                    if (id_ap == 22) posicao_desejada_indexer = 0;
                    if (id_ap == 23) posicao_desejada_indexer = 440;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    if (id_ap == 21) Reservoir_2.setPosition(0);
                    if (id_ap == 22) Reservoir_1.setPosition(1);
                    if (id_ap == 23) Reservoir_3.setPosition(1);

                    PIDs_state = true;
                })

                .waitSeconds(0.8)


                .afterTime(0, ()->{

                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 440;
                    if (id_ap == 22) posicao_desejada_indexer = 440;
                    if (id_ap == 23) posicao_desejada_indexer = 0;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    if (id_ap == 21) Reservoir_3.setPosition(1);
                    if (id_ap == 22) Reservoir_3.setPosition(1);
                    if (id_ap == 23) Reservoir_1.setPosition(1);

                    PIDs_state = true;
                })

                .waitSeconds(1)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);
                    posicao_desejada_indexer = 0;
                })

                .afterTime(0, ()->{
                    PIDs_state = false;
                })

                .build();

        rota8 = drive.actionBuilder(new Pose2d(10, -25, Math.toRadians(145)))

                .strafeToLinearHeading(new Vector2d(44.5, -15),Math.toRadians(132))

                .turnTo(Math.toRadians(90))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota9 = drive.actionBuilder(new Pose2d(44.5, -15, Math.toRadians(90)))

                .strafeToLinearHeading(new Vector2d(44.5, 10),Math.toRadians(90),new TranslationalVelConstraint(30))
                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota10 = drive.actionBuilder(new Pose2d(44.5, 10, Math.toRadians(90)))

                .strafeToLinearHeading(new Vector2d(10, -25),Math.toRadians(145))
                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota11 = drive.actionBuilder(new Pose2d(25, -45, Math.toRadians(145)))
                .afterTime(0, ()->{
                    velocidade_desejada = 1180;

                })
                .turnTo(Math.toRadians(150))
                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })

                .waitSeconds(1.2)

                .afterTime(0, ()->{
                    if (id_ap == 21) Reservoir_1.setPosition(1);
                    if (id_ap == 22) Reservoir_2.setPosition(0);
                    if (id_ap == 23) Reservoir_2.setPosition(0);

                    PIDs_state = true;
                })

                .waitSeconds(0.8)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 218;
                    if (id_ap == 22) posicao_desejada_indexer = 0;
                    if (id_ap == 23) posicao_desejada_indexer = 440;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    if (id_ap == 21) Reservoir_2.setPosition(0);
                    if (id_ap == 22) Reservoir_1.setPosition(1);
                    if (id_ap == 23) Reservoir_3.setPosition(1);

                    PIDs_state = true;
                })

                .waitSeconds(0.8)


                .afterTime(0, ()->{

                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 440;
                    if (id_ap == 22) posicao_desejada_indexer = 440;
                    if (id_ap == 23) posicao_desejada_indexer = 0;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    if (id_ap == 21) Reservoir_3.setPosition(1);
                    if (id_ap == 22) Reservoir_3.setPosition(1);
                    if (id_ap == 23) Reservoir_1.setPosition(1);

                    PIDs_state = true;
                })

                .waitSeconds(1)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);
                    posicao_desejada_indexer = 0;
                })

                .afterTime(0, ()->{
                    PIDs_state = false;
                })

                .build();


    }







}
