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
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@Config
@Autonomous


public class Rotas extends LinearOpMode {

    public DigitalChannel  Alliance_B, Alliance_R, Auto1,Auto2, Auto3, Auto4, LED_S_R,LED_S_G,LED_S_B;
    public LED Led_1_R, Led_1_V, Led_2_R, Led_2_V, Led_3_R, Led_3_V;
    public AnalogInput S1, S2,S3,S4;
    public VoltageSensor myControlHubVoltageSensor;
    int posicao_desejada_indexer = 0, reservatorio = 0, reservatorio_1, reservatorio_2, reservatorio_3;

    double power_correction, tx_offset;
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

    public double velocidade_desejada = 1165, Ai_shoot, Compen_at = 0, Compen_tx = 0, Compen_vl = 0;

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

    public void Rotas_APequeno_Perto() {

        MecanumDrive.PARAMS.axialGain = 7.5;
        MecanumDrive.PARAMS.lateralGain = 40;
        MecanumDrive.PARAMS.headingGain = 36;
        MecanumDrive.PARAMS.maxWheelVel = 60;


        drive = new MecanumDrive(hardwareMap, new Pose2d(18, 55,
                Math.toRadians(90)));

        rota1 = drive.actionBuilder(new Pose2d(18, 55, Math.toRadians(90)))

                .strafeToLinearHeading(new Vector2d(24, -30),Math.toRadians(85),new TranslationalVelConstraint(70))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota2 = drive.actionBuilder(new Pose2d(24, -30, Math.toRadians(85)))

                .turnTo( Math.toRadians(145))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })

                .build();

        rota3 = drive.actionBuilder(new Pose2d(24, -30, Math.toRadians(145)))
                .afterTime(0, ()->{

                })

                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


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
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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
        rota4 = drive.actionBuilder(new Pose2d(24, -30, Math.toRadians(145)))


                .setReversed(true)
                .splineTo(new Vector2d(51, -35),Math.toRadians(270))


                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota5 = drive.actionBuilder(new Pose2d(51, -20, Math.toRadians(90)))
                .setReversed(false)
                .turnTo(Math.toRadians(97))

                .strafeToLinearHeading(new Vector2d(51, -11.5),Math.toRadians(97),new TranslationalVelConstraint(30))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota6 = drive.actionBuilder(new Pose2d(51, -11.5, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(24, -30),Math.toRadians(280))
                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota7 = drive.actionBuilder(new Pose2d(24, -30, Math.toRadians(100)))

                .turnTo(Math.toRadians(143.1))
                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


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
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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

        rota8 = drive.actionBuilder(new Pose2d(24, -30, Math.toRadians(143)))
                .setReversed(true)
                .splineTo(new Vector2d(50, -15),Math.toRadians(270))


                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota9 = drive.actionBuilder(new Pose2d(50, -5, Math.toRadians(90)))
                .setReversed(false)
                .turnTo(Math.toRadians(97))
                .strafeToLinearHeading(new Vector2d(50, 10),Math.toRadians(97),new TranslationalVelConstraint(30))
                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota10 = drive.actionBuilder(new Pose2d(50, 10, Math.toRadians(90)))

                .setReversed(true)
                .splineTo(new Vector2d(24, -30),Math.toRadians(280))
                .afterTime(0, ()->{
                    PIDs_state = false;
                })


                .build();

        rota11 = drive.actionBuilder(new Pose2d(24, -30, Math.toRadians(100)))


                .turnTo(Math.toRadians(143))
                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 218;
                    if (id_ap == 22) posicao_desejada_indexer = 0;
                    if (id_ap == 23) posicao_desejada_indexer = 0;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })
//
                 //
                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 440;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


                .afterTime(0, ()->{

                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 440;
                    if (id_ap == 22) posicao_desejada_indexer = 440;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);
                    posicao_desejada_indexer = 0;
                })

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .strafeToLinearHeading(new Vector2d(20, -40),Math.toRadians(0))

                .build();


    }

    public void Rotas_APequeno_Longe() {

        MecanumDrive.PARAMS.axialGain = 7.5;
        MecanumDrive.PARAMS.lateralGain = 40;
        MecanumDrive.PARAMS.headingGain = 36;
        MecanumDrive.PARAMS.maxWheelVel = 60;


        drive = new MecanumDrive(hardwareMap, new Pose2d(18, 55,
                Math.toRadians(90)));

        rota1 = drive.actionBuilder(new Pose2d(18, 55, Math.toRadians(90)))

                .strafeToLinearHeading(new Vector2d(18, 47),Math.toRadians(85))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota2 = drive.actionBuilder(new Pose2d(18, 47, Math.toRadians(85)))

                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })
                .turnTo(Math.toRadians(110))
                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


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
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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
        rota3 = drive.actionBuilder(new Pose2d(18, 47, Math.toRadians(110)))

                .strafeToLinearHeading(new Vector2d(48, 47),Math.toRadians(270))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota4 = drive.actionBuilder(new Pose2d(48, 47, Math.toRadians(270)))

                .strafeToLinearHeading(new Vector2d(48, 27),Math.toRadians(270),new TranslationalVelConstraint(30))


                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota5 = drive.actionBuilder(new Pose2d(48, 27, Math.toRadians(270)))

                .strafeToLinearHeading(new Vector2d(18, 47),Math.toRadians(102))
                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota6 = drive.actionBuilder(new Pose2d(18, 47, Math.toRadians(102)))
                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 440;
                    if (id_ap == 22) posicao_desejada_indexer = 0;
                    if (id_ap == 23) posicao_desejada_indexer = 0;

                })
                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 218;
                    if (id_ap == 22) posicao_desejada_indexer = 440;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


                .afterTime(0, ()->{

                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 440;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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

        rota7 = drive.actionBuilder(new Pose2d(18, 47, Math.toRadians(102)))

                .strafeToLinearHeading(new Vector2d(48, 27),Math.toRadians(270))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota8 = drive.actionBuilder(new Pose2d(48, 27, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(48, 5),Math.toRadians(270),new TranslationalVelConstraint(30))


                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota9 = drive.actionBuilder(new Pose2d(48, 5, Math.toRadians(270)))

                .strafeToLinearHeading(new Vector2d(18, 47),Math.toRadians(102))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota10 = drive.actionBuilder(new Pose2d(10, 55, Math.toRadians(102)))
                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 218;
                    if (id_ap == 22) posicao_desejada_indexer = 0;
                    if (id_ap == 23) posicao_desejada_indexer = 0;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })
//
                //
                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 440;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


                .afterTime(0, ()->{

                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 440;
                    if (id_ap == 22) posicao_desejada_indexer = 440;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);
                    posicao_desejada_indexer = 0;
                })

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .strafeToLinearHeading(new Vector2d(20, 35),Math.toRadians(0))


                .build();


    }

    public void Rotas_AGrande_Fora() {

        MecanumDrive.PARAMS.axialGain = 7.5;
        MecanumDrive.PARAMS.lateralGain = 40;
        MecanumDrive.PARAMS.headingGain = 36;
        MecanumDrive.PARAMS.maxWheelVel = 60;


        drive = new MecanumDrive(hardwareMap, new Pose2d(42, -55,
                Math.toRadians(90)));

        rota1 = drive.actionBuilder(new Pose2d(42, -55, Math.toRadians(90)))

                .strafeToLinearHeading(new Vector2d(6, -50),Math.toRadians(82),new TranslationalVelConstraint(70))
                .waitSeconds(0.2)

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota2 = drive.actionBuilder(new Pose2d(6, -50, Math.toRadians(82)))
                .turnTo( Math.toRadians(160))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })

                .build();

        rota3 = drive.actionBuilder(new Pose2d(6, -40, Math.toRadians(160)))



                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })
                .strafeToLinearHeading(new Vector2d(15, -40),Math.toRadians(160))

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


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
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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
        rota4 = drive.actionBuilder(new Pose2d(15, -40, Math.toRadians(172)))


                .setReversed(true)
                .splineTo(new Vector2d(47.5, -40),Math.toRadians(270))


                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota5 = drive.actionBuilder(new Pose2d(47.5, -40, Math.toRadians(90)))
                .setReversed(false)
                .turnTo(Math.toRadians(97))

                .splineTo(new Vector2d(47.5, -11.5),Math.toRadians(97),new TranslationalVelConstraint(45))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota6 = drive.actionBuilder(new Pose2d(47.5, -11.5, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(32, -32),Math.toRadians(358),new TranslationalVelConstraint(85))
                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota7 = drive.actionBuilder(new Pose2d(35, -35, Math.toRadians(170)))
                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })
                .strafeToLinearHeading(new Vector2d(15, -40),Math.toRadians(160),new TranslationalVelConstraint(70))

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


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
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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

        rota8 = drive.actionBuilder(new Pose2d(15, -40, Math.toRadians(172)))
                .turnTo(Math.toRadians(60))


                //.setReversed(true)

                //.splineTo(new Vector2d(47, -40),Math.toRadians(315))


                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota9 = drive.actionBuilder(new Pose2d(15, -40, Math.toRadians(60)))
                .splineTo(new Vector2d(47, -20),Math.toRadians(90),new TranslationalVelConstraint(50))
                .splineTo(new Vector2d(48.5, 10),Math.toRadians(90),new TranslationalVelConstraint(20))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota10 = drive.actionBuilder(new Pose2d(48.5, 10, Math.toRadians(90)))

                .setReversed(true)
                .splineTo(new Vector2d(35, -35),Math.toRadians(350),new TranslationalVelConstraint(85))
                .afterTime(0, ()->{
                    PIDs_state = false;
                })


                .build();

        rota11 = drive.actionBuilder(new Pose2d(35, -35, Math.toRadians(170)))


                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 218;
                    if (id_ap == 22) posicao_desejada_indexer = 0;
                    if (id_ap == 23) posicao_desejada_indexer = 0;

                })
                .strafeToLinearHeading(new Vector2d(15, -40),Math.toRadians(172),new TranslationalVelConstraint(70))
                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })
//
                //
                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 440;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


                .afterTime(0, ()->{

                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 440;
                    if (id_ap == 22) posicao_desejada_indexer = 440;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);
                    posicao_desejada_indexer = 0;
                })

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .strafeToLinearHeading(new Vector2d(20, -40),Math.toRadians(0))


                .build();


    }

    public void Rotas_AGrande_6() {

        MecanumDrive.PARAMS.axialGain = 7.5;
        MecanumDrive.PARAMS.lateralGain = 40;
        MecanumDrive.PARAMS.headingGain = 36;
        MecanumDrive.PARAMS.maxWheelVel = 60;


        drive = new MecanumDrive(hardwareMap, new Pose2d(42, -55,
                Math.toRadians(90)));

        rota1 = drive.actionBuilder(new Pose2d(42, -55, Math.toRadians(90)))

                .strafeToLinearHeading(new Vector2d(6, -40),Math.toRadians(82),new TranslationalVelConstraint(70))
                .waitSeconds(0.1)

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota2 = drive.actionBuilder(new Pose2d(6, -40, Math.toRadians(82)))
                .turnTo( Math.toRadians(160))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })

                .build();

        rota3 = drive.actionBuilder(new Pose2d(6, -40, Math.toRadians(160)))



                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })
                .strafeToLinearHeading(new Vector2d(15, -40),Math.toRadians(160))

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


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
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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
        rota4 = drive.actionBuilder(new Pose2d(15, -40, Math.toRadians(172)))


                .setReversed(true)
                .splineTo(new Vector2d(47.5, -40),Math.toRadians(270))


                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota5 = drive.actionBuilder(new Pose2d(47.5, -40, Math.toRadians(90)))
                .setReversed(false)
                .turnTo(Math.toRadians(97))

                .splineTo(new Vector2d(47.5, -11.5),Math.toRadians(97),new TranslationalVelConstraint(45))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota6 = drive.actionBuilder(new Pose2d(47.5, -11.5, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(32, -32),Math.toRadians(358),new TranslationalVelConstraint(85))
                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota7 = drive.actionBuilder(new Pose2d(35, -35, Math.toRadians(170)))
                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })
                .strafeToLinearHeading(new Vector2d(15, -40),Math.toRadians(160),new TranslationalVelConstraint(70))

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


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
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    posicao_desejada_indexer = 0;
                })

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .strafeToLinearHeading(new Vector2d(20, -40),Math.toRadians(0))

                .build();


    }
//------------------------------------------------------
    public void Rotas_VPequeno_Perto() {

        MecanumDrive.PARAMS.axialGain = 7.5;
        MecanumDrive.PARAMS.lateralGain = 40;
        MecanumDrive.PARAMS.headingGain = 36;
        MecanumDrive.PARAMS.maxWheelVel = 60;


        drive = new MecanumDrive(hardwareMap, new Pose2d(-18, 55,
                Math.toRadians(90)));

        rota1 = drive.actionBuilder(new Pose2d(-18, 55, Math.toRadians(90)))

                .strafeToLinearHeading(new Vector2d(-24, -30),Math.toRadians(98),new TranslationalVelConstraint(70))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota2 = drive.actionBuilder(new Pose2d(-24, -30, Math.toRadians(98)))

                .turnTo( Math.toRadians(35))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })

                .build();

        rota3 = drive.actionBuilder(new Pose2d(-24, -30, Math.toRadians(35)))
                .afterTime(0, ()->{

                })

                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


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
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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
        rota4 = drive.actionBuilder(new Pose2d(-24, -30, Math.toRadians(35)))


                .setReversed(true)
                .splineTo(new Vector2d(-51, -35),Math.toRadians(270))


                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota5 = drive.actionBuilder(new Pose2d(-51, -20, Math.toRadians(90)))
                .setReversed(false)
                .turnTo(Math.toRadians(85))

                .strafeToLinearHeading(new Vector2d(-51, -11.5),Math.toRadians(85),new TranslationalVelConstraint(30))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota6 = drive.actionBuilder(new Pose2d(-51, -11.5, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(-24, -30),Math.toRadians(260))
                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota7 = drive.actionBuilder(new Pose2d(-24, -30, Math.toRadians(80)))

                .turnTo(Math.toRadians(36.9))
                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 440;
                    if (id_ap == 22) posicao_desejada_indexer = 0;
                    if (id_ap == 23) posicao_desejada_indexer = 0;

                })
                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 218;
                    if (id_ap == 22) posicao_desejada_indexer = 440;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


                .afterTime(0, ()->{

                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 440;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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

        rota8 = drive.actionBuilder(new Pose2d(-24, -30, Math.toRadians(36.9)))
                .setReversed(true)
                .splineTo(new Vector2d(-50, -15),Math.toRadians(270))


                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota9 = drive.actionBuilder(new Pose2d(-50, -5, Math.toRadians(90)))
                .setReversed(false)
                .turnTo(Math.toRadians(85))
                .strafeToLinearHeading(new Vector2d(-50, 10),Math.toRadians(85),new TranslationalVelConstraint(30))
                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota10 = drive.actionBuilder(new Pose2d(-50, 10, Math.toRadians(90)))

                .setReversed(true)
                .splineTo(new Vector2d(-24, -30),Math.toRadians(260))
                .afterTime(0, ()->{
                    PIDs_state = false;
                })


                .build();

        rota11 = drive.actionBuilder(new Pose2d(-24, -30, Math.toRadians(80)))


                .turnTo(Math.toRadians(37))
                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 218;
                    if (id_ap == 22) posicao_desejada_indexer = 0;
                    if (id_ap == 23) posicao_desejada_indexer = 0;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })
//
                //
                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 440;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


                .afterTime(0, ()->{

                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 440;
                    if (id_ap == 22) posicao_desejada_indexer = 440;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);
                    posicao_desejada_indexer = 0;
                })

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .strafeToLinearHeading(new Vector2d(-15, -40),Math.toRadians(180))

                .build();


    }

    public void Rotas_VPequeno_Longe() {

        MecanumDrive.PARAMS.axialGain = 7.5;
        MecanumDrive.PARAMS.lateralGain = 40;
        MecanumDrive.PARAMS.headingGain = 36;
        MecanumDrive.PARAMS.maxWheelVel = 60;


        drive = new MecanumDrive(hardwareMap, new Pose2d(-18, 55,
                Math.toRadians(90)));

        rota1 = drive.actionBuilder(new Pose2d(-18, 55, Math.toRadians(90)))

                .strafeToLinearHeading(new Vector2d(-18, 47),Math.toRadians(95))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota2 = drive.actionBuilder(new Pose2d(-18, 47, Math.toRadians(95)))

                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })
                .turnTo(Math.toRadians(70))
                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


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
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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
        rota3 = drive.actionBuilder(new Pose2d(-18, 47, Math.toRadians(70)))

                .strafeToLinearHeading(new Vector2d(-48, 47),Math.toRadians(270))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota4 = drive.actionBuilder(new Pose2d(-48, 47, Math.toRadians(270)))

                .strafeToLinearHeading(new Vector2d(-48, 27),Math.toRadians(270),new TranslationalVelConstraint(30))


                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota5 = drive.actionBuilder(new Pose2d(-48, 27, Math.toRadians(270)))

                .strafeToLinearHeading(new Vector2d(-18, 47),Math.toRadians(78))
                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota6 = drive.actionBuilder(new Pose2d(-18, 47, Math.toRadians(78)))
                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })
                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


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
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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

        rota7 = drive.actionBuilder(new Pose2d(-18, 47, Math.toRadians(78)))

                .strafeToLinearHeading(new Vector2d(-48, 27),Math.toRadians(270))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota8 = drive.actionBuilder(new Pose2d(-48, 27, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-48, 5),Math.toRadians(270),new TranslationalVelConstraint(30))


                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota9 = drive.actionBuilder(new Pose2d(-48, 5, Math.toRadians(270)))

                .strafeToLinearHeading(new Vector2d(-18, 47),Math.toRadians(78))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota10 = drive.actionBuilder(new Pose2d(-18, 47, Math.toRadians(78)))
                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 218;
                    if (id_ap == 22) posicao_desejada_indexer = 0;
                    if (id_ap == 23) posicao_desejada_indexer = 0;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })
//
                //
                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 440;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


                .afterTime(0, ()->{

                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 440;
                    if (id_ap == 22) posicao_desejada_indexer = 440;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);
                    posicao_desejada_indexer = 0;
                })

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .strafeToLinearHeading(new Vector2d(-20, 35),Math.toRadians(180))

                .build();


    }

    public void Rotas_VGrande_Fora() {

        MecanumDrive.PARAMS.axialGain = 7.5;
        MecanumDrive.PARAMS.lateralGain = 40;
        MecanumDrive.PARAMS.headingGain = 36;
        MecanumDrive.PARAMS.maxWheelVel = 60;


        drive = new MecanumDrive(hardwareMap, new Pose2d(-42, -55,
                Math.toRadians(90)));

        rota1 = drive.actionBuilder(new Pose2d(-42, -55, Math.toRadians(90)))

                .strafeToLinearHeading(new Vector2d(-6, -50),Math.toRadians(90.1),new TranslationalVelConstraint(60))
                .waitSeconds(0.55)
                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota2 = drive.actionBuilder(new Pose2d(-6, -50, Math.toRadians(90)))

                .turnTo( Math.toRadians(20))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })

                .build();

        rota3 = drive.actionBuilder(new Pose2d(-6, -50, Math.toRadians(20)))



                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })
                .strafeToLinearHeading(new Vector2d(-15, -40),Math.toRadians(20))

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


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
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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
        rota4 = drive.actionBuilder(new Pose2d(-15, -40, Math.toRadians(8)))


                .setReversed(true)
                .splineTo(new Vector2d(-50, -40),Math.toRadians(270))


                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota5 = drive.actionBuilder(new Pose2d(-50, -40, Math.toRadians(90)))
                .setReversed(false)
                .turnTo(Math.toRadians(85))

                .splineTo(new Vector2d(-50, -11.5),Math.toRadians(85),new TranslationalVelConstraint(45))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota6 = drive.actionBuilder(new Pose2d(-50, -11.5, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(-35, -35),Math.toRadians(182),new TranslationalVelConstraint(85))
                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota7 = drive.actionBuilder(new Pose2d(-35, -35, Math.toRadians(10)))
                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 440;
                    if (id_ap == 22) posicao_desejada_indexer = 0;
                    if (id_ap == 23) posicao_desejada_indexer = 0;

                })
                .strafeToLinearHeading(new Vector2d(-15, -40),Math.toRadians(8),new TranslationalVelConstraint(70))
                .waitSeconds(0.8)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 218;
                    if (id_ap == 22) posicao_desejada_indexer = 440;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })

                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.6)


                .afterTime(0, ()->{

                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 440;

                })

                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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

        rota8 = drive.actionBuilder(new Pose2d(-15, -40, Math.toRadians(20)))
                .turnTo(Math.toRadians(120))


                //.setReversed(true)

                //.splineTo(new Vector2d(47, -40),Math.toRadians(315))


                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota9 = drive.actionBuilder(new Pose2d(-15, -40, Math.toRadians(120)))
                .splineTo(new Vector2d(-47, -20),Math.toRadians(83),new TranslationalVelConstraint(50))
                .splineTo(new Vector2d(-50, 10),Math.toRadians(83),new TranslationalVelConstraint(20))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota10 = drive.actionBuilder(new Pose2d(-50, 10, Math.toRadians(90)))

                .setReversed(true)
                .splineTo(new Vector2d(-35, -35),Math.toRadians(190),new TranslationalVelConstraint(85))
                .afterTime(0, ()->{
                    PIDs_state = false;
                })


                .build();

        rota11 = drive.actionBuilder(new Pose2d(-35, -35, Math.toRadians(10)))


                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 218;
                    if (id_ap == 22) posicao_desejada_indexer = 0;
                    if (id_ap == 23) posicao_desejada_indexer = 0;

                })
                .strafeToLinearHeading(new Vector2d(-15, -40),Math.toRadians(8),new TranslationalVelConstraint(70))
                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })
//
                //
                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 440;

                })

                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


                .afterTime(0, ()->{

                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 440;
                    if (id_ap == 22) posicao_desejada_indexer = 440;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);
                    posicao_desejada_indexer = 0;
                })

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .strafeToLinearHeading(new Vector2d(-20, -40),Math.toRadians(180))

                .build();


    }

    public void Rotas_VGrande_6() {

        MecanumDrive.PARAMS.axialGain = 7.5;
        MecanumDrive.PARAMS.lateralGain = 40;
        MecanumDrive.PARAMS.headingGain = 36;
        MecanumDrive.PARAMS.maxWheelVel = 60;


        drive = new MecanumDrive(hardwareMap, new Pose2d(-42, -55,
                Math.toRadians(90)));

        rota1 = drive.actionBuilder(new Pose2d(-42, -55, Math.toRadians(90)))

                .strafeToLinearHeading(new Vector2d(-11, -40),Math.toRadians(90),new TranslationalVelConstraint(70))
                .waitSeconds(0.1)
                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota2 = drive.actionBuilder(new Pose2d(-11, -40, Math.toRadians(90)))

                .turnTo( Math.toRadians(20))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })

                .build();

        rota3 = drive.actionBuilder(new Pose2d(-11, -40, Math.toRadians(20)))



                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })
                .strafeToLinearHeading(new Vector2d(-15, -40),Math.toRadians(20))

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


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
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

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
        rota4 = drive.actionBuilder(new Pose2d(-15, -40, Math.toRadians(8)))


                .setReversed(true)
                .splineTo(new Vector2d(-48.5, -40),Math.toRadians(270))


                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota5 = drive.actionBuilder(new Pose2d(-48.5, -40, Math.toRadians(90)))
                .setReversed(false)
                .turnTo(Math.toRadians(85))

                .splineTo(new Vector2d(-48.5, -11.5),Math.toRadians(85),new TranslationalVelConstraint(45))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota6 = drive.actionBuilder(new Pose2d(-48.5, -11.5, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(-32, -32),Math.toRadians(182),new TranslationalVelConstraint(85))
                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota7 = drive.actionBuilder(new Pose2d(-35, -35, Math.toRadians(10)))
                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 440;
                    if (id_ap == 22) posicao_desejada_indexer = 0;
                    if (id_ap == 23) posicao_desejada_indexer = 0;

                })
                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 218;
                    if (id_ap == 22) posicao_desejada_indexer = 440;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();
                    PIDs_state = true;
                })

                .waitSeconds(0.7)


                .afterTime(0, ()->{

                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);

                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 440;

                })

                .waitSeconds(0.6)

                .afterTime(0, ()->{
                    Aciona_servo();

                    PIDs_state = true;
                })

                .waitSeconds(0.7)

                .afterTime(0, ()->{
                    Reservoir_1.setPosition(0);
                    Reservoir_2.setPosition(1);
                    Reservoir_3.setPosition(0);
                    posicao_desejada_indexer = 0;

                })

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .strafeToLinearHeading(new Vector2d(-20, -40),Math.toRadians(180))

                .build();


    }

    public void Aciona_servo(){
        if (Indexer.getCurrentPosition() <  100) Reservoir_1.setPosition(1);
        else if (Indexer.getCurrentPosition() > 300) Reservoir_3.setPosition(1);
        else Reservoir_2.setPosition(0);



    }








}
