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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous


public class Autonomo extends LinearOpMode {

    public class PIDs implements Action{
        public PIDs(){
            //inicializa o braço
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            //executa o movimento do braço em loop

            int erroB;
            double KpB = 0.002;
            double potenciaB;

            int erroV;
            double KpV = 0.01;
            double KiV = 0.001;
            double potenciaV;

            int erroH;
            double KpH = 0.01;
            double potenciaH;

            erroB = posicaoDesejada_Braco - BracoOuttake.getCurrentPosition();
            potenciaB = Math.min(Math.max(KpB * erroB, -0.7), 0.7);
            BracoOuttake.setPower(potenciaB);
            //-------------------------

            //slide vertical PiD
            erroV = posicaoDesejada_Slide - slide_vertical_e.getCurrentPosition();
            double Ap = KpV * erroV;
            double Ai = 0;
            Ai = Ai + KiV * erroV;
            potenciaV = Math.min(Math.max(-(Ap + Ai), -1), 0.8);
            slide_vertical_d.setPower(potenciaV);
            slide_vertical_e.setPower(potenciaV);
            //-------------------------

            erroH = - slide_horizontal.getCurrentPosition();
            potenciaH = Math.min(Math.max((KpH * erroH), -0.8), 0.8);
            slide_horizontal.setPower(potenciaH);

            return PIDs_state;
        }
    }


    int posicaoDesejada_Braco, posicaoDesejada_Slide;

    boolean PIDs_state;


    DcMotor leftBack, leftFront, rightBack, rightFront,
            slide_vertical_d, slide_vertical_e, BracoOuttake, slide_horizontal;

    Servo GarraIntake, GarraOuttake, BracoIntake, PulsoIntake,  RotacaoOuttake, RotacaoIntake;

    DigitalChannel FimDeCurso;

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
        slide_vertical_d = hardwareMap.get(DcMotorEx.class, "slide_vertical_d");
        slide_vertical_e = hardwareMap.get(DcMotorEx.class, "slide_vertical_e");
        slide_horizontal = hardwareMap.get(DcMotorEx.class, "slide_horizontal");
        BracoOuttake = hardwareMap.get(DcMotorEx.class, "BracoOuttake");

        BracoIntake = hardwareMap.get(Servo.class, "BracoIntake");
        GarraIntake = hardwareMap.get(Servo.class, "GarraIntake");
        GarraOuttake = hardwareMap.get(Servo.class, "GarraOuttake");
        PulsoIntake = hardwareMap.get(Servo.class, "PulsoIntake");
        RotacaoOuttake = hardwareMap.get(Servo.class, "RotacaoOuttake");
        RotacaoIntake = hardwareMap.get(Servo.class,"RotacaoIntake");

        FimDeCurso = hardwareMap.get(DigitalChannel.class, "FimDeCursoSV");
        // limelight = hardwareMap.get(Limelight3A.class, "limelight");



        slide_vertical_d.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_vertical_d.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide_vertical_d.setDirection(DcMotor.Direction.REVERSE);
        slide_vertical_d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide_vertical_e.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_vertical_e.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide_vertical_e.setDirection(DcMotor.Direction.FORWARD);
        slide_vertical_e.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide_horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide_horizontal.setDirection(DcMotor.Direction.FORWARD);
        slide_horizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BracoOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BracoOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BracoOuttake.setDirection(DcMotor.Direction.REVERSE);
        BracoOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //--------Calcula Rotas e parametriza drivetrain---------------------
        Rotas();
        //------------------------------------------
        GarraOuttake.setPosition(1.0);
        RotacaoIntake.setPosition(0.45);
        BracoIntake.setPosition(1);
        PulsoIntake.setPosition(0.25);
        RotacaoOuttake.setPosition(0.5);
        waitForStart();
        PIDs_state = true;
        Actions.runBlocking(new ParallelAction(rota1,new PIDs()));
        GarraOuttake.setPosition(1);
        sleep(305);

        PIDs_state = true;
        Actions.runBlocking(new ParallelAction(rota4,new PIDs()));
        sleep(200);

        PIDs_state = true;
        Actions.runBlocking(new ParallelAction(rota5,new PIDs()));
        GarraOuttake.setPosition(1);
        sleep(305);

        PIDs_state = true;
        Actions.runBlocking(new ParallelAction(rota6,new PIDs()));
        sleep(200);

        PIDs_state = true;
        Actions.runBlocking(new ParallelAction(rota7,new PIDs()));
        GarraOuttake.setPosition(1);
        sleep(305);

        PIDs_state = true;
        Actions.runBlocking(new ParallelAction(rota8,new PIDs()));
        sleep(200);

        PIDs_state = true;
        Actions.runBlocking(new ParallelAction(rota9,new PIDs()));
        PIDs_state = true;


    }

    public void Rotas(){

        MecanumDrive.PARAMS.axialGain = 7.5;
        MecanumDrive.PARAMS.lateralGain = 40;
        MecanumDrive.PARAMS.headingGain = 36;
        MecanumDrive.PARAMS.maxWheelVel = 35;


        drive = new MecanumDrive(hardwareMap, new Pose2d(0, -65,
                Math.toRadians(270)));

        rota1 = drive.actionBuilder(new Pose2d(0, -65, Math.toRadians(270)))
                .afterTime(0,()->{
                    RotacaoOuttake.setPosition(0);
                    posicaoDesejada_Braco = 240;
                    posicaoDesejada_Slide = 1150;
                })
                .strafeToLinearHeading(new Vector2d(-0.5, -27), Math.toRadians(270))
                .afterTime(0,()->{
                    GarraOuttake.setPosition(0.3);

                })
                .waitSeconds(0.175)
                .setTangent(Math.toRadians(270))//trajetória de frente
                .splineTo(new Vector2d(20, -38), Math.toRadians(0))
                .afterTime(0,()->{
                    posicaoDesejada_Slide = 100;
                    posicaoDesejada_Braco = 0;
                    RotacaoOuttake.setPosition(0.5);

                })

                .splineTo(new Vector2d(34, -28), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(34, -22), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(46, -21), Math.toRadians(270))
               .afterTime(0,()->{
                    posicaoDesejada_Braco = 680;

                })
                .splineToConstantHeading(new Vector2d(47, -60), Math.toRadians(270),new TranslationalVelConstraint(50),  new ProfileAccelConstraint(-46, 60))
                .afterTime(0,()->{
                    posicaoDesejada_Braco = 0;

                })
                //volta e empurra o segundo sample

                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(46, -24), Math.toRadians(90), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-42, 56))
                .splineToConstantHeading(new Vector2d(55, -20), Math.toRadians(270))
                .afterTime(0,()->{
                    posicaoDesejada_Braco = 0;

                })
                .splineToConstantHeading(new Vector2d(58, -58), Math.toRadians(270), new TranslationalVelConstraint(35), new ProfileAccelConstraint(-46, 56))
                .splineToConstantHeading(new Vector2d(37, -60), Math.toRadians(270), new TranslationalVelConstraint(18.5), new ProfileAccelConstraint(-46, 56))
                .splineToConstantHeading(new Vector2d(37, -70.5), Math.toRadians(270), new TranslationalVelConstraint(18.5), new ProfileAccelConstraint(-46, 56))
                .afterTime(0,()->{


                })
                //volta e empurra o terceiro sample

                .afterTime(0,()->{
                    GarraOuttake.setPosition(0.9);
                    PIDs_state = false;
                })
                .build();

        //clipa o segundo clipe
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
                .build();
    }
}