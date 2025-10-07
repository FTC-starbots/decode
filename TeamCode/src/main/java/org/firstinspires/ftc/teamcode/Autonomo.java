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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

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
            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) fr.getFiducialId();


            return PIDs_state;
        }
    }

    private Limelight3A limelight;

    int posicaoDesejada_Braco, posicaoDesejada_Slide;

    boolean PIDs_state;

    DcMotor leftBack, leftFront, rightBack, rightFront;

    public int pattern[] = {0,0,0};
    public int id_ap;

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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");




        //--------Calcula Rotas e parametriza drivetrain---------------------
        Rotas();
        //------------------------------------------
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

        if (fiducialResults.get(0).getFiducialId() == 21 || fiducialResults.get(1).getFiducialId() == 21 || fiducialResults.get(2).getFiducialId() == 21) id_ap = 21;pattern[0] = 1;
        if (fiducialResults.get(0).getFiducialId() == 22 || fiducialResults.get(1).getFiducialId() == 22 || fiducialResults.get(2).getFiducialId() == 22) id_ap =22; pattern[1] = 1;
        if (fiducialResults.get(0).getFiducialId() == 23 || fiducialResults.get(1).getFiducialId() == 23 || fiducialResults.get(2).getFiducialId() == 23) id_ap = 23; pattern[2] = 1;
        waitForStart();
        PIDs_state = true;
        Actions.runBlocking(new ParallelAction(rota1,new PIDs()));

    }

    public void Rotas(){

        MecanumDrive.PARAMS.axialGain = 7.5;
        MecanumDrive.PARAMS.lateralGain = 40;
        MecanumDrive.PARAMS.headingGain = 36;
        MecanumDrive.PARAMS.maxWheelVel = 35;


        drive = new MecanumDrive(hardwareMap, new Pose2d(0, -65,
                Math.toRadians(90)));

        rota1 = drive.actionBuilder(new Pose2d(0, -65, Math.toRadians(90)))

                .strafeTo(new Vector2d(32.5, -39))

                .strafeTo(new Vector2d(-0, 20))

                .strafeTo(new Vector2d(-32.5, -35))

                .build();

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
    }
}