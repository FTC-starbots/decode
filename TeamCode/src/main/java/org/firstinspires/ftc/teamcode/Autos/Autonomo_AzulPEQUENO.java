package org.firstinspires.ftc.teamcode.Autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;

@Config
@Autonomous


public class Autonomo_AzulPEQUENO extends Rotas {
    private static final Canvas DEFAULT_FIELD = new Canvas();
    public  class PIDs implements Action {
        public PIDs() {

            cicle_time = deltaTime.milliseconds();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            //executa o movimento do braço em loop
            double erro = velocidade_desejada + Atirador_1.getVelocity();

            double kp = 0.003;
            double ki = 0.0000001;
            double kd = 0.003;
            double kf = 0.05;

            if (Math.abs(erro)<10){
                kp = kp/500;
                ki = ki/500;
                kd = kd/500;
                kf = kf/500;

            }

            double BatVoltage = myControlHubVoltageSensor.getVoltage();
            double feedfoward = 1 + (kf * (12.5-BatVoltage));
            double integrativo = ki * (erro * cicle_time);
            double derivativo = kd * ((erro - lasterror_PiDF_Shooter) / cicle_time);
            double proporcional = kp * erro;

            integral_PiDF_Shooter += integrativo;
            lasterror_PiDF_Shooter = erro;

            if (integral_PiDF_Shooter>0.5) integral_PiDF_Shooter=0.5;
            else if (integral_PiDF_Shooter<-0.5) integral_PiDF_Shooter=-0.5;




            double power_correction = (proporcional + integral_PiDF_Shooter + derivativo ) * feedfoward;

            if ( power_correction > 1) power_correction = 1;
            else if (power_correction < 0) power_correction = 0;

if(erro < 0) power_correction = 0;
            if (velocidade_desejada == 0 ) Atirador_1.setPower(0);
            else Atirador_1.setPower( power_correction );

            Atirador_2.setPower(-Atirador_1.getPower());







            double potencia_indexer;
            double erro1 = posicao_desejada_indexer - Indexer.getCurrentPosition();
            // Acumulando o erro para o integral
            double proporcional1 =  erro1 * 0.028;


            double integrativo1 = 0.02 * (erro1 * cicle_time);

            double derivativo1 = 1 * ((erro - lasterror_PiDF_Indexer) / cicle_time);
            integral_Indexer += integrativo1;

            lasterror_PiDF_Indexer = erro;

            if (erro<0 && integral_Indexer >  0)integral_Indexer = 0;
            if (erro>0 && integral_Indexer <  0)integral_Indexer = 0;
            if (integral_Indexer>0.15) integral_Indexer=0.05;
            else if (integral_Indexer<-0.15) integral_Indexer=-0.05;

            potencia_indexer =(proporcional1 + integral_Indexer + derivativo1 ) * feedfoward;


            if ( potencia_indexer > 0.4) potencia_indexer = 0.4;
            else if (potencia_indexer < -0.4) potencia_indexer = -0.4;

            Indexer.setPower(potencia_indexer);

            telemetry.addData("Velocidade atirador", Atirador_1.getVelocity());
            telemetry.addData("erro speed", erro);
            telemetry.addData("potencia atirador", power_correction);
            telemetry.addData("OFFSET", tx_offset);


            // Atualiza os dados na tela*/
            telemetry.update();


            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                tx_offset = fr.getTargetXPixels();


                if ((limelight.getLatestResult().isValid()) && (fr.getFiducialId() == 24 || fr.getFiducialId() == 20)) {

                    if (tx_offset < 1190) {
                        leftBack.setPower(-0.165);
                        leftFront.setPower(-0.165);
                        rightBack.setPower(0.165);
                        rightFront.setPower(0.165);

                    } else if (tx_offset > 1390) {
                        leftBack.setPower(0.165);
                        leftFront.setPower(0.165);
                        rightBack.setPower(-0.165);
                        rightFront.setPower(-0.165);
                    } else {
                        leftBack.setPower(0);
                        leftFront.setPower(0);
                        rightBack.setPower(0);
                        rightFront.setPower(0);
                    }
                }
            }

                return PIDs_state;
        }
    }





    @Override
    public void runOpMode() throws InterruptedException {

        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
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

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Atirador_1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Atirador_2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Atirador_1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Atirador_2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Indexer.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        timer_shooter   = new ElapsedTime();
        deltaTime = new ElapsedTime();

        timer_shooter.reset();
        deltaTime.reset();

        limelight.start();
        limelight.pipelineSwitch(5);
        //--------Calcula Rotas e parametriza drivetrain---------------------

        Rotas_APequeno_Perto();
        //------------------------------------------
        //Leitura de AP
        {
            LLResult result2 = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducialResults2 = result2.getFiducialResults();

            for (LLResultTypes.FiducialResult fr : fiducialResults2) {
                id_ap = fr.getFiducialId();
            }
            if (id_ap == 21) {
                Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (id_ap == 22) {
                Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

            } else if (id_ap == 23) {
                Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else {
                id_ap = 21;
                Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            }
        }
        waitForStart();

        Auto_APequeno_Perto();
    }

    public void Auto_APequeno_Perto() {

        PIDs_state = false;


        Actions.runBlocking(rota1);
        //Leitura de AP

        {
            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                id_ap = fr.getFiducialId();
            }
            if (id_ap == 21) {
                Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (id_ap == 22) {
                Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

            } else if (id_ap == 23) {
                Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);

        }
        limelight.stop();
        Actions.runBlocking(rota2);

        velocidade_desejada = 1170;
        limelight.start();
        limelight.pipelineSwitch(5);

        PIDs_state = true;
        Actions.runBlocking(new ParallelAction(rota3, new PIDs()));
        limelight.stop();
        Intake.setPower(1);

        Atirador_1.setPower(0);
        Atirador_2.setPower(0);

        PIDs_state = false;
        Actions.runBlocking(rota4);

        Actions.runBlocking(rota5);

        Actions.runBlocking(rota6);

        Intake.setPower(0);


        velocidade_desejada = 1170;
        limelight.start();
        limelight.pipelineSwitch(5);

        PIDs_state = true;
        Actions.runBlocking(new ParallelAction(rota7, new PIDs()));
        limelight.stop();
        Intake.setPower(1);

        Atirador_1.setPower(0);
        Atirador_2.setPower(0);

        PIDs_state = false;
        Actions.runBlocking(rota8);

        Actions.runBlocking(rota9);

        Actions.runBlocking(rota10);

        Intake.setPower(0);


        velocidade_desejada = 1180;
        limelight.start();
        limelight.pipelineSwitch(5);


        PIDs_state = true;
        Actions.runBlocking(new ParallelAction(rota11, new PIDs()));

    }

    public void Auto_APequeno_Longe() {

        PIDs_state = false;
        //Leitura de AP
        {
            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                id_ap = fr.getFiducialId();
            }
            if (id_ap == 21) {
                Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (id_ap == 22) {
                Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

            } else if (id_ap == 23) {
                Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);

        }

        Actions.runBlocking(rota1);


        velocidade_desejada = 1165;

        PIDs_state = true;
        Actions.runBlocking(new ParallelAction(rota2, new PIDs()));

        Atirador_1.setPower(0);
        Atirador_2.setPower(0);

        PIDs_state = false;
        Actions.runBlocking(rota3);

        Intake.setPower(1);

        Actions.runBlocking(rota4);

        Intake.setPower(0);

        Actions.runBlocking(rota5);


        velocidade_desejada = 1165;

        PIDs_state = true;
        Actions.runBlocking(new ParallelAction(rota6, new PIDs()));

        Atirador_1.setPower(0);
        Atirador_2.setPower(0);

        PIDs_state = false;
        Actions.runBlocking(rota7);

        Intake.setPower(1);

        Actions.runBlocking(rota8);

        Intake.setPower(0);

        Actions.runBlocking(rota9);


        velocidade_desejada = 1165;

        PIDs_state = true;
        Actions.runBlocking(new ParallelAction(rota10, new PIDs()));



    }

    public void Auto_AGrande() {

        PIDs_state = false;


        Actions.runBlocking(rota1);
        //Leitura de AP

        {
            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                id_ap = fr.getFiducialId();
            }
            if (id_ap == 21) {
                Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (id_ap == 22) {
                Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

            } else if (id_ap == 23) {
                Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);

        }


        velocidade_desejada = 1165;

        PIDs_state = true;
        Actions.runBlocking(new ParallelAction(rota2, new PIDs()));



        Atirador_1.setPower(0);
        Atirador_2.setPower(0);

        PIDs_state = false;
        Actions.runBlocking(rota3);

        Intake.setPower(1);
        Actions.runBlocking(rota4);
        Intake.setPower(0);
        Actions.runBlocking(rota5);


        velocidade_desejada = 1165;

        PIDs_state = true;
        Actions.runBlocking(new ParallelAction(rota6, new PIDs()));



        Atirador_1.setPower(0);
        Atirador_2.setPower(0);

        PIDs_state = false;
        Actions.runBlocking(rota7);
        Intake.setPower(1);
        Actions.runBlocking(rota8);
        Intake.setPower(0);
        Actions.runBlocking(rota9);




        velocidade_desejada = 1180;

        PIDs_state = true;
        Actions.runBlocking(new ParallelAction(rota10, new PIDs()));

    }



}
