package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.List;

@Config
@TeleOp
public class TeleOperado extends Autonomo {

    boolean Movimento_robo = false, Base_Motriz_State = false, Shoot = false, gamepad_1 = true;
    boolean Iniciar_atirador = false, Corrigir_velocidade = false, detecting_ap = false, goal_ap = false;

    private Limelight3A limelight;

    private DcMotorEx leftBack, leftFront, rightBack, rightFront, Intake, Atirador_1, Atirador_2, Indexer;

    private Servo Reservoir_2, Reservoir_1, Reservoir_3;

    private RevBlinkinLedDriver Led_Strip;

    private VoltageSensor myControlHubVoltageSensor;
    int posicao_desejada_indexer = 0, reservatorio = 0, reservatorio_1, reservatorio_2, reservatorio_3;

    double controle_rotacao, Speed, angulo_desejado, angulo_robo, vectorX, vectorY, correcao_angular, velocidade_desejada = 0, kf_atirar,kp_atirar, alpha, kp_ap, ki_ap;

    boolean areapermitida;

    private VoltageSensor ControlHub_VoltageSensor;

    double T_max = 0;
    double T_min = 1000;
    double T_atual = 0;

    //variáveis dos controladores PiDF____________________________
    double integral_PiDF_Correcao_Angular = 0;
    double lasterror_PiDF_Correcao_Angular = 0;

    //____________________________________________________________

    double cicle_time = 0;

    Pose2d pose_atual;

    Autonomo auto = new Autonomo();

    double CorrTx,TxOffset, ErroMovX, ErroMovY, limelight_posX, limelight_posY, limelight_yaw, last_pose_X, last_pose_Y, goal_X, goal_Y = 54, Ai_shoot = 0, hip;
    ElapsedTime RotacaoRobo, Tempo_de_ciclo,
            MASTER_TIMER, timer_shooter, deltaTime;


    @Override
    public void runOpMode() throws InterruptedException {
        // Mapeamento dos sensores e motores
        {
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


        }
        // Configuração dos motores da base
        {
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

            // Configuração dos motores adicionais
        }
        // Inicialização dos temporizadores
        {
            RotacaoRobo     = new ElapsedTime();
            Tempo_de_ciclo  = new ElapsedTime();
            MASTER_TIMER    = new ElapsedTime();
            timer_shooter   = new ElapsedTime();
            deltaTime       = new ElapsedTime();

            timer_shooter   .reset();
            RotacaoRobo     .reset();
            Tempo_de_ciclo  .reset();
            deltaTime       .reset();
        }
        Speed = 1;
        angulo_desejado = 0;
        posicao_desejada_indexer = 0;

        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        boolean blue_alliance = true; //false = red
        if (blue_alliance) {
            goal_X = -56.5;
        } else {
            goal_X = 56.5;
        }
        // Aguardando o início da execução
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(180)));
        limelight.start();
        limelight.pipelineSwitch(5);

        waitForStart();

        MASTER_TIMER.reset();

        // Execução principal do OpMode
        if (opModeIsActive()) {

            cicle_time = deltaTime.milliseconds();
            deltaTime.reset();
            angulo_desejado = 0;
            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

                //MOVIMENTO______________________________________________________________________________
                //Cálculo de posição e correção----------------------

                if (limelight.getLatestResult().isValid()) {
                    detecting_ap = true;
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {

                        if ((limelight.getLatestResult().isValid()) && (fr.getFiducialId() == 24 || fr.getFiducialId() == 20)){
                            TxOffset = fr.getTargetXPixels();
                            goal_ap = true;
                            limelight_posY = -result.getBotpose().getPosition().x * 35.3;
                            limelight_posX = result.getBotpose().getPosition().y * 35.3;
                            limelight_yaw = -result.getBotpose().getOrientation().getYaw() + 180;
                            drive = new MecanumDrive(hardwareMap, new Pose2d(limelight_posX, limelight_posY, Math.toRadians(limelight_yaw)));
                        }
                        else goal_ap =false;
                    }
                }else{
                    TxOffset = 0;
                    detecting_ap = false;
                }
                drive.updatePoseEstimate();

                pose_atual = drive.localizer.getPose();
                if (gamepad_1){
                if (Math.abs(gamepad1.left_stick_x) > 0.085 || Math.abs(gamepad1.left_stick_y) > 0.085) {
                    last_pose_X = pose_atual.position.x;
                    last_pose_Y = pose_atual.position.y;
                    Movimento_robo = true;
                } else {
                    Movimento_robo = false;
                }
                }else{
                    if (Math.abs(gamepad2.left_stick_x) > 0.085 || Math.abs(gamepad2.left_stick_y) > 0.085) {
                        last_pose_X = pose_atual.position.x;
                        last_pose_Y = pose_atual.position.y;
                        Movimento_robo = true;
                    } else {
                        Movimento_robo = false;
                    }

                }

                if (Movimento_robo == false) {
                    ErroMovX = pose_atual.position.x - last_pose_X;
                    ErroMovY = pose_atual.position.y - last_pose_Y;
                } else {
                    ErroMovX = 0;
                    ErroMovY = 0;
                }



                Base_Motriz();
                Indexer();
                Shooting_Vel();
            if (gamepad_1) {
                if (gamepad1.options) {
                    angulo_robo = 0;
                    last_pose_X = pose_atual.position.x;
                    last_pose_Y = pose_atual.position.y;
                }
            }else{
                if (gamepad2.options) {
                    angulo_robo = 0;
                    last_pose_X = pose_atual.position.x;
                    last_pose_Y = pose_atual.position.y;
                }

            }
                //Cálculo de alinhamento com o alvo
                double cateto_a = goal_X - pose_atual.position.x;
                double cateto_b = goal_Y - pose_atual.position.y;
                hip = Math.sqrt(Math.pow(cateto_a, 2) + Math.pow(cateto_b, 2));

                alpha =  Math.toDegrees(Math.atan2(-cateto_b , cateto_a));
                //_______________________________________________________________________________
                controle_rotacao = 0.5;
                //-----------------------------------------------------------------------------------------------
                if ((pose_atual.position.y >= Math.abs(pose_atual.position.x)) || (pose_atual.position.y <= -Math.abs(pose_atual.position.x) - 40)) {
                    areapermitida = true;
                    if (gamepad2.triangle){
                        gamepad_1 = false;
                        angulo_desejado = alpha;
                        velocidade_desejada = 787.87 + 5.365 * hip;
                    }else{
                        kf_atirar = 0;
                        kp_atirar = 0;
                        CorrTx = 0;
                        velocidade_desejada = 0;
                    }

                } else {
                    areapermitida = false;
                    Ai_shoot = 0;
                    velocidade_desejada = 0;
                }

                    if (gamepad1.cross) Intake.setPower(1);
                    else if (gamepad1.circle) Intake.setPower(-1);
                    else Intake.setPower(0);


                if (gamepad2.cross) gamepad_1 = false;
                else if (gamepad1.square) gamepad_1 = true;
//--------------------Reservatorios

                if(gamepad2.dpad_left&& areapermitida) {
                    timer_shooter.reset();
                    Shoot(1);
                }

                if(gamepad2.dpad_up&& areapermitida) {
                    timer_shooter.reset();
                    Shoot(2);

                }

                if(gamepad2.dpad_right&& areapermitida) {
                    timer_shooter.reset();
                    Shoot(3);

                }

                if (areapermitida && velocidade_desejada != 0 && (velocidade_desejada > Math.abs(Atirador_1.getVelocity())-40 && velocidade_desejada < Math.abs(Atirador_1.getVelocity())+40)) {
                    if (detecting_ap) Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
                    else Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                }
                else if(areapermitida){
                    if (detecting_ap) Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
                    else Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                }
                else {
                    if (detecting_ap) Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
                    else Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                }

                if(Shoot) Shoot_single();
//-----------------------------------------

                Telemetria();
                Tempo_de_ciclo.reset();
            }
        }
    }

    //-------FUNÇÕES DE MOVIMENTO-----------------------------------------------------------------------------------------
    private double PiDF_Correcao_Angular(double erro, double kp, double ki, double kd, double kf, double min, double max){

        double BatVoltage = myControlHubVoltageSensor.getVoltage();
        double feedfoward = 1 + (kf * (12.5-BatVoltage));
        double integrativo = ki * (erro * cicle_time);
        double derivativo = kd * ((erro - lasterror_PiDF_Correcao_Angular) / cicle_time);
        double proporcional = kp * erro;

        integral_PiDF_Correcao_Angular += integrativo;
        lasterror_PiDF_Correcao_Angular = erro;

        if (Math.abs(erro)<0.05) integral_PiDF_Correcao_Angular = 0;
        if (integral_PiDF_Correcao_Angular>0.15) integral_PiDF_Correcao_Angular=0.15;
        else if (integral_PiDF_Correcao_Angular<-0.15) integral_PiDF_Correcao_Angular=-0.15;

        double output = (proporcional + integral_PiDF_Correcao_Angular + derivativo ) * feedfoward;

        if ( output > max) output = max;
        else if (output < min) output = min;

        return output;

    }

    public double PiDF_Vel(double erro, double kp, double ki, double kd, double kf, double min, double max){

        double BatVoltage = myControlHubVoltageSensor.getVoltage();
        double feedfoward = 1 + (kf * (12.5-BatVoltage));
        double integrativo = ki * (erro * cicle_time);
        double derivativo = kd * ((erro - lasterror_PiDF_Correcao_Angular) / cicle_time);
        double proporcional = kp * erro;

        integral_PiDF_Correcao_Angular += integrativo;
        lasterror_PiDF_Correcao_Angular = erro;

        if (Math.abs(erro)<0.05) integral_PiDF_Correcao_Angular = 0;
        if (integral_PiDF_Correcao_Angular>0.15) integral_PiDF_Correcao_Angular=0.15;
        else if (integral_PiDF_Correcao_Angular<-0.15) integral_PiDF_Correcao_Angular=-0.15;

        double output = (proporcional + integral_PiDF_Correcao_Angular + derivativo ) * feedfoward;

        if ( output > max) output = max;
        else if (output < min) output = min;

        return output;

    }
    private void Base_Motriz() {
        angulo_robo = (((rightFront.getCurrentPosition() - leftBack.getCurrentPosition()) / 370.0) % 360);

        double divisorX = 5 + 10 * Math.max(0, 1.0 - (Math.abs(ErroMovX) / 4.0));
        double divisorY = 5 + 10 * Math.max(0, 1.0 - (Math.abs(ErroMovY) / 4.0));
    if (gamepad_1) {
        vectorX = (Math.cos(-angulo_robo / 180 * Math.PI) * (gamepad1.left_stick_x/*+ (ErroMovY/divisorY)*/))
                + Math.sin(-angulo_robo / 180 * Math.PI) * (-gamepad1.left_stick_y/*+(-ErroMovX/divisorX)*/);

        vectorY = ((-Math.sin(-angulo_robo / 180 * Math.PI) * (gamepad1.left_stick_x/*+ (ErroMovY/divisorY)*/))
                + Math.cos(-angulo_robo / 180 * Math.PI) * (-gamepad1.left_stick_y /*+(-ErroMovX/divisorX)*/));
    }else{
        vectorX = (Math.cos(-angulo_robo / 180 * Math.PI) * (gamepad2.left_stick_x/*+ (ErroMovY/divisorY)*/))
                + Math.sin(-angulo_robo / 180 * Math.PI) * (-gamepad2.left_stick_y/*+(-ErroMovX/divisorX)*/);

        vectorY = ((-Math.sin(-angulo_robo / 180 * Math.PI) * (gamepad2.left_stick_x/*+ (ErroMovY/divisorY)*/))
                + Math.cos(-angulo_robo / 180 * Math.PI) * (-gamepad2.left_stick_y /*+(-ErroMovX/divisorX)*/));

    }

        if (gamepad_1) {
            leftBack.setPower((vectorY * 1 + vectorX * -1) * Speed + ((gamepad1.right_trigger - gamepad1.left_trigger) + correcao_angular));
            leftFront.setPower((vectorY * 1 + vectorX * 1) * Speed + ((gamepad1.right_trigger - gamepad1.left_trigger) + correcao_angular));
            rightBack.setPower((vectorY * 1 + vectorX * 1) * Speed - ((gamepad1.right_trigger - gamepad1.left_trigger) + correcao_angular));
            rightFront.setPower((vectorY * 1 + vectorX * -1) * Speed - ((gamepad1.right_trigger - gamepad1.left_trigger) + correcao_angular));
        }else{
            leftBack.setPower((vectorY * 1 + vectorX * -1) * Speed + ((gamepad2.right_trigger - gamepad2.left_trigger) + correcao_angular));
            leftFront.setPower((vectorY * 1 + vectorX * 1) * Speed + ((gamepad2.right_trigger - gamepad2.left_trigger) + correcao_angular));
            rightBack.setPower((vectorY * 1 + vectorX * 1) * Speed - ((gamepad2.right_trigger - gamepad2.left_trigger) + correcao_angular));
            rightFront.setPower((vectorY * 1 + vectorX * -1) * Speed - ((gamepad2.right_trigger - gamepad2.left_trigger) + correcao_angular));
        }
        int rotate;

        if (angulo_robo >= 0) {
            rotate = 360;
        } else {
            rotate = -360;
        }
        double corr_1 = angulo_robo - angulo_desejado;
        double corr_2 = angulo_desejado + rotate - angulo_robo;
        double correcaoP;
        if (Math.abs(corr_1) > Math.abs(corr_2)) correcaoP = corr_2;
        else correcaoP = -corr_1;

        if (RotacaoRobo.seconds() < 1) {
            correcao_angular = 0;
            angulo_desejado = angulo_robo;
            last_pose_X = pose_atual.position.x;
            last_pose_Y = pose_atual.position.y;
        } else {

            if (goal_ap && gamepad2.triangle && TxOffset != 0) correcaoP = (TxOffset-512)/40;

            correcao_angular = PiDF_Correcao_Angular(correcaoP, 0.0135 /*+ kp_atirar*/,0.000004, -0.00, 0.15 + kf_atirar,-0.7,0.7);

        }


        // Ajuste de velocidade baseado nos triggers do gamepad

        if(gamepad_1) {
            if (Math.abs(gamepad1.left_trigger) > 0.1 || Math.abs(gamepad1.right_trigger) > 0.1 || gamepad1.dpad_left) {
                angulo_desejado = angulo_robo;
                RotacaoRobo.reset();

                Speed = 1 - 0.5 * Math.abs(gamepad1.left_trigger - gamepad1.right_trigger);
            } else if (gamepad1.start) {
                // Resetando os encoders dos motores
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                angulo_desejado = 0;
            } else {
                // Ajustando a velocidade com base na correção angular
                Speed = 1.2 - Math.abs(correcao_angular) - (ControlHub_VoltageSensor.getVoltage() - 11) / 10;
//
            }
        }else{
            if (Math.abs(gamepad2.left_trigger) > 0.1 || Math.abs(gamepad2.right_trigger) > 0.1 || gamepad2.dpad_left) {
                angulo_desejado = angulo_robo;
                RotacaoRobo.reset();

                Speed = 1 - 0.5 * Math.abs(gamepad2.left_trigger - gamepad2.right_trigger);
            } else if (gamepad2.start) {
                // Resetando os encoders dos motores
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                angulo_desejado = 0;
            } else {
                // Ajustando a velocidade com base na correção angular
                Speed = 1.2 - Math.abs(correcao_angular) - (ControlHub_VoltageSensor.getVoltage() - 11) / 10;
//
            }

        }

        // Reduzindo a velocidade caso o motor `slide_vertical_e` ultrapasse uma posição

    } // OK

    private void Indexer() {
        double potencia_indexer;
        double erro = posicao_desejada_indexer - Indexer.getCurrentPosition();
        // Acumulando o erro para o integral
        potencia_indexer =  PiDF_Vel(erro,0.0085,0.0005,0,0.15,-0.6,0.6);
        Indexer.setPower(potencia_indexer);
    }

    private void Shooting_Vel() {
        double power_correction;
        double erro = velocidade_desejada + Atirador_1.getVelocity();
        power_correction =  PiDF_Vel(erro,0.005,0.015,0,0.15,-1,1);


        if (velocidade_desejada == 0 ) Atirador_1.setPower(0);
        else Atirador_1.setPower(1 + power_correction );
        Atirador_2.setPower(-Atirador_1.getPower());
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

    private void Shoot(int padrao){
        //posições individuais--------------------------------
        // Esquerda
        if (padrao == 1){
            reservatorio = 1;
            Shoot = true;
        }
        //Meio
        if (padrao == 2) {
            reservatorio = 2;
            Shoot = true;
        }
        //Direita
        if (padrao == 3){
            reservatorio = 3;
            Shoot = true;
        }
        //verde mais próxima -------------------------------
        if (padrao == 3);


        //roxa mais próxima --------------------------------
        if (padrao == 4);
        //todas o mais rapido
        if (padrao == 5){


        };
    }

    private void SensorCor(){



    }
    private void Telemetria(){

        telemetry.addData("Área de lançamento", areapermitida);

        telemetry.addData("","");

        telemetry.addData("posicao indexador", Indexer.getCurrentPosition());
        telemetry.addData("potencia indexador", Indexer.getPower());
        telemetry.addData("posicao desejada", posicao_desejada_indexer);
        telemetry.addData("hipotenusa", hip);
        telemetry.addData("alpha", alpha);



        telemetry.addData("","");

        telemetry.addData("Offset Tx", (TxOffset-512));

        telemetry.addData("","");

        telemetry.addData("Atirador1_vel", Atirador_1.getVelocity());
        telemetry.addData("Atirador1_power", Atirador_1.getPower());
        telemetry.addData("Atirador1_powdesejada",velocidade_desejada);

        telemetry.addData("Corrigir_velocidade", Corrigir_velocidade);
        telemetry.addData("Iniciar_atirador", Iniciar_atirador);






        telemetry.addData("","");

       // telemetry.addData("Posição", pose);
        telemetry.addData("x", pose_atual.position.x);
        telemetry.addData("y", pose_atual.position.y);

        telemetry.addData("lastx", last_pose_X);
        telemetry.addData("lasty", last_pose_Y);

        telemetry.addData("limex", limelight_posX);
        telemetry.addData("limey", limelight_posY);

        telemetry.addData("","");

        telemetry.addData("Velocidade: ",Speed);

        telemetry.addData("Base motriz função: ", Base_Motriz_State);
        telemetry.addData("Tempo de ciclo:", "");


        T_atual  = Tempo_de_ciclo.milliseconds();
        if (T_atual>T_max) T_max = T_atual;
        if (T_atual<T_min) T_min = T_atual;
        if (MASTER_TIMER.seconds()%5<1){
            T_max = 0;
            T_min = 9999;
        }

        telemetry.addData("Atual:", T_atual);
        telemetry.addData("Max:", T_max);
        telemetry.addData("Mín:", T_min);
        telemetry.addData("", "");

        telemetry.addData("**Motores**", "");
        telemetry.addData("Motor LB (Esquerda Traseira)", leftBack.getCurrentPosition());
        telemetry.addData("Motor LF (Esquerda Dianteira)", leftFront.getCurrentPosition());
        telemetry.addData("Motor RB (Direita Traseira)", rightBack.getCurrentPosition());
        telemetry.addData("Motor RF (Direita Dianteira)", rightFront.getCurrentPosition());
        telemetry.addData("", "");

        telemetry.addData("**Outros**", "");
        telemetry.addData("Ângulo Robô", angulo_robo);
        telemetry.addData("Ângulo Lime", limelight_yaw);
        telemetry.addData("Ângulo Desejado", angulo_desejado);
        telemetry.addData("Correç~caoa Angularo", correcao_angular);
        telemetry.addData("erro angulo", angulo_desejado-angulo_robo);
        telemetry.addData("X", vectorX);
        telemetry.addData("Y", vectorY);
        telemetry.addData("", "");



        // Atualiza os dados na tela*/
        telemetry.update();
    }

}
