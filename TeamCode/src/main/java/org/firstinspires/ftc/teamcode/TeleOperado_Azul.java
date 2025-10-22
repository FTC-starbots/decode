package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Config
@TeleOp


public class TeleOperado_Azul extends Autonomo_VermelhoGRANDE {

    boolean Movimento_robo = false, Base_Motriz_State = false, Shoot = false, gamepad_1 = true, atualizar_limelight = false;
    boolean Iniciar_atirador = false, Corrigir_velocidade = false, detecting_ap = false, goal_ap = false;

public static double /*kpt= 0.0005,kit = 0.25,kdt = 20 ,kft = 0.15,kot= 100,mint = 0,maxt = 1,*/ Const = 787.87, Variable = 5.365;
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
    double integral_PiDF_Correcao_Angular = 0, integral_Indexer = 0, integral_PiDF_Shooter = 0;
    double lasterror_PiDF_Correcao_Angular = 0, lasterror_PiDF_Shooter = 0, lasterror_PiDF_Indexer = 0;

    //____________________________________________________________

    double cicle_time = 0, cicle_nano, osc_time_ca, osc_time_ind;

    double[] amortecimento = new double[3];




    Pose2d pose_atual;
    double distLime;

    double CorrTx,TxOffset, ErroMovX, ErroMovY, limelight_posX, limelight_posY, limelight_yaw, last_pose_X, last_pose_Y, goal_X, goal_Y = 55.630, Ai_shoot = 0, hip;
    ElapsedTime RotacaoRobo, Tempo_de_ciclo,
            MASTER_TIMER, timer_shooter, deltaTime, osc_pos_time_ca, osc_neg_time_ca, osc_pos_time_ind, osc_neg_time_ind;


    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();




    @Override
    public void runOpMode() throws InterruptedException {
        // Mapeamento dos sensores e motores
        {
            myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
            ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

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
            osc_pos_time_ca = new ElapsedTime();
            osc_neg_time_ca = new ElapsedTime();
            osc_pos_time_ind = new ElapsedTime();
            osc_neg_time_ind = new ElapsedTime();

            timer_shooter   .reset();
            RotacaoRobo     .reset();
            Tempo_de_ciclo  .reset();
            deltaTime       .reset();
            osc_pos_time_ca.reset();
            osc_neg_time_ca.reset();
            osc_pos_time_ind.reset();
            osc_neg_time_ind.reset();

        }

        Speed = 1;
        angulo_desejado = 0;
        posicao_desejada_indexer = 0;



        boolean blue_alliance = true; //false = red

        if (blue_alliance) {
            goal_X = -58.346;
        } else {
            goal_X = 58.346;
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 61, Math.toRadians(180)));

        limelight.start();
        limelight.pipelineSwitch(5);

        waitForStart();

        MASTER_TIMER.reset();

        // Execução principal do OpMode
        if (opModeIsActive()) {

            cicle_time = deltaTime.milliseconds();
            cicle_nano = deltaTime.nanoseconds();
            deltaTime.reset();
            angulo_desejado = 0;
            while (opModeIsActive()) {
                {
                    //Atualização de dados

                    LLResult result = limelight.getLatestResult();
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();


                    //Atualização de dados da Limelight----------------------

                    {
                        if (limelight.getLatestResult().isValid()) {
                            detecting_ap = true;
                            for (LLResultTypes.FiducialResult fr : fiducialResults) {

                                if ((limelight.getLatestResult().isValid()) && (fr.getFiducialId() == 24 || fr.getFiducialId() == 20)) {
                                    TxOffset = fr.getTargetXPixels();
                                    goal_ap = true;
                                    limelight_posY = -result.getBotpose().getPosition().x * 35.3;
                                    limelight_posX = result.getBotpose().getPosition().y * 35.3;
                                    limelight_yaw = -result.getBotpose().getOrientation().getYaw() + 180;
                                    distLime = result.getBotposeAvgDist() / 0.0254;


                                    if (result.getBotposeAvgDist() < 80) {
                                        atualizar_limelight = true;
                                        drive = new MecanumDrive(hardwareMap, new Pose2d(limelight_posX, limelight_posY, Math.toRadians(limelight_yaw)));

                                    }

                                } else goal_ap = false;
                            }
                        } else {
                            TxOffset = 0;
                            detecting_ap = false;
                        }
                    }

                    //Sistema de posição-----------------------------------------------------------------------------

                    drive.updatePoseEstimate();
                    pose_atual = drive.localizer.getPose();

                    //Controle da posição atual----------------------------------------------------------------------

                    {
                        if (gamepad_1) {
                            if (Math.abs(gamepad1.left_stick_x) > 0.085 || Math.abs(gamepad1.left_stick_y) > 0.085) {
                                last_pose_X = pose_atual.position.x;
                                last_pose_Y = pose_atual.position.y;
                                Movimento_robo = true;
                            } else {
                                Movimento_robo = false;
                            }
                        } else {
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
                    }

                    //Funções principais ------------------------------------------------------
                    Base_Motriz();
                    Indexer();
                    Shooting_Vel();
                    //Reset robô----------------------------------------------------------------------------------------

                    {
                        if (gamepad_1) {
                            if (gamepad1.options) {
                                angulo_robo = 0;
                                last_pose_X = pose_atual.position.x;
                                last_pose_Y = pose_atual.position.y;
                            }
                        } else {
                            if (gamepad2.options) {
                                angulo_robo = 0;
                                last_pose_X = pose_atual.position.x;
                                last_pose_Y = pose_atual.position.y;
                            }

                        }
                    }

                    //Cálculo de alinhamento com o alvo----------------------------------------------------------------

                    {
                        double cateto_a = goal_X - pose_atual.position.x;
                        double cateto_b = goal_Y - pose_atual.position.y;
                        hip = Math.sqrt(Math.pow(cateto_a, 2) + Math.pow(cateto_b, 2));

                        alpha = Math.toDegrees(Math.atan2(-cateto_b, cateto_a));
                    }

                    //Velocidade do Atirador---------------------------------------------------------------------------

                    if (true) {
                        areapermitida = true;
                        if (gamepad2.triangle) {
                            gamepad_1 = false;
                            velocidade_desejada = Const + Variable * hip;
                        } else if (gamepad2.circle) {
                            velocidade_desejada = 1280;
                        } else if (gamepad2.square) {
                            velocidade_desejada = 1650;
                        } else {
                            kf_atirar = 0;
                            kp_atirar = 0;
                            CorrTx = 0;
                            velocidade_desejada = 0;
                        }
                    }

//----------------Intake-------------------------------------------------------------------------------------------

                    if (gamepad1.cross) Intake.setPower(1);
                    else if (gamepad1.circle) Intake.setPower(-1);
                    else Intake.setPower(0);
//----------------Troca de controles-------------------------------------------------------------------------------

                    if (gamepad2.cross) gamepad_1 = false;
                    else if (gamepad1.square) gamepad_1 = true;

//--------------------Reservatorios--------------------------------------------------------------------------------

                    {
                        if (gamepad2.dpad_left) {
                            timer_shooter.reset();
                            Shoot(1);
                        }

                        if (gamepad2.dpad_up) {
                            timer_shooter.reset();
                            Shoot(2);

                        }

                        if (gamepad2.dpad_right) {
                            timer_shooter.reset();
                            Shoot(3);

                        }

                        if (gamepad2.right_bumper) {
                            Reservoir_1.setPosition(0);
                            Reservoir_2.setPosition(0);
                            Reservoir_3.setPosition(0);
                        }
                    }
                    //INDICAÇÃO DOS LEDS-------------------------------------------------------------------------------

                    {
                        if (areapermitida && velocidade_desejada != 0 && (velocidade_desejada > Math.abs(Atirador_1.getVelocity()) - 60 && velocidade_desejada < Math.abs(Atirador_1.getVelocity()) + 60)) {
                            if (detecting_ap)
                                Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
                            else Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                        } else if (areapermitida) {
                            if (detecting_ap)
                                Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
                            else Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                        } else {
                            if (detecting_ap)
                                Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
                            else Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                        }
                    }

//------------------------------------------------------------------------------------------------------------------
                    if (Shoot) Shoot_single();


                    Telemetria();
                    Tempo_de_ciclo.reset();
                }



            }
        }
    }

    //-------FUNÇÕES DE MOVIMENTO-----------------------------------------------------------------------------------------

    private double PiDFO_Correcao_Angular(double erro, double kp, double ki, double kd, double kf,double ko, double min, double max){

        double BatVoltage = myControlHubVoltageSensor.getVoltage();
        double feedfoward = 1 + (kf * (12.5-BatVoltage));
        double integrativo = ki * (erro * cicle_time);
        double derivativo = kd * ((erro - lasterror_PiDF_Correcao_Angular) / cicle_time);
        double proporcional = kp * erro;

        integral_PiDF_Correcao_Angular += integrativo;
        lasterror_PiDF_Correcao_Angular = erro;

        if (erro<0 && integral_PiDF_Correcao_Angular >  0)integral_PiDF_Correcao_Angular = 0;
        if (erro>0 && integral_PiDF_Correcao_Angular <  0)integral_PiDF_Correcao_Angular = 0;
        if (integral_PiDF_Correcao_Angular>0.15) integral_PiDF_Correcao_Angular=0.05;
        else if (integral_PiDF_Correcao_Angular<-0.15) integral_PiDF_Correcao_Angular=-0.05;

        double output = (proporcional + integral_PiDF_Correcao_Angular + derivativo ) * feedfoward;

        if ( output > max) output = max;
        else if (output < min) output = min;

        if (output < 0)osc_pos_time_ca.reset();

        if (output > 0)osc_neg_time_ca.reset();

        osc_time_ca = osc_pos_time_ca.milliseconds() + osc_neg_time_ca.milliseconds();
        double ko_corr = osc_time_ca/ko;

        if ( ko_corr > 1) ko_corr = 1;

        output = output * ko_corr;

        if (Math.abs(output) < 0.065)output = 0;

        return output;

    }

    private double PiDFO_Indexer(double erro, double kp, double ki, double kd, double kf,double ko, double min, double max){

        double BatVoltage = myControlHubVoltageSensor.getVoltage();
        double feedfoward = 1 + (kf * (12.5-BatVoltage));
        double integrativo = ki * (erro * cicle_time);
        double derivativo = kd * ((erro - lasterror_PiDF_Indexer) / cicle_time);
        double proporcional = kp * erro;

        integral_Indexer += integrativo;
        lasterror_PiDF_Indexer = erro;

        if (erro<0 && integral_Indexer >  0)integral_Indexer = 0;
        if (erro>0 && integral_Indexer <  0)integral_Indexer = 0;
        if (integral_Indexer>0.15) integral_Indexer=0.05;
        else if (integral_Indexer<-0.15) integral_Indexer=-0.05;

        double output = (proporcional + integral_Indexer + derivativo ) * feedfoward;

        if ( output > max) output = max;
        else if (output < min) output = min;

        if (output < 0)osc_pos_time_ind.reset();

        if (output > 0)osc_neg_time_ind.reset();

        osc_time_ind = osc_pos_time_ind.milliseconds() + osc_neg_time_ind.milliseconds();
        double ko_corr = osc_time_ind/ko;

        if ( ko_corr > 1) ko_corr = 1;

        output = output * ko_corr;

        if (Math.abs(output) < 0.065)output = 0;

        return output;

    }

    public double PiDF_Vel(double erro, double kp, double ki, double kd, double kf, double min, double max){

        double BatVoltage = myControlHubVoltageSensor.getVoltage();
        double feedfoward = 1 + (kf * (12.5-BatVoltage));
        double integrativo = ki * (erro * cicle_nano);
        double derivativo = kd * ((erro - lasterror_PiDF_Shooter) / cicle_nano);
        double proporcional = kp * erro;

        integral_PiDF_Shooter += integrativo;
        lasterror_PiDF_Shooter = erro;

        if (Math.abs(erro)<0.05) integral_PiDF_Shooter = 0;
        if (integral_PiDF_Shooter>0.15) integral_PiDF_Shooter=0.15;
        else if (integral_PiDF_Shooter<-0.15) integral_PiDF_Shooter=-0.15;


        double output = (proporcional + integral_PiDF_Shooter + derivativo ) * feedfoward;

        if ( output > max) output = max;
        else if (output < min) output = min;

        amortecimento[0] = output;

        for (int j =2; j > 0; j--) amortecimento[j] = (amortecimento[j - 1]);

        double soma = 0;

        for(int x=0 ; x<3 ; x++) soma = soma + amortecimento[x];

        output = soma/3;


        return output;

    }
    private void Base_Motriz() {

        {
            angulo_robo = Math.toDegrees(Math.atan2(pose_atual.heading.real, pose_atual.heading.imag));

            double divisorX = 5 + 10 * Math.max(0, 1.0 - (Math.abs(ErroMovX) / 4.0));
            double divisorY = 5 + 10 * Math.max(0, 1.0 - (Math.abs(ErroMovY) / 4.0));
            if (gamepad_1) {
                vectorX = (Math.cos(-angulo_robo / 180 * Math.PI) * (gamepad1.left_stick_x/*+ (ErroMovY/divisorY)*/))
                        + Math.sin(-angulo_robo / 180 * Math.PI) * (-gamepad1.left_stick_y/*+(-ErroMovX/divisorX)*/);

                vectorY = ((-Math.sin(-angulo_robo / 180 * Math.PI) * (gamepad1.left_stick_x/*+ (ErroMovY/divisorY)*/))
                        + Math.cos(-angulo_robo / 180 * Math.PI) * (-gamepad1.left_stick_y /*+(-ErroMovX/divisorX)*/));
            } else {
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
            } else {
                leftBack.setPower((vectorY * 1 + vectorX * -1) * Speed + ((gamepad2.right_trigger - gamepad2.left_trigger) + correcao_angular));
                leftFront.setPower((vectorY * 1 + vectorX * 1) * Speed + ((gamepad2.right_trigger - gamepad2.left_trigger) + correcao_angular));
                rightBack.setPower((vectorY * 1 + vectorX * 1) * Speed - ((gamepad2.right_trigger - gamepad2.left_trigger) + correcao_angular));
                rightFront.setPower((vectorY * 1 + vectorX * -1) * Speed - ((gamepad2.right_trigger - gamepad2.left_trigger) + correcao_angular));
            }

        }

        int rotate;

        if (angulo_robo >= 0) {
            rotate = 360;
        } else {
            rotate = -360;
        }

        // if (gamepad2.triangle && !goal_ap) angulo_desejado = alpha;

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

            if (goal_ap && gamepad2.triangle && TxOffset != 0){
                angulo_desejado = angulo_robo;
                correcaoP = (TxOffset-1250)/47.5;
            } //else if (gamepad2.triangle && !goal_ap) angulo_desejado = alpha;

            correcao_angular = PiDFO_Correcao_Angular(correcaoP, 0.014 ,0.4, 1.3, 0.15,100,-0.5,0.5);

        }


        // Ajuste de velocidade baseado nos triggers do gamepad
        {
        if(gamepad_1) {
            if (Math.abs(gamepad1.left_trigger) > 0.1 || Math.abs(gamepad1.right_trigger) > 0.1) {
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
        }else {
            if (Math.abs(gamepad2.left_trigger) > 0.1 || Math.abs(gamepad2.right_trigger) > 0.1) {
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
        }

        // Reduzindo a velocidade caso o motor `slide_vertical_e` ultrapasse uma posição

    }

    private void Indexer() {
        double potencia_indexer;
        double erro = posicao_desejada_indexer - Indexer.getCurrentPosition();
        // Acumulando o erro para o integral


        if (Math.abs(erro)<2) integral_Indexer = 0;
        if (integral_Indexer>0.1) integral_Indexer=0.1;
        else if (integral_Indexer<-0.1) integral_Indexer=-0.1;

        potencia_indexer = PiDFO_Indexer(erro, 0.058,0.5,10,0.1,300,-0.4,0.4);

        Indexer.setPower(potencia_indexer);
    }

    private void Shooting_Vel() {
        double power_correction;
        double erro = velocidade_desejada + Atirador_1.getVelocity();

        power_correction =  PiDF_Vel(erro,0.002,1,1,0.15,0,1);

        //if (power_correction < 0) power_correction = 0;
        if (velocidade_desejada == 0 ) Atirador_1.setPower(0);
        else Atirador_1.setPower( power_correction );
        Atirador_2.setPower(-Atirador_1.getPower());
    }

    private void Shoot_single() {

        if (0 < timer_shooter.seconds() && 0.9 >= timer_shooter.seconds()){
            if(reservatorio == 1)  posicao_desejada_indexer= -100;
            if(reservatorio == 2)  posicao_desejada_indexer= 218;
            if(reservatorio == 3)  posicao_desejada_indexer= 440;

        }else if (0.9 < timer_shooter.seconds() && 1.7 >= timer_shooter.seconds()){
            if(reservatorio == 1){
                Indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Reservoir_1.setPosition(1);
            }
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

        //Telemetria para gráficos----------------

        {
            dashboardTelemetry.addData("Velocidade Atirador", -Atirador_1.getVelocity());
            dashboardTelemetry.addData("Velocidade desejada", velocidade_desejada);

            dashboardTelemetry.addData("Coreeção angular",correcao_angular );
            dashboardTelemetry.addData("Correção desejada", 0);

            dashboardTelemetry.addData("angulo_robo",angulo_robo );

            dashboardTelemetry.addData("posicao desejada ind",posicao_desejada_indexer );
            dashboardTelemetry.addData("posicao ind",Indexer.getCurrentPosition() );
            dashboardTelemetry.addData("power atirador",Atirador_1.getPower() );
        }

        //Telemetria padrão-----------------------

        {
            telemetry.addData("Área de lançamento", areapermitida);
            telemetry.addData("Att Limelight", atualizar_limelight);

            telemetry.addData("DisLime", distLime);




            telemetry.addData("", "");

            telemetry.addData("posicao indexador", Indexer.getCurrentPosition());
            telemetry.addData("potencia indexador", Indexer.getPower());
            telemetry.addData("posicao desejada", posicao_desejada_indexer);
            telemetry.addData("hipotenusa", hip);
            telemetry.addData("alpha", alpha);


            telemetry.addData("", "");

            telemetry.addData("Offset Tx", (TxOffset - 1250));

            telemetry.addData("", "");

            telemetry.addData("erro_vel", (velocidade_desejada + Atirador_1.getVelocity()));
            telemetry.addData("Atirador1_vel", Atirador_1.getVelocity());
            telemetry.addData("Atirador1_power", Atirador_1.getPower());
            telemetry.addData("Atirador1_veldesejada", velocidade_desejada);

            telemetry.addData("Corrigir_velocidade", Corrigir_velocidade);
            telemetry.addData("Iniciar_atirador", Iniciar_atirador);


            telemetry.addData("", "");

            // telemetry.addData("Posição", pose);
            telemetry.addData("x", pose_atual.position.x);
            telemetry.addData("y", pose_atual.position.y);

            telemetry.addData("lastx", last_pose_X);
            telemetry.addData("lasty", last_pose_Y);

            telemetry.addData("limex", limelight_posX);
            telemetry.addData("limey", limelight_posY);

            telemetry.addData("", "");

            telemetry.addData("Velocidade: ", Speed);

            telemetry.addData("Base motriz função: ", Base_Motriz_State);
            telemetry.addData("Tempo de ciclo:", "");


            T_atual = Tempo_de_ciclo.milliseconds();
            if (T_atual > T_max) T_max = T_atual;
            if (T_atual < T_min) T_min = T_atual;
            if (MASTER_TIMER.seconds() % 5 < 1) {
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
            telemetry.addData("erro angulo", angulo_desejado - angulo_robo);
            telemetry.addData("X", vectorX);
            telemetry.addData("Y", vectorY);
            telemetry.addData("", "");

        }

        // Atualiza os dados na tela--------------
        telemetry.update();
        dashboardTelemetry.update();
    }

}
