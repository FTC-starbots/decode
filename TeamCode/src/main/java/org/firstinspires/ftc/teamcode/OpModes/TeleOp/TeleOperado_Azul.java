package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpModes.Autos.Autonomo_VermelhoGRANDE;

import java.util.List;

@Config
@TeleOp


public class TeleOperado_Azul extends Autonomo_VermelhoGRANDE {

    public DigitalChannel  Alliance_B, Alliance_R, Auto1,Auto2, Auto3, Auto4, LED_S_R,LED_S_G,LED_S_B,LED_S_BB;

    public static boolean RED=true, GRENN=true, BRUE=true;
    public LED Led_1_R, Led_1_V, Led_2_R, Led_2_V, Led_3_R, Led_3_V;
    public AnalogInput S1, S2,S3,S4;

    boolean AlinharTx = false, Manual = true, Movimento_robo = false, Base_Motriz_State = false, Shoot_mul = false, Shoot = false, gamepad_1 = true, atualizar_limelight = false;
    boolean Iniciar_atirador = false, Corrigir_velocidade = false, detecting_ap = false, goal_ap = false;

public static double kpt= 0.003,kit = 0.0000001,kdt = 0.003 ,kft = 0.05,mint = 0,maxt = 1;
    private Limelight3A limelight;

    private DcMotorEx leftBack, leftFront, rightBack, rightFront, Intake, Atirador_1, Atirador_2, Indexer;

    private Servo Reservoir_2, Reservoir_1, Reservoir_3;

    private RevBlinkinLedDriver Led_Strip;


    int posicao_desejada_indexer = 0, reservatorio = 1,lastreservatorio = 0, reservatorio_1, reservatorio_2, reservatorio_3;

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





    double distLime, Velin0 = 0;

    double CorrTx,TxOffset, ErroMovX, ErroMovY, limelight_posX, limelight_posY, limelight_yaw, last_pose_X, last_pose_Y, goal_X, goal_Y = 55.630, Ai_shoot = 0, hip;
    ElapsedTime timer_sequencia,RotacaoRobo, Tempo_de_ciclo,
            MASTER_TIMER, timer_shooter, deltaTime, osc_pos_time_ca, osc_neg_time_ca, osc_pos_time_ind, osc_neg_time_ind;


    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public double XInicial, YInicial;


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

            Alliance_B = hardwareMap.get(DigitalChannel.class, "Alliance_B");
            Alliance_R = hardwareMap.get(DigitalChannel.class, "Alliance_R");
            Auto1 = hardwareMap.get(DigitalChannel.class, "Auto1");
            Auto2 = hardwareMap.get(DigitalChannel.class, "Auto2");
            Auto3 = hardwareMap.get(DigitalChannel.class, "Auto3");
            Auto4 = hardwareMap.get(DigitalChannel.class, "Auto4");

            LED_S_R = hardwareMap.get(DigitalChannel.class, "LED_S_R");
            LED_S_G = hardwareMap.get(DigitalChannel.class, "LED_S_G");
            LED_S_B = hardwareMap.get(DigitalChannel.class, "LED_S_B");
            LED_S_BB = hardwareMap.get(DigitalChannel.class, "LED_S_BB");

            Led_1_R = hardwareMap.get(LED.class, "Led_1_R");
            Led_1_V = hardwareMap.get(LED.class, "Led_1_V");
            Led_2_R = hardwareMap.get(LED.class, "Led_2_R");
            Led_2_V = hardwareMap.get(LED.class, "Led_2_V");
            Led_3_R = hardwareMap.get(LED.class, "Led_3_R");
            Led_3_V = hardwareMap.get(LED.class, "Led_3_V");

            S1 = hardwareMap.get(AnalogInput.class, "S1");
            S2 = hardwareMap.get(AnalogInput.class, "S2");
            S3 = hardwareMap.get(AnalogInput.class, "S3");
            S4 = hardwareMap.get(AnalogInput.class, "S4");


        }
        // Configuração dos motores da base
        {

            Alliance_B.setMode(DigitalChannel.Mode.INPUT);
            Alliance_R.setMode(DigitalChannel.Mode.INPUT);
            Auto1.setMode(DigitalChannel.Mode.INPUT);
            Auto2.setMode(DigitalChannel.Mode.INPUT);
            Auto3.setMode(DigitalChannel.Mode.INPUT);
            Auto4.setMode(DigitalChannel.Mode.INPUT);

            LED_S_R.setMode(DigitalChannel.Mode.OUTPUT);
            LED_S_G.setMode(DigitalChannel.Mode.OUTPUT);
            LED_S_B.setMode(DigitalChannel.Mode.OUTPUT);
            LED_S_BB.setMode(DigitalChannel.Mode.OUTPUT);

            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
            timer_sequencia = new ElapsedTime();

            timer_shooter   .reset();
            RotacaoRobo     .reset();
            Tempo_de_ciclo  .reset();
            deltaTime       .reset();
            osc_pos_time_ca.reset();
            osc_neg_time_ca.reset();
            osc_pos_time_ind.reset();
            osc_neg_time_ind.reset();
            timer_sequencia.reset();

        }

        Speed = 1;
        angulo_desejado = 180;
        posicao_desejada_indexer = 0;



        boolean blue_alliance = Alliance_B.getState(); //false = red

        if (blue_alliance) {
            goal_X = -58.346;
        } else {
            goal_X = 58.346;
        }

        if (!Auto1.getState()){
            XInicial = 20;
            YInicial  = 40;
        }
        else if (!Auto2.getState()){
            XInicial = 20;
            YInicial  = -35;
        }
        else if (!Auto3.getState()){
            XInicial = 15;
            YInicial  = 40;
        }
        else if (!Auto4.getState()){
            XInicial = 20;
            YInicial  = 40;
        }



        limelight.start();
        limelight.pipelineSwitch(5);

        waitForStart();

        MASTER_TIMER.reset();

        // Execução principal do OpMode
        if (opModeIsActive()) {

            cicle_time = deltaTime.milliseconds();
            cicle_nano = deltaTime.nanoseconds();
            deltaTime.reset();
            angulo_desejado = 180;
            while (opModeIsActive()) {

                Sensor_de_cor();


                if (gamepad2.leftBumperWasPressed()) Manual = !Manual;

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
                                        limelight_yaw = (result.getBotpose().getOrientation().getYaw() - 90);
                                        if (result.getBotposeAvgDist() != 0)
                                            distLime = result.getBotposeAvgDist() / 0.0254;


                                        atualizar_limelight = true;

                                    } else {
                                        TxOffset = 0;
                                        distLime = 0;
                                        goal_ap = false;
                                    }
                                }
                            } else {
                                TxOffset = 0;
                                detecting_ap = false;
                            }
                        }
                    //Sistema de posição-----------------------------------------------------------------------------

                    //Controle da posição atual----------------------------------------------------------------------

                    {

                    }
                    //Funções principais ------------------------------------------------------
                    Base_Motriz();
                    Indexer();
                    Shooting_Vel();
                    //Reset robô----------------------------------------------------------------------------------------

                    {
                        if (gamepad_1) {

                    //Velocidade do Atirador---------------------------------------------------------------------------

                    if (Manual) {
                            areapermitida = false;
                            if (gamepad2.triangle) {
                                limelight.start();
                                limelight.pipelineSwitch(5);
                                Iniciar_atirador = true;
                                gamepad_1 = false;
                                velocidade_desejada = 1120;
                            } else if (gamepad2.square) {
                                limelight.start();
                                limelight.pipelineSwitch(5);

                                Iniciar_atirador = true;
                                velocidade_desejada = 1050;
                            } else {
                                limelight.stop();

                                Iniciar_atirador = false;
                                kf_atirar = 0;
                                kp_atirar = 0;
                                CorrTx = 0;
                                velocidade_desejada = 0;
                            }
                    }}
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

                        if (gamepad2.dpad_down) {
                            timer_sequencia.reset();
                            Shoot(5);
                            Shoot_mul = true;

                        }

                    }
                    //INDICAÇÃO DOS LEDS-------------------------------------------------------------------------------

                    {
                        if(!Manual) {
                            if (areapermitida && velocidade_desejada != 0 && (velocidade_desejada > Math.abs(Atirador_1.getVelocity()) - 60 && velocidade_desejada < Math.abs(Atirador_1.getVelocity()) + 60)) {
                                if (goal_ap)
                                    Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
                                else Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                            } else if (areapermitida) {
                                if (goal_ap)
                                    Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
                                else Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                            } else {
                                if (goal_ap)
                                    Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
                                else Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                            }
                        }else{
                            if (goal_ap) Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
                            else Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                        }
                    }

//------------------------------------------------------------------------------------------------------------------
                    if(Shoot)Shoot_single();
                    if(Shoot_mul)Shoot_multiple();


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




        double output = (proporcional + integral_PiDF_Shooter + derivativo ) * feedfoward;

        if ( output > max) output = max;
        else if (output < min) output = min;

        return output;

    }
    private void Base_Motriz() {


        {
            angulo_robo = Math.toDegrees(Math.atan2(pose_atual.heading.imag, pose_atual.heading.real));

            double divisorX = 5 + 10 * Math.max(0, 1.0 - (Math.abs(ErroMovX) / 4.0));
            double divisorY = 5 + 10 * Math.max(0, 1.0 - (Math.abs(ErroMovY) / 4.0));

                if (gamepad_1) {

                    vectorX = (Math.cos((angulo_robo + 180) / 180 * Math.PI) * (gamepad1.left_stick_x/*+ (-ErroMovY/divisorY)*/))
                            + Math.sin((angulo_robo + 180) / 180 * Math.PI) * (-gamepad1.left_stick_y/*+(ErroMovX/divisorX)*/);

                    vectorY = ((-Math.sin((angulo_robo + 180) / 180 * Math.PI) * (gamepad1.left_stick_x/*+ (-ErroMovY/divisorY)*/))
                            + Math.cos((angulo_robo + 180) / 180 * Math.PI) * (-gamepad1.left_stick_y /*+(ErroMovX/divisorX)*/));
                } else {

                    vectorX = (Math.cos((angulo_robo + 180) / 180 * Math.PI) * ( gamepad2.left_stick_x /*+ (-ErroMovY/divisorY)*/))
                            + Math.sin((angulo_robo + 180) / 180 * Math.PI) * (-gamepad2.left_stick_y/*+ (ErroMovY/divisorX)*/);

                    vectorY = ((-Math.sin((angulo_robo + 180) / 180 * Math.PI) * (gamepad2.left_stick_x /*+ (-ErroMovY/divisorY)*/))
                            + Math.cos((angulo_robo + 180) / 180 * Math.PI) *   (-gamepad2.left_stick_y/*+ (ErroMovY/divisorX)*/));

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


        double corr_1 = angulo_robo - angulo_desejado;
        double corr_2 = angulo_desejado + rotate - angulo_robo;
        double correcaoP;
        if (Math.abs(corr_1) > Math.abs(corr_2)) correcaoP = -corr_2;
        else correcaoP = corr_1;


        if (RotacaoRobo.seconds() < 1) {
            correcao_angular = 0;
            angulo_desejado = angulo_robo;
            last_pose_X = pose_atual.position.x;
            last_pose_Y = pose_atual.position.y;
        } else {
            if (!Manual) {
                if (!gamepad2.triangle && !goal_ap){
                    AlinharTx = false;
                    correcao_angular = PiDFO_Correcao_Angular(correcaoP, 0.014 ,0.4, 1.3, 0.15,100,-0.5,0.5);
                }


                if (goal_ap && gamepad2.triangle && TxOffset != 0) {
                    AlinharTx = true;
                    angulo_desejado = angulo_robo;

                }
            }else{
                AlinharTx = false;
                correcao_angular = PiDFO_Correcao_Angular(correcaoP, 0.014 ,0.4, 1.3, 0.15,100,-0.5,0.5);
            }



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


                angulo_desejado = 180;
            } else {
                // Ajustando a velocidade com base na correção angular
                Speed = 1.2 - Math.abs(correcao_angular) - (ControlHub_VoltageSensor.getVoltage() - 11) / 10;

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

                angulo_desejado = 180;
            } else {
                // Ajustando a velocidade com base na correção angular
                Speed = 1.2 - Math.abs(correcao_angular) - (ControlHub_VoltageSensor.getVoltage() - 11) / 10;

            }
        }
        }

        // Reduzindo a velocidade caso o motor `slide_vertical_e` ultrapasse uma posição


    }
    private void Sensor_de_cor() {

        if(S4.getVoltage()<0.4){
            Led_3_R.off();
            Led_3_V.off();
        }
        else if(S4.getVoltage()<0.67){
            Led_3_R.off();
            Led_3_V.on();
        }
        else{
            Led_3_R.on();
            Led_3_V.off();
        }

        if(S2.getVoltage()<0.25){
            Led_2_R.off();
            Led_2_V.off();
        }
        else if(S2.getVoltage()<0.4){
            Led_2_R.off();
            Led_2_V.on();
        }
        else{
            Led_2_R.on();
            Led_2_V.off();
        }

        if(S1.getVoltage()<0.1){
            Led_1_R.off();
            Led_1_V.off();
        }
        else if(S1.getVoltage()<0.2){
            Led_1_R.off();
            Led_1_V.on();
        }
        else{
            Led_1_R.on();
            Led_1_V.off();
        }

        LED_S_G.setState(true);
        LED_S_R.setState(true);
        LED_S_B.setState(true);
        LED_S_BB.setState(true);
    }
    private void Indexer() {
        double potencia_indexer;
        double erro = posicao_desejada_indexer - Indexer.getCurrentPosition();
        // Acumulando o erro para o integral


        if (Math.abs(erro)<5) integral_Indexer = 0;
        if (integral_Indexer>0.1) integral_Indexer=0.1;
        else if (integral_Indexer<-0.1) integral_Indexer=-0.1;

        potencia_indexer = PiDFO_Indexer(erro, 0.002,0.0001,0,0.1,300,-0.4,0.4);


        Indexer.setPower(potencia_indexer);
    }

    private void Shooting_Vel() {
        if (goal_ap && gamepad2.triangle && TxOffset != 0) {
            AlinharTx = true;
            angulo_desejado = angulo_robo;
            if (TxOffset < 1190) {
                leftBack.setPower(-0.151);
                leftFront.setPower(-0.151);
                rightBack.setPower(0.151);
                rightFront.setPower(0.151);

            } else if (TxOffset > 1390) {
                leftBack.setPower(0.151);
                leftFront.setPower(0.151);
                rightBack.setPower(-0.151);
                rightFront.setPower(-0.151);
            } else {
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
            }

        }
    double power_correction;
        if(Iniciar_atirador) {
            double erro = velocidade_desejada + Atirador_1.getVelocity(); // Desired Vel - Velocity


    power_correction = PiDF_Vel(erro, kpt, kit, kdt, kft, mint, maxt);
}else power_correction = 0;

        if (power_correction < 0) power_correction = 0;
        if (velocidade_desejada == 0 ) Atirador_1.setPower(0);
        else Atirador_1.setPower( power_correction );


        Atirador_1.setPower(power_correction);
        Atirador_2.setPower(-Atirador_1.getPower());
    }

    private void Shoot_single() {
        Reservoir_1.setPosition(0);
        Reservoir_2.setPosition(1);
        Reservoir_3.setPosition(0);
        if (0 < timer_shooter.seconds() && 0.6 >= timer_shooter.seconds()){
            if(reservatorio == 1)  posicao_desejada_indexer= -100;
            if(reservatorio == 2)  posicao_desejada_indexer= 218;
            if(reservatorio == 3)  posicao_desejada_indexer= 440;

        }else if (0.6 < timer_shooter.seconds() && 1.3 >= timer_shooter.seconds()){
            if(reservatorio == 1){
                Indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Reservoir_1.setPosition(1);
            }
            if(reservatorio == 2) Reservoir_2.setPosition(0);
            if(reservatorio == 3)Reservoir_3.setPosition(1);

        }else if (1.3 < timer_shooter.seconds() && 1.4 >= timer_shooter.seconds()){
            Shoot = false;

        }

    }

    private void Shoot_multiple(){
        if (0 < timer_sequencia.seconds() && 0.15 >= timer_sequencia.seconds()) {

            if (lastreservatorio == 1) Shoot(1);
            else if (lastreservatorio == 2) Shoot(2);
            else if (lastreservatorio == 3) Shoot(3);

        }else  if (1.3 < timer_sequencia.seconds() && 1.45 >= timer_sequencia.seconds()){


            if (lastreservatorio == 1) Shoot(2);
            else if (lastreservatorio == 2) Shoot(3);
            else if (lastreservatorio == 3) Shoot(2);

        }else  if (2.6 < timer_sequencia.seconds() && 2.75 >= timer_sequencia.seconds()){


            if (lastreservatorio == 1) Shoot(3);
            else if (lastreservatorio == 2) Shoot(1);
            else if (lastreservatorio == 3) Shoot(1);
        }
    }

    private void Shoot(int padrao){

        //posições individuais--------------------------------
        // Esquerda
        if (padrao == 1){
            timer_shooter.reset();
            reservatorio = 1;
            Shoot = true;
        }
        //Meio
        if (padrao == 2) {
            timer_shooter.reset();
            reservatorio = 2;
            Shoot = true;
        }
        //Direita
        if (padrao == 3){
            timer_shooter.reset();
            reservatorio = 3;
            Shoot = true;
        }
        //verde mais próxima -------------------------------
        if (padrao == 3);


        //roxa mais próxima --------------------------------
        if (padrao == 4);


        //todas o mais rapido
        if (padrao == 5){
            Shoot_mul = true;
            lastreservatorio = reservatorio;
        }
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


            telemetry.addData("Modo Manual", Manual);
            telemetry.addData("", "");

            telemetry.addData("S1", S1.getVoltage());
            telemetry.addData("S2", S2.getVoltage());
            telemetry.addData("S3", S3.getVoltage());
            telemetry.addData("S4", S4.getVoltage());


            telemetry.addData("AZUL", Alliance_B.getState());
            telemetry.addData("vermelho", Alliance_R.getState());

            if(!Auto1.getState())telemetry.addLine("AUTO 1 ");
            else if(!Auto2.getState())telemetry.addLine("AUTO 2");
            else if(!Auto3.getState())telemetry.addLine("AUTO 3");
            else if(!Auto4.getState())telemetry.addLine("AUTO 4");



            telemetry.addData("Área de lançamento", areapermitida);
            telemetry.addData("Att Limelight", atualizar_limelight);

            telemetry.addData("DisLime", distLime);

            telemetry.addData("Velocity in 0", Velin0);





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
