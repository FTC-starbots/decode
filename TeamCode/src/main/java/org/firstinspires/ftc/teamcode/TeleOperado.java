package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class TeleOperado extends AprilTag{

    boolean Movimento_robo = false,Base_Motriz_State = false;



    private DcMotor leftBack, leftFront, rightBack, rightFront;

    //private Limelight3A limelight;

    int pulse_reset, posicao_desejada_slide_horizontal = 0, posicao_desejada_braco_outtake = 0;

    double  controle_rotacao, Speed, angulo_desejado, angulo_robo, vectorX, vectorY, correcao_angular;

    int zona_bloqueada = 1;

    double CorrLimeX, CorrLimeY;

    private VoltageSensor ControlHub_VoltageSensor;

    double T_max = 0;
    double T_min = 1000;
    double T_atual = 0;

    Pose2d pose_atual;

    double ErroMovX, ErroMovY, last_pose_X, last_pose_Y;
    ElapsedTime RotacaoRobo, Tempo_de_ciclo,
           MASTER_TIMER;



    @Override
    public void runOpMode() throws InterruptedException {

        // Mapeamento dos sensores e motores
        {

            leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

           // limelight = hardwareMap.get(Limelight3A.class, "limelight");
        }
        // Configuração dos motores da base
        {
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            leftBack.setDirection(DcMotor.Direction.REVERSE);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.FORWARD);

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Configuração dos motores adicionais
        }
        // Inicialização dos temporizadores
        {
            RotacaoRobo = new ElapsedTime();
            Tempo_de_ciclo = new ElapsedTime();
            MASTER_TIMER = new ElapsedTime();

            RotacaoRobo.reset();
            Tempo_de_ciclo.reset();

        }
        pulse_reset = 1;
        Speed = 1;
        angulo_desejado = 0;
        posicao_desejada_slide_horizontal = 0;

        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        // Aguardando o início da execução
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();

        MASTER_TIMER.reset();

        // Execução principal do OpMode
        if (opModeIsActive()) {
            angulo_desejado = 0;

            while (opModeIsActive()) {
                //MOVIMENTO______________________________________________________________________________

                drive.updatePoseEstimate();

                pose_atual = drive.localizer.getPose();
                if (Math.abs(gamepad1.left_stick_x) >0.085|| Math.abs(gamepad1.left_stick_y) > 0.085) {
                    last_pose_X = pose_atual.position.x;
                    last_pose_Y = pose_atual.position.y;
                    Movimento_robo = true;
                }else {
                    Movimento_robo = false;
                }

                if (Movimento_robo == false) {
                    ErroMovX = pose_atual.position.x - last_pose_X;
                    ErroMovY = pose_atual.position.y - last_pose_Y;
                }else {
                    ErroMovX = 0;
                    ErroMovY = 0;
                }

                if (pose_atual.position.y > 35 && pose_atual.position.x > -20 && pose_atual.position.x < 20)zona_bloqueada = 0;
                else zona_bloqueada = 1;

                Base_Motriz();

                if (gamepad1.options) {
                    angulo_robo = 0;
                    last_pose_X = pose_atual.position.x;
                    last_pose_Y = pose_atual.position.y;
                }
                //_______________________________________________________________________________

                CorrLimeY = 0;
                CorrLimeX = 0;
                controle_rotacao = 0.5;

                //--BUMPERS------------------------------------------------------------------------------------



                //--DEMAIS-BOTÕES-------------------------------------------------------------------------------

                //-----------------------------------------------------------------------------------------------

                Telemetria();
                Tempo_de_ciclo.reset();
            }
        }
    }

    //-------FUNÇÕES DE MOVIMENTO-----------------------------------------------------------------------------------------

    private void Base_Motriz(){

        angulo_robo = (rightFront.getCurrentPosition() - leftBack.getCurrentPosition()) / 249.0;

        double divisorX = 5 + 10 * Math.max(0, 1.0 - (Math.abs(ErroMovX) / 4.0));
        double divisorY = 5 + 10 * Math.max(0, 1.0 - (Math.abs(ErroMovY) / 4.0));

        vectorX = (Math.cos(-angulo_robo / 180 * Math.PI) * (gamepad1.left_stick_x/*+ (ErroMovY/divisorY)*/))
                + Math.sin(-angulo_robo / 180 * Math.PI) * (gamepad1.left_stick_y/*+(-ErroMovX/divisorX)*/);

        vectorY = (-Math.sin(-angulo_robo / 180 * Math.PI) * (gamepad1.left_stick_x/*+ (ErroMovY/divisorY)*/))
                + Math.cos(-angulo_robo / 180 * Math.PI) * (gamepad1.left_stick_y /*+(-ErroMovX/divisorX)*/);

        leftBack.setPower((vectorY * 0.9 + vectorX * 1) * Speed + ((gamepad1.left_trigger - gamepad1.right_trigger)  + correcao_angular ));
        leftFront.setPower((vectorY * 0.9 + vectorX * -1) * Speed + ((gamepad1.left_trigger - gamepad1.right_trigger)  + correcao_angular ));
        rightBack.setPower((vectorY * 0.9 + vectorX * -1) * Speed - ((gamepad1.left_trigger - gamepad1.right_trigger)  + correcao_angular ));
        rightFront.setPower((vectorY * 0.9 + vectorX * 1) * Speed - ((gamepad1.left_trigger - gamepad1.right_trigger) + correcao_angular));

        if (RotacaoRobo.seconds() < 1) {
            correcao_angular = 0;
            angulo_desejado = angulo_robo;
            last_pose_X = pose_atual.position.x;
            last_pose_Y = pose_atual.position.y;
        }
        else {
            int rotate;
            if (angulo_robo >= 0) {
                rotate = 360;
            } else {
                rotate = -360;
            }

            double corr_1 = angulo_robo - angulo_desejado;
            double corr_2 = angulo_desejado + rotate - angulo_robo;

            double div_corr1 = 0.0185 + 0.002 * Math.max(0, 1.0 - (Math.abs(corr_1) / 10));
            double div_corr2 = 0.0185 + 0.002 * Math.max(0, 1.0 - (Math.abs(corr_2) / 10 + rotate));

            double erroAcumulado = 0;
            erroAcumulado += (Math.abs(corr_1) < Math.abs(corr_2) ? corr_1 : corr_2);

            erroAcumulado = (Math.abs(erroAcumulado) > 10 ? 0 : Math.max(Math.min(erroAcumulado, 10), -10));

            double Ki = 0.0005;

            double correcaoI = erroAcumulado * Ki;

            double correcaoP;

            if (Math.abs(corr_1) > Math.abs(corr_2)) {
                correcaoP = corr_2 * div_corr2;
            } else {
                correcaoP = -corr_1 * div_corr1;
            }
            correcao_angular = Math.min(Math.max(correcaoP + correcaoI, -0.7), 0.7);
        }



        // Ajuste de velocidade baseado nos triggers do gamepad
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

        }

        // Reduzindo a velocidade caso o motor `slide_vertical_e` ultrapasse uma posição

    } // OK

    private void Telemetria(){


        telemetry.addData("","");

       // telemetry.addData("Posição", pose);
        telemetry.addData("x", pose_atual.position.x);
        telemetry.addData("y", pose_atual.position.y);

        telemetry.addData("lastx", last_pose_X);
        telemetry.addData("lasty", last_pose_Y);

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
        telemetry.addData("Ângulo Robô", angulo_robo % 360);
        telemetry.addData("Ângulo Desejado", angulo_desejado);
        telemetry.addData("X", vectorX);
        telemetry.addData("Y", vectorY);
        telemetry.addData("", "");

        // Atualiza os dados na tela
        telemetry.update();
    }

}
