package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Teste_Motores extends LinearOpMode {

    boolean Base_Motriz_State = false, Slide_Horizontal_State = false,
            Slide_Vertical_State = false, Sequencia_Intake_State = false,
            Transferencia_State = false, Controle_Garras_State = true, Controle_Rotacao_State = false,
            Clipar_State = false, Procurar_sample = false,Procurar_sample_reset = false, Rotacionar_reset_State = true, IntRotacao_state = false, Auto_Rot_state = false;

    boolean aux_intake_cicle = true, intake_remote_switch = false, Intake_state = true,
            Intake_reset_state = true, sample_coletado = false, reset_coleta = false,
            pre_coleta = false, pre_coleta_reset = false, outtake_reset = false,
            outtake_state = false, yellow_sample = true, Retorna_Slide_Horizontal_State = false,
            Coleta_Clipe_state = false, Coletar_limelight = false, Slide_Auto = false, down_state, down_reset_State;;

    boolean braco_outtake_encoder_reset = false;
    boolean alliance_color = false; // true = red, false = blue.
    double integral = 0;
    // === PID Gains ===
    public static double kP = 0.003;
    public static double kI = 0.0000;
    public static double kD = 0.0001;

    public static double kmax = -0.25;
    public static double kmin = 0.25;


    public static double up_intake_vetical_position_braco = 0.25;
    public static double up_intake_vetical_position_pulso = 0.25;
    public static double up_intake_vetical_position_rot_pulso = 0.25;
    public static double up_intake_horizontal_position_braco = 0.25;
    public static double up_intake_horizontal_position_pulso = 0.25;
    public static double up_intake_horizontal_position_rot_pulso = 0.25;
    // === PID State ===
    double erroAnterior = 0;
    double Timer = 0;
    double derivada;
    private RevBlinkinLedDriver LEDs;

    private DcMotor leftBack, leftFront, rightBack, rightFront, slide_vertical_d, slide_vertical_e, slide_horizontal, BracoOuttake;

    private Servo GarraIntake, BracoIntake, GarraOuttake, PulsoIntake, RotacaoOuttake, RotacaoIntake;

    private DigitalChannel FimDeCurso;

    private Limelight3A limelight;

    int pulse_reset, posicao_desejada_slide_horizontal = 0, posicao_desejada_slide_vertical = 0, Pulse_bolinha, posicao_desejada_braco_outtake = 0, Pulse_quadrado = 0;

    int leitura_R, leitura_G, leitura_B, leitura_A, leitura_argb;

    double  controle_rotacao,Pos_servo,el_speed, Speed, angulo_desejado, angulo_robo, vectorX, vectorY, correcao_angular, potencia_max_correcao_angular, potencia_min_correcao_angular, potencia_min_bracoouttake, potencia_max_bracoouttake,
            BracoIntake_posicao_desejada = 0;

    double angle, centerX, centerY, area, samples;
    double CorrLimeX, CorrLimeY, ErroLimeX, LastError;
    double angintake;

    private VoltageSensor ControlHub_VoltageSensor;
    //  AprilTagLocalization_Oscar_v01 AprilTag = new AprilTagLocalization_Oscar_v01();
    double T_max = 0;
    double T_min = 1000;
    double T_atual = 0;

    ColorSensor SensorDeCor;

    ElapsedTime RotacaoRobo, outTimer, TimerColeta, meuTempo, TempoColetaeTranferencia,
            Clipar, Transfer_timer, Intake_timer, Clear_sample_cicle_timer, Tempo_de_ciclo,
            Controle_garra_auto_reset, MASTER_TIMER, Transferencia_timer,
            Clipar_timer, Coleta_Clipe_timer, Timer_limelight,deltaTime, reset_braco;



    @Override
    public void runOpMode() throws InterruptedException {

        // Mapeamento dos sensores e motores
        {
            SensorDeCor = hardwareMap.get(ColorSensor.class, "SensorDeCor");
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

            LEDs = hardwareMap.get(RevBlinkinLedDriver.class, "LEDs");
            FimDeCurso = hardwareMap.get(DigitalChannel.class, "FimDeCursoSV");
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
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
            slide_vertical_d.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide_vertical_d.setDirection(DcMotor.Direction.REVERSE);
            slide_vertical_d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            slide_vertical_e.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide_vertical_e.setDirection(DcMotor.Direction.FORWARD);
            slide_vertical_e.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            slide_horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide_horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide_horizontal.setDirection(DcMotor.Direction.FORWARD);
            slide_horizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


            BracoOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        // Inicialização dos temporizadores
        {
            TimerColeta = new ElapsedTime();
            outTimer = new ElapsedTime();
            meuTempo = new ElapsedTime();
            TempoColetaeTranferencia = new ElapsedTime();
            Clipar = new ElapsedTime();
            Transfer_timer = new ElapsedTime();
            Intake_timer = new ElapsedTime();
            Clear_sample_cicle_timer = new ElapsedTime();
            Tempo_de_ciclo = new ElapsedTime();
            Controle_garra_auto_reset = new ElapsedTime();
            MASTER_TIMER = new ElapsedTime();
            Transferencia_timer = new ElapsedTime();
            Clipar_timer = new ElapsedTime();
            Coleta_Clipe_timer = new ElapsedTime();

            Intake_timer.reset();
            Transfer_timer.reset();
            Clear_sample_cicle_timer.reset();
            TimerColeta.reset();
            outTimer.reset();
            meuTempo.reset();
            TempoColetaeTranferencia.reset();
            Clipar.reset();
            Tempo_de_ciclo.reset();
            Controle_garra_auto_reset.reset();
            Transferencia_timer.reset();
            Clipar_timer.reset();
            Coleta_Clipe_timer.reset();
        }
        pulse_reset = 1;
        Speed = 1;
        angulo_desejado = 0;
        posicao_desejada_slide_horizontal = 0;

        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");


        // Aguardando o início da execução
        waitForStart();
        posicao_desejada_braco_outtake = 550;
        posicao_desejada_slide_vertical = 0;
        MASTER_TIMER.reset();

        // Execução principal do OpMode
        if (opModeIsActive()) {

            meuTempo.reset();
            angulo_desejado = 0;
            posicao_desejada_slide_vertical = 0;

            Thread TMoverBase = new Thread(this::Base_Motriz);
            Thread TPrincipal = new Thread(this::CodigoPrincipal);

            TMoverBase.start();
            TPrincipal.start();


            TMoverBase.join(120000);
            TPrincipal.join(120000);

            // Loop principal

        }
    }


    //-------Funções de Posição--------------------------------------------------------------------------------------------//


    //-------Novas Funções-----------------------------------------------------------------------------------------

    public void CodigoPrincipal() {
        while (opModeIsActive()) {
            Telemetria();
        }
    }

    private void Base_Motriz() {
        while (opModeIsActive()) {
           if(gamepad1.triangle) leftFront.setPower(1);
           else  leftFront.setPower(0);

            if(gamepad1.circle) rightFront.setPower(1);
            else rightFront.setPower(0);

            if(gamepad1.cross)  leftBack.setPower(1);
            else leftBack.setPower(0);

            if(gamepad1.square) rightBack.setPower(1);
            else rightBack.setPower(0);
//-----------------------SERVOS--------------------------------
            //Garras
            if (!gamepad1.dpad_up) {
                outtake_reset = true;
            }

            if (gamepad1.dpad_up && outtake_reset) {
                outtake_state = !outtake_state;
                outtake_reset = false;
            }
            if (gamepad1.dpad_up) {
                if (outtake_state) GarraOuttake.setPosition(1);
                else GarraOuttake.setPosition(0.3);
            }


            if (!gamepad1.dpad_up) Intake_reset_state = true;


            if (gamepad1.dpad_up && Intake_reset_state) {
                Intake_state = !Intake_state;
                Intake_reset_state = false;
            }

            if (Intake_state) GarraIntake.setPosition(0.45);
            else GarraIntake.setPosition(0.75);
//-----------------------------------------------------------------------------
                //Anguladores
            if (!gamepad1.dpad_left) Rotacionar_reset_State = true;

            if (gamepad1.dpad_left && Rotacionar_reset_State){
                Coletar_limelight = false;
                IntRotacao_state = !IntRotacao_state;
                Rotacionar_reset_State = false;
            }
            if (!Rotacionar_reset_State) {
                if (IntRotacao_state) {

                    RotacaoIntake.setPosition(0.45);
                    RotacaoOuttake.setPosition(0);

                } else if (!IntRotacao_state) {
                    RotacaoIntake.setPosition(0.85);
                    RotacaoOuttake.setPosition(1);

                }
            }
            //-----------------------------
            //Braço
            if (!gamepad1.dpad_down) down_reset_State = true;


            if (gamepad1.dpad_down && down_reset_State){
                down_state = !down_state;
                down_reset_State = false;
            }
            if (!down_reset_State) {
                if (down_state) {

                    BracoIntake.setPosition(1);
                    PulsoIntake.setPosition(0.3);

                } else if (!down_state) {
                    BracoIntake.setPosition(0.4);
                    PulsoIntake.setPosition(0.5);

                }
            }


        }
    }


    private void Telemetria() {



        telemetry.addData("**Motores**", "");
        telemetry.addData("Motor LB (Esquerda Traseira)", leftBack.getCurrentPosition());
        telemetry.addData("Motor LF (Esquerda Dianteira)", leftFront.getCurrentPosition());
        telemetry.addData("Motor RB (Direita Traseira)", rightBack.getCurrentPosition());
        telemetry.addData("Motor RF (Direita Dianteira)", rightFront.getCurrentPosition());

        telemetry.addData("Motor Braço Outtake", BracoOuttake.getCurrentPosition());
        telemetry.addData("", "");

        telemetry.addData("**Slides**", "");
        telemetry.addData("Slide Horizontal (Posição)", slide_horizontal.getCurrentPosition());
        telemetry.addData("Slide Horizontal (Potência)", slide_horizontal.getPower());

        telemetry.addData("Slide Vertical (Posição)", slide_vertical_e.getCurrentPosition());
        telemetry.addData("Slide Vertical (Potência)", slide_vertical_e.getPower());
        telemetry.addData("", "");

        telemetry.addData("**Sensores**", "");
        telemetry.addData("Sensor Fim de Curso", FimDeCurso.getState());
        telemetry.addData("Sensor de Cor (Vermelho)", leitura_R);
        telemetry.addData("Sensor de Cor (Verde)", leitura_G);
        telemetry.addData("Sensor de Cor (Azul)", leitura_B);
        telemetry.addData("Sensor de Cor (Alpha)", leitura_A);


        telemetry.addData("", "");

        // Atualiza os dados na tela
        telemetry.update();
    }
}
