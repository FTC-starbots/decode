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

    boolean Movimento_robo = false,Base_Motriz_State = false, Slide_Horizontal_State = false,
            Slide_Vertical_State = false, Sequencia_Intake_State = false,
            Transferencia_State = false, Controle_Garras_State = true, Controle_Rotacao_State = false,
            Clipar_State = false, Procurar_sample = false,Procurar_sample_reset = false, Rotacionar_reset_State = true, IntRotacao_state = false, Auto_Rot_state = false;

    boolean aux_intake_cicle = true, intake_remote_switch = false, Intake_state = true,
            Intake_reset_state = true, sample_coletado = false, reset_coleta = false,
            pre_coleta = false, pre_coleta_reset = false, outtake_reset = false,
            outtake_state = false, yellow_sample = true, Retorna_Slide_Horizontal_State = false,
            Coleta_Clipe_state = false, Coletar_limelight = false, Slide_Auto = false;

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

    //private Limelight3A limelight;

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

    Pose2d pose_atual;

    double ErroMovX, ErroMovY, last_pose_X, last_pose_Y;
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
            slide_vertical_d.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide_vertical_d.setDirection(DcMotor.Direction.REVERSE);
            slide_vertical_d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            slide_vertical_e.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide_vertical_e.setDirection(DcMotor.Direction.FORWARD);
            slide_vertical_e.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            slide_horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide_horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide_horizontal.setDirection(DcMotor.Direction.FORWARD);
            slide_horizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


            BracoOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BracoOuttake.setDirection(DcMotor.Direction.REVERSE);
        }
        // Inicialização dos temporizadores
        {
            TimerColeta = new ElapsedTime();
            outTimer = new ElapsedTime();
            RotacaoRobo = new ElapsedTime();
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
            Timer_limelight = new ElapsedTime();
            deltaTime = new ElapsedTime();
            reset_braco = new ElapsedTime();

            reset_braco.reset();
            Intake_timer.reset();
            Transfer_timer.reset();
            Clear_sample_cicle_timer.reset();
            TimerColeta.reset();
            outTimer.reset();
            RotacaoRobo.reset();
            meuTempo.reset();
            TempoColetaeTranferencia.reset();
            Clipar.reset();
            Tempo_de_ciclo.reset();
            Controle_garra_auto_reset.reset();
            Transferencia_timer.reset();
            Clipar_timer.reset();
            Coleta_Clipe_timer.reset();
            Timer_limelight.reset();
            deltaTime.reset();
        }
        pulse_reset = 1;
        Speed = 1;
        angulo_desejado = 0;
        posicao_desejada_slide_horizontal = 0;

        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        // initAprilTag();

      //  limelight.start();

        // Aguardando o início da execução
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();
        angintake = 0.45;
        RotacaoIntake.setPosition(0.45);
        IntRotacao_state = true;
        rest_position();
        posicao_desejada_slide_vertical = -200;
        posicao_desejada_braco_outtake = 0;
        RotacaoOuttake.setPosition(0.5);
        MASTER_TIMER.reset();

        // Execução principal do OpMode
        if (opModeIsActive()) {
            angulo_desejado = 0;

            while (opModeIsActive()) {

                meuTempo.reset();
                Base_Motriz();
                //_______________________________________________________________________________
                posicao_servos_intake();
                //________________________________________________________________________________


                if (gamepad1.triangle || gamepad1.square  || Sequencia_Intake_State ) {
                    Controle_Garras_State = true;
                    Controle_garra_auto_reset.reset();
                    sample_coletado = false;
                }

                if (Controle_garra_auto_reset.seconds() > 1) {
                    Controle_Garras_State = false;
                }

                    CorrLimeY = 0;
                    CorrLimeX = 0;
                    controle_rotacao = 0.5;


                if(Coletar_limelight){
                    Desliga_Motores_base();
                    Sequencia_Intake_State = true;
                    Intake_timer.reset();
                    Procurar_sample = false;
                    Coletar_limelight = false;
                }

                if (gamepad1.cross) {
                    RotacaoOuttake.setPosition(0.5);
                    posicao_desejada_slide_vertical = 200;
                }
                //--DPADS------------------------------------------------------------


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

                //--BUMPERS------------------------------------------------------------------------------------
                if (gamepad1.right_bumper || gamepad1.left_bumper) {
                    angintake = 0.45;
                    RotacaoIntake.setPosition(0.45);
                    IntRotacao_state = true;
                    rest_position();
                    if (gamepad1.right_bumper) {
                        outtake_state = true;
                        RotacaoOuttake.setPosition(0.45);
                        GarraOuttake.setPosition(0.85);
                        Coleta_Clipe_state = true;
                        Coleta_Clipe_timer.reset();
                    }
                    if (gamepad1.left_bumper) {
                        GarraOuttake.setPosition(0.75);
                        RotacaoOuttake.setPosition(1);
                        posicao_desejada_slide_vertical = 3500;
                        posicao_desejada_braco_outtake = 430;

                    }
                }
                if(gamepad1.touchpad){
                    if (slide_vertical_e.getCurrentPosition() < 2500) {
                        Clipar_State = true;
                        Clipar_timer.reset();
                    }else{
                        GarraOuttake.setPosition(0.3);
                        Movimenta_base(0.4);
                        outtake_state = true;
                        RotacaoOuttake.setPosition(0.5);
                        posicao_desejada_slide_vertical = -200;
                        posicao_desejada_braco_outtake = 500;
                    }
                }


                //--DEMAIS-BOTÕES-------------------------------------------------------------------------------
                if (gamepad1.options) angulo_robo = 0;
                //-----------------------------------------------------------------------------------------------
                if (Math.abs(gamepad1.right_stick_y) > 0.4) Slide_Horizontal();
                else if (Retorna_Slide_Horizontal_State) Retorna_Slide_Horizontal();
                else if (Slide_Auto) slide_horizontal.setPower(CorrLimeY);
                else slide_horizontal.setPower(0);

                if (Clipar_State) Clipar();

                if (Coleta_Clipe_state) Coleta_Clipe();

                Slide_Vertical();

                Sequencia_Intake();

                if (Transferencia_State) Transferencia();

                if (Controle_Garras_State) Controle_Garras();
                Telemetria();
                Tempo_de_ciclo.reset();
            }
        }
        //limelight.stop();
    }


    //-------Funções de Posição--------------------------------------------------------------------------------------------//


    /*private void up_intake_position() { // posição de coleta - OK
        BracoIntake.setPosition(0.53); // BracoIntake.setPosition(0.47);
        PulsoIntake.setPosition(0.2); // PulsoIntake.setPosition(0.2);

    }
    */
    private void posicao_servos_intake() {
        if (Pos_servo == 1) up_intake_position();
        if (Pos_servo == 2) mid_intake_position_internal();
        if (Pos_servo == 3) down_intake_position_internal();
        if (Pos_servo == 4) downmore_intake_position_internal();
        if (Pos_servo == 5) color_sample_position();
        if (Pos_servo == 6) mid_transfer_intake_position();
        if (Pos_servo == 7) transfer_intake_position();
        if (Pos_servo == 8) rest_position();

    }
    private void up_intake_position() {// posição de coleta - OK
        Pos_servo = 1;
        double adj = 0.11*(((double)slide_horizontal.getCurrentPosition())/1640);
        BracoIntake.setPosition(0.3+adj); // BracoIntake.setPosition(0.47);
        PulsoIntake.setPosition(0.192-adj/3); // PulsoIntake.setPosition(0.2);

    }
    private void mid_intake_position_internal() { //Primeiro estágio coleta - por dentro do clipe
        Pos_servo = 2;
        double adj = 0.12*(((double)slide_horizontal.getCurrentPosition())/1640);
        BracoIntake.setPosition(0.25+adj);
        PulsoIntake.setPosition(0.20-adj/3);
    }

    private void down_intake_position_internal() {//Segundo estágio coleta - por dentro do clipe
        Pos_servo = 3;
        double adj = 0.12*(((double)slide_horizontal.getCurrentPosition())/1640);
        BracoIntake.setPosition(0.2+adj);
        PulsoIntake.setPosition(0.23-adj/2);
    }

    private void downmore_intake_position_internal() {//Terceiro estágio coleta - por dentro do clipe
        Pos_servo = 4;
        double hfactor = 0;
        if(!IntRotacao_state) hfactor = 0.05;
        double adj = 0.08*(((double)slide_horizontal.getCurrentPosition())/1640);
        BracoIntake.setPosition(0.13+adj);
        PulsoIntake.setPosition(0.25-adj/3+hfactor);

    }

    private void transfer_intake_position() {
        Pos_servo = 7;
        BracoIntake.setPosition(0.9);
        PulsoIntake.setPosition(0.3);
    }

    private void mid_transfer_intake_position() {
        Pos_servo = 6;
        BracoIntake.setPosition(0.98);
        PulsoIntake.setPosition(0.2);
    }
    private void rest_position() { // OK
        Pos_servo = 8;
        BracoIntake.setPosition(1);
        PulsoIntake.setPosition(0.25);
    }
    private void color_sample_position() {
        Pos_servo = 5;
        BracoIntake.setPosition(0.78);
        PulsoIntake.setPosition(0.2);
    }

    //-------Novas Funções-----------------------------------------------------------------------------------------

    private void Base_Motriz(){

        angulo_robo = (rightFront.getCurrentPosition() - leftBack.getCurrentPosition()) / 249.0;

        double divisorX = 5 + 10 * Math.max(0, 1.0 - (Math.abs(ErroMovX) / 4.0));
        double divisorY = 5 + 10 * Math.max(0, 1.0 - (Math.abs(ErroMovY) / 4.0));

        vectorX = (Math.cos(-angulo_robo / 180 * Math.PI) * (gamepad1.left_stick_x+ (ErroMovY/divisorY)))
                + Math.sin(-angulo_robo / 180 * Math.PI) * (-gamepad1.left_stick_y+(-ErroMovX/divisorX));

        vectorY = (-Math.sin(-angulo_robo / 180 * Math.PI) * (gamepad1.left_stick_x+ (ErroMovY/divisorY)))
                + Math.cos(-angulo_robo / 180 * Math.PI) * (-gamepad1.left_stick_y +(-ErroMovX/divisorX));

        el_speed = controle_rotacao * (1 - ((double) slide_horizontal.getCurrentPosition() / 2500));
        leftBack.setPower((vectorY * 0.9 + vectorX * -1) * Speed + (el_speed *  (gamepad1.right_trigger - gamepad1.left_trigger)  + correcao_angular ));
        leftFront.setPower((vectorY * 0.9 + vectorX * 1) * Speed + (el_speed *  (gamepad1.right_trigger - gamepad1.left_trigger)  + correcao_angular ));
        rightBack.setPower((vectorY * 0.9 + vectorX * 1) * Speed - (el_speed *  (gamepad1.right_trigger - gamepad1.left_trigger)  + correcao_angular ));
        rightFront.setPower((vectorY * 0.9 + vectorX * -1) * Speed - (el_speed * (gamepad1.right_trigger - gamepad1.left_trigger) + correcao_angular));

        if (RotacaoRobo.seconds() < 1) {
            correcao_angular = 0;
            angulo_desejado = angulo_robo;
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
        if (slide_vertical_e.getCurrentPosition() > 2000) {
            Speed = Speed * 1500 / slide_vertical_e.getCurrentPosition();

        }

    } // OK

    private void Slide_Horizontal(){
        /*double potencia_slide_horizontal;
        if(Math.abs(gamepad1.right_stick_y) > 0.95)potencia_slide_horizontal = (-1 * gamepad1.right_stick_y);
        else potencia_slide_horizontal = (-0.8 * gamepad1.right_stick_y);
        // Evitar movimento para baixo caso o motor esteja em posição zero
        if (slide_horizontal.getCurrentPosition() <= 0 && potencia_slide_horizontal < 0) {
            slide_horizontal.setPower(0);
        }else {
            slide_horizontal.setPower(potencia_slide_horizontal );
        }*/
    } // OK

    private void Clipar(){
        if (0 < Clipar_timer.seconds() && 0.15 >= Clipar_timer.seconds()) {
            Controle_leds(6);
            posicao_desejada_braco_outtake = -200;
            GarraOuttake.setPosition(0.5);
        }
        else if (0.2 < Clipar_timer.seconds() && 0.5 >= Clipar_timer.seconds()) {
            posicao_desejada_slide_vertical = -200;
            posicao_desejada_braco_outtake = -200;
            GarraOuttake.setPosition(0.3);
            outtake_state = false;
            angulo_desejado = 0;
            Movimenta_base(0.5);
        }
        else if (0.5 < Clipar_timer.seconds() && 1 >= Clipar_timer.seconds()){
            Controle_leds(4);
            Desliga_Motores_base();
            Clipar_State = false;
            RotacaoOuttake.setPosition(0.5);
        }
        else if (1 < Clipar_timer.seconds() && 1.1 >= Clipar_timer.seconds()) {
            BracoOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BracoOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BracoOuttake.setDirection(DcMotor.Direction.REVERSE);

        }

    }

    private void Coleta_Clipe(){
        if (0 < Coleta_Clipe_timer.seconds() && 0.2 >= Coleta_Clipe_timer.seconds() ) {
            if(slide_vertical_e.getCurrentPosition() < 100 && posicao_desejada_slide_vertical != 50) posicao_desejada_slide_vertical = 50;
        }
        if (0.2 < Coleta_Clipe_timer.seconds() && 0.5 >= Coleta_Clipe_timer.seconds() ) {
            Controle_leds(7);
            RotacaoOuttake.setPosition(0);

        }
        else if (0.5 < Coleta_Clipe_timer.seconds() && 0.65 >= Coleta_Clipe_timer.seconds()) {
            posicao_desejada_slide_vertical = 1200;
            posicao_desejada_braco_outtake = 245;
        }
        else if (0.65 < Coleta_Clipe_timer.seconds() && 1 >= Coleta_Clipe_timer.seconds()) {

        }
        else if (1 < Coleta_Clipe_timer.seconds() && 1.5 >= Coleta_Clipe_timer.seconds()){
            Controle_leds(4);
            Coleta_Clipe_state = false;
        }

        if (0.8 < Coleta_Clipe_timer.seconds() &&
                (Math.abs(gamepad1.left_stick_x)>0.05 ||
                        Math.abs(gamepad1.left_stick_y)>0.05)){
            Controle_leds(4);
            Coleta_Clipe_state = false;

        }


    }
    private void Slide_Vertical(){
        //--------Braço Outtake-----------------------------------------------------------

        double potencia_slide_vertical;
        double potencia_braco_outtake = Math.min(Math.max(0.004 * (posicao_desejada_braco_outtake - BracoOuttake.getCurrentPosition())
                , potencia_min_bracoouttake), potencia_max_bracoouttake);

        if(posicao_desejada_braco_outtake < 0 && !braco_outtake_encoder_reset){
            reset_braco.reset();
            braco_outtake_encoder_reset = true;
        }
        if (reset_braco.seconds() >= 1.5 && braco_outtake_encoder_reset) {
            BracoOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BracoOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BracoOuttake.setDirection(DcMotor.Direction.REVERSE);
            BracoOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            posicao_desejada_braco_outtake = 0;
            braco_outtake_encoder_reset = false;

        }

        BracoOuttake.setPower(potencia_braco_outtake);



        if (slide_vertical_e.getCurrentPosition() >= 4000) {
            potencia_max_bracoouttake = 0.3; potencia_min_bracoouttake = -0.3;
        } else if (slide_vertical_e.getCurrentPosition() < 4000){
            potencia_max_bracoouttake = 0.5; potencia_min_bracoouttake = -0.5;
        }
        //--------------------------------------------------------------------------------
        // Controle de movimento vertical com controle PID
        double erro = posicao_desejada_slide_vertical - slide_vertical_e.getCurrentPosition();
        double Kp = 0.01;
        double Ki = 0.001;
        double Ap = Kp * erro;
        double Ai = 0;
        Ai = Ai + Ki * erro;  // Acumulando o erro para o integral
        potencia_slide_vertical = Math.min(Math.max(-(Ap + Ai), -1), 0.8); // Limitar a potência

        // Evitar movimento para baixo caso o motor esteja em posição zero
        if (FimDeCurso.getState() && posicao_desejada_slide_vertical <= 0) {
            slide_vertical_d.setPower(0);
            slide_vertical_e.setPower(0);

            slide_vertical_d.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide_vertical_d.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide_vertical_d.setDirection(DcMotor.Direction.REVERSE);
            slide_vertical_d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            slide_vertical_e.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide_vertical_e.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide_vertical_e.setDirection(DcMotor.Direction.FORWARD);
            slide_vertical_e.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            posicao_desejada_slide_vertical = 0;
            posicao_desejada_braco_outtake = -200;
        } else {
            slide_vertical_d.setPower(potencia_slide_vertical);
            slide_vertical_e.setPower(potencia_slide_vertical);
        }
        //Slide_Vertical_State = false;
    }

    private void Telemetria(){
        telemetry.addData("SAMPLES", samples);
        telemetry.addData("Coleta", Coletar_limelight);
        telemetry.addData("LimeX", centerX);
        telemetry.addData("LimeY", centerY);
        telemetry.addData("Ang", angle);

        telemetry.addData("","");

       // telemetry.addData("Posição", pose);
        telemetry.addData("x", pose_atual.position.x);
        telemetry.addData("y", pose_atual.position.y);

        telemetry.addData("lastx", last_pose_X);
        telemetry.addData("lasty", last_pose_Y);




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

        telemetry.addData("**Motores**", "");
        telemetry.addData("Motor LB (Esquerda Traseira)", leftBack.getCurrentPosition());
        telemetry.addData("Motor LF (Esquerda Dianteira)", leftFront.getCurrentPosition());
        telemetry.addData("Motor RB (Direita Traseira)", rightBack.getCurrentPosition());
        telemetry.addData("Motor RF (Direita Dianteira)", rightFront.getCurrentPosition());
        telemetry.addData("Motor Braço Outtake", BracoOuttake.getCurrentPosition());
        telemetry.addData("Motor Braço Outtake (Posição Desejada)", posicao_desejada_braco_outtake);
        telemetry.addData("", "");

        telemetry.addData("**Slides**", "");
        telemetry.addData("Slide Horizontal (Posição)", slide_horizontal.getCurrentPosition());
        telemetry.addData("Slide Horizontal (Potência)", slide_horizontal.getPower());
        telemetry.addData("Slide Horizontal (Posição Desejada)", posicao_desejada_slide_horizontal);

        telemetry.addData("Slide Vertical (Posição)", slide_vertical_e.getCurrentPosition());
        telemetry.addData("Slide Vertical (Potência)", slide_vertical_e.getPower());
        telemetry.addData("Slide Vertical (Posição Desejada)", posicao_desejada_slide_vertical);
        telemetry.addData("", "");

        telemetry.addData("**Sensores**", "");
        telemetry.addData("Sensor Fim de Curso", FimDeCurso.getState());
        telemetry.addData("Sensor de Cor (Vermelho)", leitura_R);
        telemetry.addData("Sensor de Cor (Verde)", leitura_G);
        telemetry.addData("Sensor de Cor (Azul)", leitura_B);
        telemetry.addData("Sensor de Cor (Alpha)", leitura_A);
        telemetry.addData("Sensor de Cor (Argb)", leitura_argb);


        telemetry.addData("", "");



        telemetry.addData("**Outros**", "");
        telemetry.addData("Ângulo Robô", angulo_robo % 360);
        telemetry.addData("Ângulo Desejado", angulo_desejado);
        telemetry.addData("X", vectorX);
        telemetry.addData("Y", vectorY);
        telemetry.addData("", "");
        telemetry.addData("Garra intake", Intake_state);
        telemetry.addData("Intake timer", Intake_timer.seconds());

        telemetry.addData("**Temporizadores**", "");
        telemetry.addData("Timer Delay", outTimer.time());
        telemetry.addData("Timer da Coleta", TimerColeta.time());
        telemetry.addData("Transfer timer", Transfer_timer.seconds());
        telemetry.addData("", "");

        // Atualiza os dados na tela
        telemetry.update();
    }


    private void Sequencia_Intake(){
        boolean break_sequencia_intake = false;
        RotacaoIntake.setPosition(angintake);
        if (0 < Intake_timer.seconds() && 0.15 >= Intake_timer.seconds()) {

            Intake_state = true;
            RotacaoOuttake.setPosition(0);
            potencia_min_bracoouttake = -0.8;
            potencia_max_bracoouttake = 0.8;
            posicao_desejada_slide_vertical = 1600;
            posicao_desejada_braco_outtake = 1100;
            outtake_state = false;
            GarraOuttake.setPosition(0.3);

            mid_intake_position_internal();

        } else if (0.2 < Intake_timer.seconds() && 0.4 >= Intake_timer.seconds()) {
            down_intake_position_internal();

        } else if (0.4 < Intake_timer.seconds() && 1 >= Intake_timer.seconds()) {
            downmore_intake_position_internal();
        } else if (1 < Intake_timer.seconds() && 1.2 >= Intake_timer.seconds()) {
            Intake_state = false;
            if (aux_intake_cicle) {
                intake_remote_switch = true;
                Intake_reset_state = true;
                aux_intake_cicle = false;
                sample_coletado = true;
            }
        }
        else if (1.2 < Intake_timer.seconds() && 1.4 >= Intake_timer.seconds()) {
            up_intake_position();
            Rotacionar_reset_State = false;
            angintake = 0.45;
            IntRotacao_state = true;


        } else if (1.4 < Intake_timer.seconds() && 2 >= Intake_timer.seconds()) {
            Rotacionar_reset_State = false;
            aux_intake_cicle = true;
            Sensor_de_Cor();
            posicao_desejada_slide_vertical = 1300;

            if(leitura_R>leitura_B && leitura_R>leitura_G){
                Controle_leds(2);
                yellow_sample = false;
                Retorna_Slide_Horizontal_State = true;
            }

            else if(leitura_B>leitura_R && leitura_B>leitura_G){
                Controle_leds(3);
                yellow_sample = false;
                Retorna_Slide_Horizontal_State = true;
            }
            else if(leitura_G+leitura_R+leitura_G<1100){
                Controle_leds(5);
                pre_coleta = true;
                Coletar_limelight = false;
                Sequencia_Intake_State = false;
                yellow_sample = false;
                Retorna_Slide_Horizontal_State = false;
            }
            else {
                Controle_leds(1);
                break_sequencia_intake = true;
                yellow_sample = true;
                Retorna_Slide_Horizontal_State = true;
            }
            angintake = 0.45;
            RotacaoIntake.setPosition(0.45);
            IntRotacao_state = true;
            if(yellow_sample) {
                angintake = 0.45;
                RotacaoIntake.setPosition(0.45);// OK
                IntRotacao_state = true;
                color_sample_position();

            }
            else{
                angintake = 0.45;
                RotacaoIntake.setPosition(0.45);// OK
                IntRotacao_state = true;
                color_sample_position();
                posicao_desejada_braco_outtake = 0;
                posicao_desejada_slide_vertical = 150;
            }

        }else if (2.1 < Intake_timer.seconds() && 2.2 >= Intake_timer.seconds()){

            if(yellow_sample){
                mid_transfer_intake_position();
                angintake = 0.45;
                RotacaoIntake.setPosition(0.45);
                IntRotacao_state = true;
            }
        }
        else if (2.2 < Intake_timer.seconds() && 2.5 >= Intake_timer.seconds()) {
            if(yellow_sample){
                angintake = 0.45;
                RotacaoIntake.setPosition(0.45);
                IntRotacao_state = true;
                transfer_intake_position();
            }
            if(Retorna_Slide_Horizontal_State) Retorna_Slide_Horizontal();
            if (break_sequencia_intake) {
                Coletar_limelight = false;
                Sequencia_Intake_State = false;
            }
        } else {
            Coletar_limelight = false;
            Sequencia_Intake_State = false;
        }

    }

    private void Retorna_Slide_Horizontal(){

        if (slide_horizontal.getCurrentPosition()>5){
            slide_horizontal.setPower(-1);
        }else{
            if(yellow_sample) {
                Transferencia_timer.reset();
                Transferencia_State = true;
            }
            slide_horizontal.setPower(0);
            Retorna_Slide_Horizontal_State = false;
            Slide_Horizontal_State = true;
        }


    }

    private void Transferencia(){

        if (0.5 < Transferencia_timer.seconds() && 0.8 >= Transferencia_timer.seconds())
        {

            posicao_desejada_slide_vertical = 575;
        }
        else if (0.8 < Transferencia_timer.seconds() && 1.2 >= Transferencia_timer.seconds())
        {
            GarraOuttake.setPosition(1);
            outtake_state = true;
        }
        else if (1.2 < Transferencia_timer.seconds() && 1.5 >= Transferencia_timer.seconds())
        {
            angintake = 0.45;
            RotacaoIntake.setPosition(0.45);
            IntRotacao_state = true;
            rest_position();
            GarraOuttake.setPosition(1);
            pre_coleta_reset = false;
            posicao_desejada_slide_vertical = 850;
            posicao_desejada_braco_outtake = 250;

        }
        else if (1.5 < Transferencia_timer.seconds() && 2 >= Transferencia_timer.seconds()) {
            potencia_min_bracoouttake = -0.6;
            potencia_max_bracoouttake = 0.6;
        }

    }

    private void Sensor_de_Cor(){
        leitura_B = SensorDeCor.blue();
        leitura_R = SensorDeCor.red();
        leitura_G = SensorDeCor.green();
        leitura_A = SensorDeCor.alpha();
        leitura_argb = SensorDeCor.argb();


    }

    private void Controle_Garras(){
        //------CIRCLE-(pré coleta)----------------------------------------
        if (!gamepad1.circle) pre_coleta_reset = true;
        else Transferencia_State = false;

        if (gamepad1.circle && pre_coleta_reset) {
            pre_coleta = !pre_coleta;
            pre_coleta_reset = false;
        }

        if (!Sequencia_Intake_State && gamepad1.circle){
            if(pre_coleta) {
                up_intake_position();
                Intake_state = true;
                GarraIntake.setPosition(0.55);

            }else{
                angintake = 0.45;
                RotacaoIntake.setPosition(0.45);
                IntRotacao_state = true;
                rest_position();
            }
        }




        //-----------Outtake-----------------------------------------------

        if (!gamepad1.triangle) {
            outtake_reset = true;
        }

        if (gamepad1.triangle && outtake_reset) {
            outtake_state = !outtake_state;
            outtake_reset = false;
        }
        if (gamepad1.triangle) {
            if (outtake_state) GarraOuttake.setPosition(1);
            else GarraOuttake.setPosition(0.3);
        }

        //------------Intake------------------------------------------------
        if (!gamepad1.triangle) Intake_reset_state = true;


        if (gamepad1.triangle && Intake_reset_state) {
            Intake_state = !Intake_state;
            Intake_reset_state = false;
        }


        if (Intake_state) GarraIntake.setPosition(0.45);
        else GarraIntake.setPosition(0.75);




//------------Rotacão------------------------------------------------
        if (!gamepad1.square) Rotacionar_reset_State = true;

        if (gamepad1.square && Rotacionar_reset_State){
            Coletar_limelight = false;
            IntRotacao_state = !IntRotacao_state;
            Rotacionar_reset_State = false;
        }
        if (!Rotacionar_reset_State) {
            if (IntRotacao_state) {
                Controle_leds(8);
                angintake = 0.45;
            } else if (!IntRotacao_state) {
                Controle_leds(9);
                angintake = 0.85;
            }
        }
    }

    private void LimelightUpdate(){
        //LLResult result = limelight.getLatestResult();
        //double[] pythonOutputs = result.getPythonOutput();
        //
        //angle = pythonOutputs[0];
        //centerX = pythonOutputs[1];
        //centerY = pythonOutputs[2];
        //area = pythonOutputs[3];
        //samples = pythonOutputs[4];
    }

    private void Controle_rotacao(){
        double rot_power = gamepad1.right_trigger-gamepad1.left_trigger;
        if (slide_horizontal.getCurrentPosition()>200) rot_power = rot_power*0.2;
        leftBack.setPower(rot_power);
        leftFront.setPower(rot_power);
        rightBack.setPower(-rot_power);
        rightFront.setPower(-rot_power);
    }

    private void Controle_leds(int padrao){
        /* 1 - Amarelo
        2- Vermelho
        3 - Azul
        4 - Verde
        5 - Roxo
        6 - Amarelo Piscando
        7- Vermelho Piscando
        8 - Azul Piscando
        9 - Verde Piscando
        10 - Roxo Piscando
         */


        if (padrao==1) LEDs.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

        else if (padrao==2) LEDs.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        else if (padrao==3) LEDs.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

        else if (padrao==4) LEDs.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY);

        else if (padrao==5) LEDs.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_GRAY);

        else if (padrao==6) LEDs.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);

        else if (padrao==7) LEDs.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER);

        else if (padrao==8) LEDs.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        else if (padrao==9) LEDs.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);

        else if (padrao==10) LEDs.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);

    }

    private void Desliga_Motores_base(){
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    private void Movimenta_base(double power){
        leftBack.setPower(power);
        leftFront.setPower(power);
        rightBack.setPower(power);
        rightFront.setPower(power);
    }

}
