package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

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

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;

@Config
@Autonomous


public class Autonomo_AzulGRANDE extends LinearOpMode {

    private static final Logger log = LoggerFactory.getLogger(Autonomo_AzulGRANDE.class);

    public class PIDs implements Action {
        public PIDs() {

            cicle_time = deltaTime.milliseconds();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            //executa o movimento do braço em loop
            double erro = velocidade_desejada + Atirador_1.getVelocity();
            double BatVoltage = myControlHubVoltageSensor.getVoltage();
            double feedfoward = 1 + (0.15 * (12.5-BatVoltage));
            double integrativo = 0.015 * (erro * cicle_time);
            double derivativo = 0 * ((erro - lasterror_PiDF_Shooter) / cicle_time);
            double proporcional = 0.00005 * erro;

            integral_PiDF_Shooter += integrativo;
            lasterror_PiDF_Shooter = erro;

            if (Math.abs(erro)<0.05) integral_PiDF_Shooter = 0;
            if (integral_PiDF_Shooter>0.15) integral_PiDF_Shooter=0.15;
            else if (integral_PiDF_Shooter<-0.15) integral_PiDF_Shooter=-0.15;

            double power_correction = (proporcional + integral_PiDF_Shooter + derivativo ) * feedfoward;

            if ( power_correction > 1) power_correction = 1;
            else if (power_correction < -1) power_correction = -1;

            if (power_correction < 0) power_correction = 0;
            if (velocidade_desejada == 0 ) Atirador_1.setPower(0);
            else Atirador_1.setPower( power_correction );
            Atirador_2.setPower(-Atirador_1.getPower());






            double potencia_indexer;
            double erro1 = posicao_desejada_indexer - Indexer.getCurrentPosition();
            // Acumulando o erro para o integral
            double proporcional1 =  erro1 * 0.0045;


            double integrativo1 = 0.0005 * (erro1 * cicle_time);
            integral_Indexer += integrativo1;

            if (Math.abs(erro)<2) integral_Indexer = 0;
            if (integral_Indexer>0.1) integral_Indexer=0.1;
            else if (integral_Indexer<-0.1) integral_Indexer=-0.1;

            potencia_indexer = proporcional1 + integral_Indexer;

            if ( potencia_indexer > 0.4) potencia_indexer = 0.4;
            else if (potencia_indexer < -0.4) potencia_indexer = -0.4;

            Indexer.setPower(potencia_indexer);

            telemetry.addData("Velocidade atirador", Atirador_1.getVelocity());
            telemetry.addData("erro speed", erro);
            telemetry.addData("potencia atirador", power_correction);


            // Atualiza os dados na tela*/
            telemetry.update();

            return PIDs_state;
        }
    }





    public VoltageSensor myControlHubVoltageSensor;
    int posicao_desejada_indexer = 0, reservatorio = 0, reservatorio_1, reservatorio_2, reservatorio_3;

    double power_correction;
    double potencia_indexer, integral_PiDF_Shooter, lasterror_PiDF_Shooter, integral_Indexer;
    double[] media_speed = new double[10];
    private VoltageSensor ControlHub_VoltageSensor;


    //variáveis dos controladores PiDF____________________________
    double integral_PiDF_Correcao_Angular = 0;
    double lasterror_PiDF_Correcao_Angular = 0;

    //____________________________________________________________

    double cicle_time = 0;



    private Limelight3A limelight;

    private DcMotorEx leftBack, leftFront, rightBack, rightFront, Intake, Atirador_1, Atirador_2, Indexer;

    private Servo Reservoir_2, Reservoir_1, Reservoir_3;

    private RevBlinkinLedDriver Led_Strip;

    boolean PIDs_state;

    public int id_ap;

    ElapsedTime  timer_shooter, deltaTime;

    public double velocidade_desejada = 1347, Ai_shoot;

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
        Rotas();
        //------------------------------------------
        waitForStart();
        PIDs_state = false;
        Actions.runBlocking(rota1);

        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            id_ap = fr.getFiducialId();
        }
        if (id_ap == 21) {
            Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }else  if (id_ap == 22) {
            Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

        }else  if (id_ap == 23) {
            Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }else  Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);

        LLResult result2 = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults2 = result2.getFiducialResults();

        for (LLResultTypes.FiducialResult fr : fiducialResults2) {
            id_ap = fr.getFiducialId();
        }
        if (id_ap == 21) {
            Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }else  if (id_ap == 22) {
            Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

        }else  if (id_ap == 23) {
            Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }else  {
            id_ap = 21;
            Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        }



        Actions.runBlocking(rota2);

        velocidade_desejada = 1280;

        PIDs_state = true;
        Actions.runBlocking(new ParallelAction(rota3, new PIDs()));

    }

    public void Rotas() {

        MecanumDrive.PARAMS.axialGain = 7.5;
        MecanumDrive.PARAMS.lateralGain = 40;
        MecanumDrive.PARAMS.headingGain = 36;
        MecanumDrive.PARAMS.maxWheelVel = 50;


        drive = new MecanumDrive(hardwareMap, new Pose2d(40, -55,
                Math.toRadians(90)));

        rota1 = drive.actionBuilder(new Pose2d(40, -55, Math.toRadians(90)))

                .strafeToLinearHeading(new Vector2d(10, -35),Math.toRadians(82))

                .waitSeconds(0.5)

                .afterTime(0, ()->{
                    PIDs_state = false;
                })
                .build();

        rota2 = drive.actionBuilder(new Pose2d(10, -35, Math.toRadians(82)))

                .turnTo( Math.toRadians(163))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })

                .build();

        rota3 = drive.actionBuilder(new Pose2d(10, -35, Math.toRadians(163)))
                .afterTime(0, ()->{
                    velocidade_desejada = 1280;

                })

                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 0;
                    if (id_ap == 22) posicao_desejada_indexer = 218;
                    if (id_ap == 23) posicao_desejada_indexer = 218;

                })

                .waitSeconds(3.4)

                .afterTime(0, ()->{
                    if (id_ap == 21) Reservoir_1.setPosition(1);
                    if (id_ap == 22) Reservoir_2.setPosition(1);
                    if (id_ap == 23) Reservoir_2.setPosition(1);

                    PIDs_state = true;
                })

                .waitSeconds(1.5)

                .afterTime(0, ()->{
                    if (id_ap == 21) Reservoir_1.setPosition(0);
                    if (id_ap == 22) Reservoir_2.setPosition(0);
                    if (id_ap == 23) Reservoir_2.setPosition(0);
                })

                .waitSeconds(1.2)

                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 218;
                    if (id_ap == 22) posicao_desejada_indexer = 0;
                    if (id_ap == 23) posicao_desejada_indexer = 440;

                })

                .waitSeconds(2)

                .afterTime(0, ()->{
                    if (id_ap == 21) Reservoir_2.setPosition(1);
                    if (id_ap == 22) Reservoir_1.setPosition(1);
                    if (id_ap == 23) Reservoir_3.setPosition(1);

                    PIDs_state = true;
                })

                .waitSeconds(1.2)

                .afterTime(0, ()->{
                    if (id_ap == 21) Reservoir_2.setPosition(0);
                    if (id_ap == 22) Reservoir_1.setPosition(0);
                    if (id_ap == 23) Reservoir_3.setPosition(0);
                    velocidade_desejada = 1292;

                })

                .waitSeconds(1.2)

                .afterTime(0, ()->{
                    if (id_ap == 21) posicao_desejada_indexer = 440;
                    if (id_ap == 22) posicao_desejada_indexer = 440;
                    if (id_ap == 23) posicao_desejada_indexer = 0;

                })

                .waitSeconds(2.2)

                .afterTime(0, ()->{
                    if (id_ap == 21) Reservoir_3.setPosition(1);
                    if (id_ap == 22) Reservoir_3.setPosition(1);
                    if (id_ap == 23) Reservoir_1.setPosition(1);

                    PIDs_state = true;
                })

                .waitSeconds(1.2)

                .afterTime(0, ()->{
                    if (id_ap == 21) Reservoir_3.setPosition(0);
                    if (id_ap == 22) Reservoir_3.setPosition(0);
                    if (id_ap == 23) Reservoir_1.setPosition(0);
                    posicao_desejada_indexer = 0;
                })

                .waitSeconds(2)

                .turnTo(Math.toRadians(0))

                .afterTime(0, ()->{
                    PIDs_state = false;
                })

                .build();


    }

    public double PiDF_Ind(double erro, double kp, double ki, double kd, double kf, double min, double max){

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


}
