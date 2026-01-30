package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous


public class Rotas extends OpMode {

    public DigitalChannel  Alliance_B, Alliance_R, Auto1,Auto2, Auto3, Auto4, LED_S_R,LED_S_G,LED_S_B;
    public LED Led_1_R, Led_1_V, Led_2_R, Led_2_V, Led_3_R, Led_3_V;
    public AnalogInput S1, S2,S3,S4;
    public VoltageSensor myControlHubVoltageSensor;

    double power_correction, tx_offset;
    double potencia_indexer, integral_PiDF_Shooter, lasterror_PiDF_Shooter, integral_Indexer, lasterror_PiDF_Indexer;
    double[] media_speed = new double[10];
    public VoltageSensor ControlHub_VoltageSensor;


    //variáveis dos controladores PiDF____________________________
    double integral_PiDF_Correcao_Angular = 0;
    double lasterror_PiDF_Correcao_Angular = 0;

    //____________________________________________________________

    double cicle_time = 0;



    public Limelight3A limelight;

    public DcMotorEx leftBack, leftFront, rightBack, rightFront, Intake, Atirador_1, Atirador_2, Indexer;

    public Servo Reservoir_2, Reservoir_1, Reservoir_3;

    public RevBlinkinLedDriver Led_Strip;

    boolean PIDs_state;

    public int id_ap;

    ElapsedTime timer_shooter, deltaTime;



    public void Azul_12_AbrindoGate() {}

    public void  Azul_9_AbrindoGate() {}

//------------------------------------------------------
    public void Rotas_VPequeno_Perto() {}

    public void Rotas_VPequeno_Longe() {}

    public void Rotas_VGrande_Fora() {}

    public void Rotas_VGrande_6() {}


    @Override
    public void init() {

    }
    @Override
    public void loop() {

    }
}
