package org.firstinspires.ftc.teamcode.Autos;

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

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;

@Config
@Autonomous


public class Rotas extends LinearOpMode {

    public VoltageSensor myControlHubVoltageSensor;
    int posicao_desejada_indexer = 0, reservatorio = 0, reservatorio_1, reservatorio_2, reservatorio_3;

    double power_correction;
    double potencia_indexer, integral_PiDF_Shooter, lasterror_PiDF_Shooter, integral_Indexer, lasterror_PiDF_Indexer;
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
/*
    public Action rota1;
    public Action rota2;
    public Action rota3;
    public Action rota4;
    public Action rota5;
    public Action rota6;
    public Action rota7;
    public Action rota8;
    public Action rota9;
*/

//    public Action rota;
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    //public Action Rotas_APequeno() {

//return rota;

    //}



}
