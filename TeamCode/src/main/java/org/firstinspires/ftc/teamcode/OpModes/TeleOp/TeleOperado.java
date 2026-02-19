package org.firstinspires.ftc.teamcode.OpModes.TeleOp;





import org.firstinspires.ftc.teamcode.Core.Algorithms.Launching;
import org.firstinspires.ftc.teamcode.Core.Algorithms.PoseIdentifier;
import org.firstinspires.ftc.teamcode.Core.Algorithms.ShooterControl;
import org.firstinspires.ftc.teamcode.Core.Algorithms.TagAlign;
import org.firstinspires.ftc.teamcode.Core.Hardware.AprilTagCam;
import org.firstinspires.ftc.teamcode.Core.Hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Core.Hardware.Leds;
import org.firstinspires.ftc.teamcode.Core.Hardware.Lime;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Parts.AimAssisters;
import org.firstinspires.ftc.teamcode.Core.Parts.Drivetrain;
import org.firstinspires.ftc.teamcode.Core.Parts.LimeServo;
import org.firstinspires.ftc.teamcode.Core.Parts.Reservoirs;
import org.firstinspires.ftc.teamcode.Core.Parts.Shooter;
import org.firstinspires.ftc.teamcode.Core.Parts.Intake;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.OpModes.Autos.Autonomo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Core.Algorithms.Drawing;


@Configurable

@TeleOp
public class TeleOperado extends Autonomo {


    Follower followerteleop;

    boolean isGamepad1 = false, isManual = false, isValidArea = false;
    Robot robot = new Robot();

    Drivetrain drivetrain = robot.drivetrain;
    TelemetryManager telemetryM;


    Launching launching = new Launching();
    public static double kpShooter = 0.0017, kiShooter = 0.000125, kdShooter = 0, kfShooter = 0.05;
    public static double kpAlign = 0.02, kiAlign = 0.00005, kdAlign = 0, kfAlign = 0;


    public static double Firstkp = 0.9, Firstki = 1, Firstkd = 0.05, Firstkf = 0;
    public static double Secondkp = 0.23, Secondki= 0, Secondkd = 0.037, Secondkf = 0;

    ElapsedTime Ciclo;

     double tempoCiclo;



    public  VoltageSensor myControlHubVoltageSensor;

     Shooter Shooter = robot.shooter;

     Intake Intake = new Intake();

     Lime Limelight = robot.limelight;

   // AprilTagCam aptag = robot.aptag;

    TagAlign tagalign = new TagAlign();
    AimAssisters Capota = launching.Capota;
    public Reservoirs reservatorio = new Reservoirs();

   // ColorSensor colorSensor = new ColorSensor();


    PoseIdentifier poseidentifier = new PoseIdentifier();

    LimeServo servolime = robot.servoLime;

    Leds LED = new Leds();

    ShooterControl ControleShooter = new ShooterControl();



    public static double servoA = 0.009, servoB = 1.55;

    public static double velA = 3.35;
    public static int velB = 850;

    public static double offset = 0.0000001;


 public double servoIdeal;

 public int  velIdeal;

 public static double timer = 0.2;

 public static double  corr;

public double ErroMovY, ErroMovX,last_pose_X, last_pose_Y;
 public boolean isitBlue, manualSelectAlliance, manualSelectAuto, Movimento_robo;
    double angleIdeal;



    @Override
    public void init() {
        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        followerteleop = Constants.createFollower(hardwareMap);


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

      //  colorSensor.start(hardwareMap);

        LED.start(hardwareMap);

        reservatorio.start(hardwareMap);

        Limelight.setup(hardwareMap);

        Intake.start(hardwareMap);

        Shooter.start(hardwareMap);


        launching.start(hardwareMap);

        Capota.start(hardwareMap);
        Capota.setMode(AimAssisters.Mode.SET_POSITION);

        servolime.start(hardwareMap);








        Drawing.init();



    }

@Override
public void init_loop(){
    telemetry.addData("auto", auto);
    telemetry.addData("isBlue", isBlue);
    telemetry.update();
}


@Override
    public void start(){
        Ciclo = new ElapsedTime();
       Ciclo.reset();

    followerteleop.setPose(finalPose);

    followerteleop.update();
    followerteleop.startTeleopDrive();

    poseidentifier.isblue(isBlue);
    ControleShooter.isBlue(isBlue);
    drivetrain.setCoefficientsAptag(kpAlign, kiAlign, kdAlign, kfAlign);

    }

    @Override
    public void loop() {

        Shooter.setCoefficients(kpShooter,kiShooter, kdShooter, kfShooter);
        drivetrain.setCoefficientsfirt(Firstkp, Firstki, Firstkd, Firstkf);
        drivetrain.setCoefficientssecond(Secondkp, Secondki, Secondkd, Secondkf);
       // colorSensor.setCor(ColorSensor.COLOR.NONE);


        Ciclo.reset();


        tagalign.setValues(Limelight.tx, Limelight.tagdetected, 1280);




        servolime.setCurrentPosition(poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.X), poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.Y), poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.YAW));


        if (gamepad1.dpadUpWasPressed()) {
            servolime.updateDesiredReference(LimeServo.Mode.BLUE_TAG);
        }

        if(gamepad1.dpadDownWasPressed()){
            servolime.updateDesiredReference(LimeServo.Mode.BLUE_GATE);
        }




        double d = poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.DISTANCE);

        ControleShooter.setDistance(d);

        ControleShooter.setServo(servoA, servoB);

        ControleShooter.setVel(velA,velB);

        velIdeal = ControleShooter.getVel();

        servoIdeal = ControleShooter.getServo();

        if (isBlue) {
            angleIdeal = Math.atan2(poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.Y) - 138, poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.X) - 12);
        }else{
            angleIdeal = Math.atan2(poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.Y) - 136, poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.X) - 132);
        }


        if (poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.Y) < Math.abs(poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.X) - 72) + 68.5
                ||
                poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.Y) < -Math.abs(poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.X) + 72) + 35.5) {
            isValidArea = true;
        } else {
            isValidArea = false;
        }




//===CONTROLE DE POSIÇÃO=========================================================================
//===FUNÇÕES VITAIS==============================================================================
        {
            launching.teleop(gamepad2,timer);

            Shooter.teleop(gamepad2, velIdeal, isGamepad1, velIdeal);

            Intake.teleop(gamepad1);


            ControleShooter.update();
        }

//===LANÇAMENTO==================================================================================
        {
            if (!Shooter.shooting) {
               reservatorio.setPosition(reservatorio.Reservoir_1, 0.48);
               reservatorio.setPosition(reservatorio.Reservoir_2, 0.5);
               reservatorio.setPosition(reservatorio.Reservoir_3, 0.5);
            }



            Shooter.isServoLifted(launching.servo_lifted);
            launching.isAtRightVel(Shooter.right_vel);
            launching.isShooting(Shooter.shooting);


            if (gamepad1.cross || gamepad1.a || gamepad1.circle || gamepad1.b){
                Capota.setValue(0.5);
            }else {
                Capota.setValue(servoIdeal);
            }
            launching.isAtRightPos(drivetrain.right_pos);
        }

//===FUNÇÕES DE MOVIMENTO========================================================================
        {
            if (isGamepad1) {
                if (isManual) drivetrain.Drive(true,isBlue, gamepad1, followerteleop, Shooter.shooting, 0, 100, false, offset, angleIdeal, launching.corretor);
                else drivetrain.Drive(true,isBlue, gamepad1, followerteleop, Shooter.shooting, tagalign.pixel, tagalign.width, false, offset, angleIdeal,launching.corretor);

            } else {
                if (isManual) drivetrain.Drive(false,isBlue, gamepad2, followerteleop, Shooter.shooting, 0, 100, false, offset, angleIdeal, launching.corretor);
                else drivetrain.Drive(false,isBlue, gamepad2, followerteleop, Shooter.shooting, tagalign.pixel, tagalign.width, false, offset, angleIdeal,launching.corretor);

            }
        }

//===FUNÇÕES DE CONTROLE=========================================================================
        {
            if (gamepad2.backWasPressed()) isManual = !isManual;


            if (gamepad1.square || gamepad1.x) {
                isGamepad1 = true;
            }

            else if (gamepad2.square || gamepad2.x){
                isGamepad1 = false;
            }



        }

//===UPDATE DE POSIÇÃO===========================================================================
        {

        if (gamepad1.startWasPressed()){
            if (isBlue){
            followerteleop.setPose(new Pose(134,8, Math.toRadians(180)));
                drivetrain.setTargetHeading(Math.toRadians(180));
            }else {
                followerteleop.setPose(new Pose(9,8, Math.toRadians(0)));
                drivetrain.setTargetHeading(Math.toRadians(0));
            }
        }


            poseidentifier.infoLIME(Limelight.x, Limelight.y, Limelight.angle, Limelight.distance);
           // poseidentifier.infoCAM(aptag.LeituraX, aptag.LeituraY, aptag.LeituraYaw);
            poseidentifier.infoPINPOINT(followerteleop.getPose().getX(), followerteleop.getPose().getY(), followerteleop.getPose().getHeading());

        }


//===LEDS========================================================================================
        {


            if(drivetrain.aptag) LED.Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            else  LED.Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        robot.update();
        if (gamepad2.a) servolime.setPosition(0.5);
        telemetry();
        tempoCiclo = Ciclo.milliseconds();
        Drawing.draw(followerteleop);
    }


    public void stop(){
      //  aptag.stop();
    }

    public void telemetry(){


        telemetry.addData("auto", auto);
        telemetry.addData("isBlue", isBlue);
        telemetry.addLine();
        telemetry.addData("FinalPose/StartPose", finalPose);

        telemetry.addLine();
        telemetry.addData("ang ideal", servolime.IdealYaw);
        telemetry.addData("current yaw", servolime.currentYaw);
        telemetry.addData("limeservo", servolime.ServoLime.getPosition());
        telemetry.addLine();
        //telemetry.addData("S1", colorSensor.getLeitura(colorSensor.S1));
        //telemetry.addData("S2", colorSensor.getLeitura(colorSensor.S2));
        //telemetry.addData("S3", colorSensor.getLeitura(colorSensor.S3));
        //telemetry.addData("S4", colorSensor.getLeitura(colorSensor.S4));
        telemetry.addLine();




        telemetry.addLine();

        telemetry.addLine();
        telemetry.addData("Capota " , Capota.getPosition());
        telemetry.addLine();
        telemetry.addLine("POSIÇÕES");
        telemetry.addData("LimeX", poseidentifier.getInfo(PoseIdentifier.Source.LIMELIGHT, PoseIdentifier.DataInfo.X));
        telemetry.addData("LimeY",  poseidentifier.getInfo(PoseIdentifier.Source.LIMELIGHT, PoseIdentifier.DataInfo.Y));
        telemetry.addData("LimeYaw",  poseidentifier.getInfo(PoseIdentifier.Source.LIMELIGHT, PoseIdentifier.DataInfo.YAW));
        telemetry.addData("LimeDist",  poseidentifier.getInfo(PoseIdentifier.Source.LIMELIGHT, PoseIdentifier.DataInfo.DISTANCE));
        telemetry.addLine();
        telemetry.addData("PinX", poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.X));
        telemetry.addData("PinY",  poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.Y));
        telemetry.addData("PinYaw",  poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.YAW));
        telemetry.addData("PinDist",  poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.DISTANCE));
        telemetry.addLine();

        telemetry.addData("ErrorX", poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.X) - poseidentifier.getInfo(PoseIdentifier.Source.LIMELIGHT, PoseIdentifier.DataInfo.X));
        telemetry.addData("ErrorY",  poseidentifier.getInfo(PoseIdentifier.Source.CAMERA, PoseIdentifier.DataInfo.Y) - poseidentifier.getInfo(PoseIdentifier.Source.LIMELIGHT, PoseIdentifier.DataInfo.Y));
        telemetry.addData("ErrorYaw",  poseidentifier.getInfo(PoseIdentifier.Source.CAMERA, PoseIdentifier.DataInfo.YAW) - poseidentifier.getInfo(PoseIdentifier.Source.LIMELIGHT, PoseIdentifier.DataInfo.YAW));
        telemetry.addData("ErrorDist",  poseidentifier.getInfo(PoseIdentifier.Source.CAMERA, PoseIdentifier.DataInfo.DISTANCE) - poseidentifier.getInfo(PoseIdentifier.Source.LIMELIGHT, PoseIdentifier.DataInfo.DISTANCE));
        telemetry.addLine();

        telemetry.addLine("ATIRADOR");


        telemetryM.addData("Velocidade desejada" , Shooter.velocidade_desejada);
        telemetryM.addData("Velocidade atual" , Shooter.Atirador_1.getVelocity());
        telemetryM.addData("erro shooter " , Shooter.erro);
        telemetryM.addData("power " , Shooter.Atirador_1.getPower());

        telemetry.addData("Velocidade desejada" , Shooter.velocidade_desejada);
        telemetry.addData("Velocidade atual" , Shooter.Atirador_1.getVelocity());
        telemetry.addData("shooter erro" , Shooter.erro);
        telemetry.addData("power " , Shooter.Atirador_1.getPower());

        telemetry.addLine();
        telemetryM.addData("Target Heading", drivetrain.targetHeading);
        telemetryM.addData("Heading", followerteleop.getHeading());
        telemetryM.addData("erooororreading", drivetrain.error_final);
        telemetry.addLine();

        telemetry.addData("Target Heading", drivetrain.targetHeading);
        telemetry.addData("Heading", followerteleop.getHeading());
        telemetry.addData("erooororreading", drivetrain.error_final);
        telemetry.addLine();



        telemetry.addData("ID", Limelight.id);
        telemetry.addData("TagPixels LIME", Limelight.tpixel);
        telemetry.addData("Balls", Limelight.TotalBalls);
        telemetry.addData("Pipeline", Limelight.pipeline);
        telemetry.addData("ModeLime", Limelight.OperationMode);


        telemetry.addLine();

        telemetry.addData("Heading error", followerteleop.getHeadingError());
        telemetry.addData("Heading Vector", followerteleop.getTeleopHeadingVector());
        telemetry.addLine();

        telemetry.addLine();


        telemetryM.update(telemetry);
        telemetry.update();

}
}
