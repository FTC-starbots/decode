package org.firstinspires.ftc.teamcode.Core.Hardware;

import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ColorSensor {
    KalmanFilter RES1 = new KalmanFilter(new KalmanFilterParameters(0.05,1), 2, 100,1.2);
    KalmanFilter RES2 = new KalmanFilter(new KalmanFilterParameters(0.05,1), 2, 100,1.2);
    KalmanFilter RES3 = new KalmanFilter(new KalmanFilterParameters(0.05,1), 2, 100,1.2);

    KalmanFilter G1 = new KalmanFilter(new KalmanFilterParameters(0.05,1), 2, 100,1.2);
    KalmanFilter G2 = new KalmanFilter(new KalmanFilterParameters(0.05,1), 2, 100,1.2);
    KalmanFilter G3 = new KalmanFilter(new KalmanFilterParameters(0.05,1), 2, 100,1.2);
    public enum COLOR{
        RED,
        GREEN,
        BLUE,
        PURPLE,
        WHITE,
        NONE

    }

    public enum BALL{
        GREEN,
        PURPLE,
        NONE,
        ERROR

    }
    public DigitalChannel S_LED_R, S_LED_G, S_LED_B;
    public AnalogInput S1, S2, S3, S4;

public boolean post_reading;
int presRead;

    BALL color1= BALL.NONE, color2= BALL.NONE, color3= BALL.NONE;
    public BALL ball1 = BALL.NONE, ball2= BALL.NONE, ball3= BALL.NONE;

    public double leituraVerde1, leituraRoxo1, leituraVerde2, leituraRoxo2, leituraVerde3, leituraRoxo3, leituraPresece1, leituraPresece2, leituraPresece3;


    public void start(HardwareMap hardware){
        S_LED_R = hardware.get(DigitalChannel.class, "S_LED_R");
        S_LED_G = hardware.get(DigitalChannel.class, "S_LED_G");
        S_LED_B = hardware.get(DigitalChannel.class, "S_LED_B");
        S1 = hardware.get(AnalogInput.class, "S4");
        S2 = hardware.get(AnalogInput.class, "S1");
        S3 = hardware.get(AnalogInput.class, "S2");
        S4 = hardware.get(AnalogInput.class, "S3");


        S_LED_R.setMode(DigitalChannel.Mode.OUTPUT);
        S_LED_G.setMode(DigitalChannel.Mode.OUTPUT);
        S_LED_B.setMode(DigitalChannel.Mode.OUTPUT);


    }

    public void setCor(COLOR cor) {
        switch (cor) {
            case RED:
                S_LED_R.setState(true);
                S_LED_G.setState(false);
                S_LED_B.setState(false);
             break;
            case BLUE:
                S_LED_R.setState(false);
                S_LED_G.setState(false);
                S_LED_B.setState(true);
                break;
            case GREEN:
                S_LED_R.setState(false);
                S_LED_G.setState(true);
                S_LED_B.setState(false);
                break;
            case PURPLE:
                S_LED_R.setState(true);
                S_LED_G.setState(false);
                S_LED_B.setState(true);
                break;
            case WHITE:
                S_LED_R.setState(true);
                S_LED_G.setState(true);
                S_LED_B.setState(true);
                break;
            case NONE:
                S_LED_R.setState(false);
                S_LED_G.setState(false);
                S_LED_B.setState(false);
                break;


        }
    }

    public double getLeitura(AnalogInput sensor){
        if (sensor == S1) return S1.getVoltage();
        else if (sensor == S2) return S2.getVoltage();
        else if (sensor == S3) return S3.getVoltage();
        else if (sensor == S4) return S4.getVoltage();
        else return 0;

    }


    public void resetBalls(){
        ball1 = BALL.NONE;
        ball2 = BALL.NONE;
        ball3 = BALL.NONE;
    }
    public void resetReservoir(int res){
        switch (res){
            case 1:
                ball1 = BALL.NONE;
                break;
            case 2:
                ball2 = BALL.NONE;
                break;
            case 3:
                ball3 = BALL.NONE;
                break;
        }
    }
    public void isPosReading(boolean is){post_reading = is;}

    public void resetKalman(){
        RES1.reset(2,100,1.2);
        RES2.reset(2,100,1.2);
        RES3.reset(2,100,1.2);
        RES1.reset(1.5,100,1.2);
        RES2.reset(1.5,30,1.2);
        RES3.reset(1.5,30,1.2);
        leituraPresece1 = 2;
        leituraPresece2 = 2;
        leituraPresece3 = 2;
        leituraRoxo1 = 2;
        leituraRoxo2 = 2;
        leituraRoxo3 = 2;
        leituraVerde1 = 2;
        leituraVerde2 = 2;
        leituraVerde3 = 2;

    }
    public void resetPres(){presRead = 0;}
    public void readPresence(ElapsedTime timer){


                setCor(COLOR.WHITE);

                RES1.update(0, getLeitura(S1));
                leituraPresece1 = RES1.getState();

                RES2.update(0, getLeitura(S2));
                leituraPresece2 = RES2.getState();

                RES3.update(0, getLeitura(S3));
                leituraPresece3 = RES3.getState();


                if (1.45 > leituraPresece1) color1 = BALL.PURPLE;
                else color1 = BALL.NONE;

                if (1.51 > leituraPresece2) color2 = BALL.PURPLE;
                else color2 = BALL.NONE;

                if (1.55 > leituraPresece3) color3 = BALL.PURPLE;
                else color3 = BALL.NONE;

                ball1 = color1;
                ball2 = color2;
                ball3 = color3;

    }
    public void readColor(ElapsedTime timer) {

            if (timer.seconds() < 0.05){
                setCor(COLOR.GREEN);
            } else if (timer.seconds() > 0.1 && timer.seconds() < 0.3){
                G1.update(0, getLeitura(S1));
                leituraVerde1 = G1.getState();

                G2.update(0, getLeitura(S2));
                leituraVerde2 = G2.getState();

                G3.update(0, getLeitura(S3));
                leituraVerde3 = G3.getState();


            } else if (timer.seconds() > 0.3 && timer.seconds() < 0.35) {
                setCor(COLOR.PURPLE);
            }else if (timer.seconds() > 0.4 && timer.seconds() < 0.6){

                RES1.update(0, getLeitura(S1));
                leituraRoxo1 = RES1.getState();

                RES2.update(0, getLeitura(S2));
                leituraRoxo2 = RES2.getState();

                RES3.update(0, getLeitura(S3));
                leituraRoxo3 = RES3.getState();


            }else if (timer.seconds() > 0.6 && timer.seconds() < 1){
                if (ball1 != BALL.NONE) {
                    if (leituraVerde1 < 1.5) color1 = BALL.GREEN;

                    if (1.3 > leituraRoxo1) color1 = BALL.PURPLE;
                }

                if (ball2 != BALL.NONE) {
                    if (leituraVerde2 < leituraRoxo2) color2 = BALL.GREEN;
                    else if (leituraVerde2 > leituraRoxo2) color2 = BALL.PURPLE;
                }

                if (ball1 != BALL.NONE) {
                    if (leituraVerde3 < leituraRoxo3) color3 = BALL.GREEN;
                    else if (leituraVerde3 > leituraRoxo3) color3 = BALL.PURPLE;
                }
                ball1 = color1;
                ball2 = color2;
                ball3 = color3;
                post_reading = true;
            }
            
            
    }
}
