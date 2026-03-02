package org.firstinspires.ftc.teamcode.Core.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ColorSensor {

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
    public void isPosReading(boolean is){post_reading = is;}
    public void readPresence(ElapsedTime timer){


            setCor(COLOR.WHITE);

            leituraPresece1 = getLeitura(S1);

            leituraPresece2 = getLeitura(S2);

            leituraPresece3 = getLeitura(S3);


            if (1.47 > leituraPresece1) color1 = BALL.PURPLE;

            if (1.47 > leituraPresece2) color2 = BALL.PURPLE;

            if (1.47 > leituraPresece3) color3 = BALL.PURPLE;

            ball1 = color1;
            ball2 = color2;
            ball3 = color3;

    }
    public void readColor(ElapsedTime timer) {

            if (timer.seconds() < 0.05){
                setCor(COLOR.GREEN);
            } else if (timer.seconds() > 0.1 && timer.seconds() < 0.3){
                 leituraVerde1 = getLeitura(S1);

                 leituraVerde2 = getLeitura(S2);

                 leituraVerde3 = getLeitura(S3);

            } else if (timer.seconds() > 0.3 && timer.seconds() < 0.35) {
                setCor(COLOR.PURPLE);
            }else if (timer.seconds() > 0.4 && timer.seconds() < 0.6){

                 leituraRoxo1 = getLeitura(S1);

                 leituraRoxo2 = getLeitura(S2);

                 leituraRoxo3 = getLeitura(S3);

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
