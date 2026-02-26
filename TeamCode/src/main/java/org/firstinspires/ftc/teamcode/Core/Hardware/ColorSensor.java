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

    ElapsedTime timer = new ElapsedTime();

    BALL color;
    public BALL ball1, ball2, ball3;

    double leituraVerde1, leituraRoxo1, leituraVerde2, leituraRoxo2, leituraVerde3, leituraRoxo3;


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

    public BALL getBall(AnalogInput sensor) {
        if (sensor == S1) {
            timer.reset();
            if (timer.seconds() < 0.05){
                setCor(COLOR.GREEN);
            } else if (timer.seconds() > 0.05 && timer.seconds() < 0.2){
                leituraVerde1 = getLeitura(S1);
            } else if (timer.seconds() > 0.2 && timer.seconds() < 0.25) {
                setCor(COLOR.PURPLE);
            }else if (timer.seconds() > 0.25 && timer.seconds() < 0.4){
                leituraRoxo1 = getLeitura(S1);
            }else if (timer.seconds() > 0.4 && timer.seconds() < 0.5){
               if (leituraVerde1 > 0.5){
                   color = BALL.GREEN;
               }else if(0.5 < leituraRoxo1){
                   color = BALL.PURPLE;
               }else{
                   color = BALL.NONE;
               }
            }


            ball1 = color;
        }else if (sensor == S2) {
            timer.reset();
            if (timer.seconds() < 0.05){
                setCor(COLOR.GREEN);
            } else if (timer.seconds() > 0.05 && timer.seconds() < 0.2){
                leituraVerde2 = getLeitura(S2);
            } else if (timer.seconds() > 0.2 && timer.seconds() < 0.25) {
                setCor(COLOR.PURPLE);
            }else if (timer.seconds() > 0.25 && timer.seconds() < 0.4){
                leituraRoxo2 = getLeitura(S2);
            }else if (timer.seconds() > 0.4 && timer.seconds() < 0.5){
                if (leituraVerde2 > 0.5){
                    color = BALL.GREEN;
                }else if(0.5 < leituraRoxo2){
                    color = BALL.PURPLE;
                }else{
                    color = BALL.NONE;
                }
            }


            ball2 = color;
        }else if (sensor == S3) {
            timer.reset();
            if (timer.seconds() < 0.05){
                setCor(COLOR.GREEN);
            } else if (timer.seconds() > 0.05 && timer.seconds() < 0.2){
                leituraVerde3 = getLeitura(S3);
            } else if (timer.seconds() > 0.2 && timer.seconds() < 0.25) {
                setCor(COLOR.PURPLE);
            }else if (timer.seconds() > 0.25 && timer.seconds() < 0.4){
                leituraRoxo3 = getLeitura(S3);
            }else if (timer.seconds() > 0.4 && timer.seconds() < 0.5){
                if (leituraVerde3 > 0.5){
                    color = BALL.GREEN;
                }else if(0.5 < leituraRoxo3){
                    color = BALL.PURPLE;
                }else{
                    color = BALL.NONE;
                }
            }


        ball3 = color;
        } else color = BALL.ERROR;

        return color;
    }



}
