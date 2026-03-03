package org.firstinspires.ftc.teamcode.Core.Hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class Leds {

    public RevBlinkinLedDriver Led_Strip;
    public LED Led_1_R, Led_1_G, Led_2_R, Led_2_G, Led_3_R, Led_3_G;
    ColorSensor.BALL bola1, bola2, bola3;

    public enum ledColor{
        RED,
        GREEN,

        ORANGE,
        OFF,

    }


    public void start(HardwareMap hardware){
        Led_Strip = hardware.get(RevBlinkinLedDriver.class, "Led_Strip");

       Led_1_R = hardware.get(LED.class, "Led_1_R");
       Led_1_G = hardware.get(LED.class, "Led_1_G");
       Led_2_R = hardware.get(LED.class, "Led_2_R");
       Led_2_G = hardware.get(LED.class, "Led_2_G");
       Led_3_R = hardware.get(LED.class, "Led_3_R");
       Led_3_G = hardware.get(LED.class, "Led_3_G");

    }


    public void setBallsState(ColorSensor.BALL b1, ColorSensor.BALL b2, ColorSensor.BALL b3){
        bola1 = b1;
        bola2 = b2;
        bola3 = b3;

        switch (bola1){
            case GREEN:
                indicadorLed(ledColor.GREEN, 1);
                break;
            case PURPLE:
                indicadorLed(ledColor.RED, 1);
                break;
            case NONE:
                indicadorLed(ledColor.OFF, 1);
                break;
        }

        switch (bola2){
            case GREEN:
                indicadorLed(ledColor.GREEN, 2);
                break;
            case PURPLE:
                indicadorLed(ledColor.RED, 2);
                break;
            case NONE:
                indicadorLed(ledColor.OFF, 2);
                break;
        }

        switch (bola3){
            case GREEN:
                indicadorLed(ledColor.GREEN, 3);
                break;
            case PURPLE:
                indicadorLed(ledColor.RED, 3);
                break;
            case NONE:
                indicadorLed(ledColor.OFF, 3);
                break;
        }
    }
    public void indicadorLed(ledColor cor, int LED){
        switch(cor){
            case RED:
                if (LED == 1){
                    Led_1_R.on();
                    Led_1_G.off();
                } else if (LED == 2) {
                    Led_2_R.on();
                    Led_2_G.off();
                } else if (LED == 3) {
                    Led_3_R.on();
                    Led_3_G.off();
                }
                break;
            case GREEN:
                if (LED == 1){
                    Led_1_R.off();
                    Led_1_G.on();
                } else if (LED == 2) {
                    Led_2_R.off();
                    Led_2_G.on();
                } else if (LED == 3) {
                    Led_3_R.off();
                    Led_3_G.on();
                }
                break;
            case ORANGE:
                if (LED == 1){
                    Led_1_R.on();
                    Led_1_G.on();
                } else if (LED == 2) {
                    Led_2_R.on();
                    Led_2_G.on();
                } else if (LED == 3) {
                    Led_3_R.on();
                    Led_3_G.on();
                }
                break;
            case OFF:
                if (LED == 1){
                    Led_1_R.off();
                    Led_1_G.off();
                } else if (LED == 2) {
                    Led_2_R.off();
                    Led_2_G.off();
                } else if (LED == 3) {
                    Led_3_R.off();
                    Led_3_G.off();
                }
                break;

        }
    }

}
