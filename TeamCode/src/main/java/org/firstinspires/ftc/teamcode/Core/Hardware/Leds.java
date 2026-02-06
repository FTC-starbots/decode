package org.firstinspires.ftc.teamcode.Core.Hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class Leds {

    public RevBlinkinLedDriver Led_Strip;
    public LED Led_1_R, Led_1_V, Led_2_R, Led_2_V, Led_3_R, Led_3_V;


    public void start(HardwareMap hardware){
        Led_Strip = hardware.get(RevBlinkinLedDriver.class, "Led_Strip");

//        Led_1_R = hardware.get(LED.class, "Led_1_R");
//        Led_1_V = hardware.get(LED.class, "Led_1_V");
//        Led_2_R = hardware.get(LED.class, "Led_2_R");
//        Led_2_V = hardware.get(LED.class, "Led_2_V");
//        Led_3_R = hardware.get(LED.class, "Led_3_R");
//        Led_3_V = hardware.get(LED.class, "Led_3_V");

    }



    public void indicadorLed(){

    }

    public void blinkinLed(){

    }

}
