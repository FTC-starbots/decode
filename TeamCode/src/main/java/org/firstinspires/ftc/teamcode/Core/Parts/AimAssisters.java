package org.firstinspires.ftc.teamcode.Core.Parts;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.Algorithms.InterfaceSimple;

public class AimAssisters implements InterfaceSimple {
    public Servo Capota1, Capota2;

    boolean Propocional = false;

    public enum Mode {
        PROPORTIONAL,
        SET_POSITION
    }

    public enum Height {
        RESETED,
        LOW,
        MEDIUM,
        HIGH
    }

    public void start(HardwareMap hardware){
        Capota1 = hardware.get(Servo.class, "Capota1");
        Capota2 = hardware.get(Servo.class, "Capota2");
    }
    public void setMode(Mode modo) {
        switch (modo) {
            case PROPORTIONAL:
                Propocional = true;
                break;
            case SET_POSITION:
                Propocional = false;
                break;
        }
    }

    public void setHeight(Height altura) {
        switch (altura) {
            case RESETED:
                Capota1.setPosition(1);
                Capota2.setPosition(1);
                break;
                case LOW:
                Capota1.setPosition(0.66);
                Capota2.setPosition(0.66);
                break;
                case MEDIUM:
                Capota1.setPosition(0.33);
                Capota2.setPosition(0.33);
                break;
                case HIGH:
                Capota1.setPosition(0);
                Capota2.setPosition(0);
                break;

        }
    }

    public void setValue(double value) {
        Capota1.setPosition(value);
        Capota2.setPosition(value);
    }

        public void capota_propor () {
            Capota1.setPosition(0);
            Capota2.setPosition(0);
        }


        @Override
        public void update () {
            if (Propocional) capota_propor();

        }
}
