package org.firstinspires.ftc.teamcode.Core.Parts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Reservoirs {
    public Servo Reservoir_2, Reservoir_1, Reservoir_3;
    public enum ReservoirsState {
        UP,
        DOWN

    }

    public void start(HardwareMap hardware) {
        Reservoir_1 = hardware.get(Servo.class, "Reservoir_1");

        Reservoir_2 = hardware.get(Servo.class, "Reservoir_2");

        Reservoir_3 = hardware.get(Servo.class, "Reservoir_3");
    }

    public void resetServos() {
        Reservoir_1.setPosition(0.5);
        Reservoir_2.setPosition(0.5);
        Reservoir_3.setPosition(0.5);
    }

    public void setPosition(Servo Reservoir, double position) {
        Reservoir.setPosition(position);
    }

    public void servoPosition(Servo Reservoir, ReservoirsState state) {
        switch (state) {
            case UP:
                 Reservoir.setPosition(1);
                break;

            case DOWN:
                if(Reservoir == Reservoir_1)Reservoir.setPosition(0.48);
                else Reservoir.setPosition(0.5);
                break;


        }
    }

}
