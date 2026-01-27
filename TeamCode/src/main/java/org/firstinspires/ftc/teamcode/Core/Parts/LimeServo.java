package org.firstinspires.ftc.teamcode.Core.Parts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.Algorithms.InterfaceSimple;

public class LimeServo implements InterfaceSimple {
public Servo ServoLime;

    public void start(HardwareMap hardware){
        ServoLime = hardware.get(Servo.class, "ServoLime");
    }

    public void setPosition(double position) {
        ServoLime.setPosition(position);
    }


    @Override
    public void update() {


    }
}
