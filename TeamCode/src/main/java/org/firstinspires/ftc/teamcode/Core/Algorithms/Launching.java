package org.firstinspires.ftc.teamcode.Core.Algorithms;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Parts.Reservoirs;
import org.firstinspires.ftc.teamcode.Core.Parts.Reservoirs.ReservoirsState;

public class Launching {
    Reservoirs reservatorio = new Reservoirs();
    ElapsedTime timer_all = new ElapsedTime();

    boolean shoot_all;
    boolean shoot_sequence;

    public void launch_all(){

        if(timer_all.seconds() > 0 && timer_all.seconds() < 1){
            reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.UP);
        }
        else if (timer_all.seconds() > 1 && timer_all.seconds() < 2) {
            reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.DOWN);
            reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.UP);
            reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.UP);
        }else reservatorio.resetServos();

    }

    public void teleop(Gamepad gamepad){
        if (gamepad.leftBumperWasPressed()){
            reservatorio.resetServos();
            timer_all.reset();
            shoot_all = true;
            shoot_sequence = false;
        }
        else if (gamepad.rightBumperWasPressed()){
            reservatorio.resetServos();
            timer_all.reset();
            shoot_all = false;
            shoot_sequence = true;

        }

        if (shoot_all) launch_all();
        else if (shoot_sequence) launch_sequence();
    }

    public void launch_sequence(){
        if(timer_all.seconds() > 0 && timer_all.seconds() < 1){

        }

    }
}
