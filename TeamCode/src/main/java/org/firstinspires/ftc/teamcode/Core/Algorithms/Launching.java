package org.firstinspires.ftc.teamcode.Core.Algorithms;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Parts.Reservoirs;
import org.firstinspires.ftc.teamcode.Core.Parts.Reservoirs.ReservoirsState;

public class Launching {
    public Reservoirs reservatorio = new Reservoirs();
    ElapsedTime timer_all = new ElapsedTime();
    ElapsedTime pressed_button = new ElapsedTime();

    boolean shoot_all;
    boolean shoot_sequence;
    public void start(HardwareMap hardware){

        reservatorio.start(hardware);
    }

    public void launch_all(){
        reservatorio.resetServos();
        if(timer_all.seconds() > 0.2 && timer_all.seconds() < 1.5){
            reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.UP);
            reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.UP);
            reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.UP);
        }
        else if (timer_all.seconds() > 1.5 && timer_all.seconds() < 2.8) {
            reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.DOWN);
            reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.DOWN);
            reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.DOWN);
            shoot_all = false;
        }


    }

    public void teleop(Gamepad gamepad){
        if (gamepad.leftBumperWasPressed()){

            reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.UP);
            reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.UP);
            reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.UP);
            /*pressed_button.reset();
            reservatorio.resetServos();
            timer_all.reset();
            shoot_all = true;
            shoot_sequence = false;*/
        }
        else if (gamepad.rightBumperWasPressed()){

            reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.DOWN);
            reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.DOWN);
            reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.DOWN);
            /*pressed_button.reset();
            reservatorio.resetServos();
            timer_all.reset();
            shoot_all = false;
            shoot_sequence = true;*/

        }

        if (pressed_button.seconds() > 3) {
           /* shoot_all = false;
            shoot_sequence = false;
            reservatorio.resetServos();*/
        }

        if (shoot_all) launch_all();
        else if (shoot_sequence) launch_sequence();
    }

    public void launch_sequence(){
        if(timer_all.seconds() > 0 && timer_all.seconds() < 1){

        }

    }
}
