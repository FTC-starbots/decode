package org.firstinspires.ftc.teamcode.Core.Algorithms;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Parts.AimAssisters;
import org.firstinspires.ftc.teamcode.Core.Parts.Reservoirs;
import org.firstinspires.ftc.teamcode.Core.Parts.Reservoirs.ReservoirsState;

public class Launching {
    public Reservoirs reservatorio = new Reservoirs();
    ElapsedTime timer_all = new ElapsedTime();
    ElapsedTime pressed_button = new ElapsedTime();
   public AimAssisters Capota = new AimAssisters();

    boolean shoot_all;
    boolean shoot_sequence;
   public boolean shoot_requested;
    boolean post_shoot;
    public void start(HardwareMap hardware){
        Capota.start(hardware);
        reservatorio.start(hardware);

    }



    public void teleop(Gamepad gamepad){
        if (gamepad.leftBumperWasPressed()){


            pressed_button.reset();
            timer_all.reset();
            shoot_all = true;
            shoot_sequence = false;
            shoot_requested = true;
        }
        else if (gamepad.rightBumperWasPressed()){


            pressed_button.reset();
            timer_all.reset();
            shoot_all = false;
            shoot_sequence = true;
            shoot_requested = true;
        }


        if (pressed_button.seconds() > 1.6 && pressed_button.seconds() < 1.7 && post_shoot) {
            shoot_all = false;
            shoot_sequence = false;
            reservatorio.resetServos();
        }

        if (shoot_all &&pressed_button.seconds() < 0.08 ) launch_all();
        else if (shoot_sequence&&pressed_button.seconds() < 0.08 ) launch_sequence();
    }



    public void launch_all(){

        if(timer_all.seconds() < 0.15){

            reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.UP);
            reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.UP);
            reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.UP);
        }

        else if (timer_all.seconds() > 0.8 && timer_all.seconds() < 1.5) {
            reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.DOWN);
            reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.DOWN);
            reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.DOWN);
        }else if (timer_all.seconds() > 1.5 && timer_all.seconds() < 1.6) {
            shoot_all = false;
            Capota.setValue(0.65);
            post_shoot = true;
            shoot_requested = false;


        }

    }

    public void launch_sequence(){
        reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.DOWN);
        reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.DOWN);
        reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.DOWN);
        if(timer_all.seconds() > 0 && timer_all.seconds() < 1){

        }

    }
}
