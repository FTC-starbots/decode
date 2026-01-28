package org.firstinspires.ftc.teamcode.Core.Algorithms;
import static java.lang.Thread.sleep;

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
    ElapsedTime timer_split = new ElapsedTime();

   public AimAssisters Capota = new AimAssisters();

    boolean shoot_all;
    boolean shoot_sequence;
    boolean right_vel, right_pos;
        public boolean servo_lifted = false;
    public boolean post_shoot;

    public int last_shoot_type;
    boolean shoot_split;

    public int corretor;

    public int times_atRightVel = 0, last_value = 0;
    public void start(HardwareMap hardware){
        Capota.start(hardware);
        reservatorio.start(hardware);

    }



    public void teleop(Gamepad gamepad, double timer){
        if (gamepad.leftBumperWasPressed()){

            last_shoot_type = 1;
        }
        else if (gamepad.rightBumperWasPressed()){

            last_shoot_type = 2;

        } else if (gamepad.aWasPressed()){

            last_shoot_type = 3;
        }

        if (right_vel) {

            if (last_shoot_type == 1) shoot_all = true;
            if (last_shoot_type == 2) shoot_sequence = true;
            if (last_shoot_type == 3) shoot_split = true;

            pressed_button.reset();
            timer_all.reset();

        }


        if (shoot_all) launch_all_togheter();
        else if (shoot_sequence) launch_sequence();
        else if (shoot_split) launch_all_split(timer);

        if (pressed_button.seconds() > 1.6 && pressed_button.seconds() < 1.7 && post_shoot) {
            shoot_all = false;
            shoot_sequence = false;
            reservatorio.resetServos();
            post_shoot = false;
        }

    }

    public void isAtRightVel(boolean vel){
            right_vel = vel;
    }
    public void isAtRightPos(boolean pos){
        right_pos = pos;
    }

    public void isShooting(boolean shoot){
        if (!shoot) times_atRightVel = 0;
    }

    public void launch_all_split(double timer){

        if(times_atRightVel < 3) {
            if (times_atRightVel == 0 && right_vel && right_pos) {
                timer_split.reset();
                reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.UP);
                times_atRightVel = 1;
                corretor = 1;

            }

            if (times_atRightVel == 1 && right_vel && timer_split.seconds() > timer) {
                timer_split.reset();
                reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.UP);
                times_atRightVel = 2;
                corretor = 0;
            }

            if (times_atRightVel == 2 && right_vel && timer_split.seconds() > timer) {
                timer_split.reset();
                reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.UP);
                times_atRightVel = 3;
                corretor = -1;
                post_shoot = true;
                shoot_split = false;

            }

        }
    }

    public void launch_all_togheter(){

        if(timer_all.seconds() < 0.15){

            reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.UP);
            reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.UP);
            reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.UP);
            servo_lifted = true;

        }

        else if (timer_all.seconds() > 0.8 && timer_all.seconds() < 1.5) {
            reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.DOWN);
            reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.DOWN);
            reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.DOWN);
            servo_lifted = false;
        }else if (timer_all.seconds() > 1.5 && timer_all.seconds() < 1.6) {
            shoot_all = false;
            post_shoot = true;



        }

    }

    public void launch_sequence(){
        reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.DOWN);
        reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.UP);
        reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.DOWN);
        if(timer_all.seconds() > 0 && timer_all.seconds() < 1){

        }
        post_shoot = true;
        shoot_sequence = false;


    }
}
