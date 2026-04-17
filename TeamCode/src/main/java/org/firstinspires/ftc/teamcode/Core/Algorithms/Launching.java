package org.firstinspires.ftc.teamcode.Core.Algorithms;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Core.Parts.AimAssisters;
import org.firstinspires.ftc.teamcode.Core.Parts.Reservoirs;
import org.firstinspires.ftc.teamcode.Core.Parts.Reservoirs.ReservoirsState;

public class Launching {
    public Reservoirs reservatorio = new Reservoirs();
    ElapsedTime timer_all = new ElapsedTime();
    ElapsedTime pressed_button = new ElapsedTime();
    ElapsedTime timer_split = new ElapsedTime();
    ColorSensor sensor = new ColorSensor();
    public ColorSensor.BALL[] shootOrder = new ColorSensor.BALL[3];
    public ColorSensor.BALL[] gateOrder = new ColorSensor.BALL[9];
    ColorSensor.BALL bola1, bola2, bola3;
    public AimAssisters Capota = new AimAssisters();

    boolean shoot_all = false;
    boolean shoot_sequence= false;
    boolean shoot_split_sen= false;
    boolean right_vel, right_pos;
    public boolean servo_lifted = false;
    public boolean post_shoot= false;

    public int shoot_type = 0;
    boolean shoot_split= false;

    public int corretor;
    boolean toShootauto, isTeleop;

    double id;
    int Balls;


    public int times_atRightVel = 0, last_value = 0;
    public void start(HardwareMap hardware){
        Capota.start(hardware);
        reservatorio.start(hardware);

    }


    public void auto(double timer, int sequence, boolean toShoot){
        isTeleop = false;
        toShootauto = toShoot;
        if (right_vel && toShoot){
            launch_auto(sequence,timer);
            pressed_button.reset();
            timer_all.reset();
            shoot_type = 3;
        }

        if (!toShootauto && !isTeleop) times_atRightVel = 0;


}

    public void teleop(Gamepad gamepad, double timer){
        isTeleop = true;
        if (gamepad.left_bumper || gamepad.a){
            shoot_all = false;
            shoot_type = 3;
        }
        else if (gamepad.right_bumper){
            shoot_all = false;
            shoot_type = 2;

        } else if (gamepad.y || gamepad.triangle) {
            shoot_type = 1;
        } else if (gamepad.dpad_left){
            shoot_all = false;
            shoot_type = 4;
        }else if (gamepad.dpad_up){
            shoot_all = false;
            shoot_type = 5;
        }else if (gamepad.dpad_right){
            shoot_all = false;
            shoot_type = 6;
        }else if (gamepad.dpad_down){
            shoot_all = false;
            shoot_type = 7;
        }

        if (gamepad.rightBumperWasPressed()){
            shootOrder[0] = ColorSensor.BALL.PURPLE;
            shootOrder[1] = ColorSensor.BALL.GREEN;
            shootOrder[2] = ColorSensor.BALL.PURPLE;

//            if (Balls <7) {
//                shootOrder[0] = gateOrder[Balls];
//                shootOrder[1] = gateOrder[Balls + 1];
//                shootOrder[2] = gateOrder[Balls + 2];
//                timer_split.reset();
//            }else if (Balls == 7){
//                shootOrder[0] = gateOrder[7];
//                shootOrder[1] = gateOrder[8];
//                shootOrder[2] = gateOrder[8];
//                timer_split.reset();
//            }else {
//                shootOrder[0] = gateOrder[8];
//                shootOrder[1] = gateOrder[8];
//                shootOrder[2] = gateOrder[8];
//                timer_split.reset();
//            }
        }


            if (shoot_type == 1) {
                shoot_all = true;
                pressed_button.reset();
            }
            if (shoot_type == 2){
                shoot_sequence = true;
                pressed_button.reset();
            }
            if (shoot_type == 3){
                shoot_split_sen = true;
                pressed_button.reset();
            }
            if (shoot_type == 7){
                shoot_split = true;
                pressed_button.reset();
            }
            if (shoot_type == 4){
                if (right_vel) {
                    reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.UP);
                    corretor = 1;
                    post_shoot = true;
                    pressed_button.reset();
                    timer_all.reset();
                    shoot_type = 0;
                }
            }
            if (shoot_type == 5) {
                if (right_vel) {
                    reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.UP);
                    corretor = 0;
                    post_shoot = true;
                    pressed_button.reset();
                    timer_all.reset();
                    shoot_type = 0;
                }
            }
            if (shoot_type == 6){
                if (right_vel){
                reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.UP);
                corretor = -1;
                post_shoot = true;
                    shoot_type = 0;
                    pressed_button.reset();
                    timer_all.reset();
                }
            }






        if (shoot_all) launch_all_togheter();
        else if (shoot_sequence) launch_sequence(timer+0.2);
        else if (shoot_split_sen) launch_all_split_sensor(timer);
        else if (shoot_split) launch_all_split(timer);


        if (pressed_button.seconds() > 2 && pressed_button.seconds() < 2.3 && post_shoot) {
            shoot_all = false;
            shoot_sequence = false;
            shoot_split_sen = false;
            shoot_split = false;
            shoot_type = 0;
        }

    }

    public void isAtRightVel(boolean vel){
            right_vel = vel;
    }
    public void isAtRightPos(boolean pos){
        right_pos = pos;
    }

    public void ispost_shoot(boolean shoot){post_shoot = shoot;}

    public void isShooting(boolean shoot){
        if (!shoot) times_atRightVel = 0;
    }

    public void setGateOrder(ColorSensor.BALL[] balls){
        gateOrder[0] = balls[0];
        gateOrder[1] = balls[1];
        gateOrder[2] = balls[2];
        gateOrder[3] = balls[3];
        gateOrder[4] = balls[4];
        gateOrder[5] = balls[5];
        gateOrder[6] = balls[6];
        gateOrder[7] = balls[7];
        gateOrder[8] = balls[8];
    }
    public void setBallsNumber(double ball){Balls = (int)ball;}

    public void SensorReadings(ColorSensor.BALL b1, ColorSensor.BALL b2, ColorSensor.BALL b3){
        bola1 = b1;
        bola2 = b2;
        bola3 = b3;
    }
    public void launch_all_split_sensor(double timer){

        if(times_atRightVel < 3) {
            if (times_atRightVel == 0 && right_vel && right_pos) {
                timer_split.reset();
                 if (bola1 != ColorSensor.BALL.NONE) {
                    reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.UP);
                    times_atRightVel = 1;
                    corretor = 1;
                }else if (bola2 != ColorSensor.BALL.NONE) {
                    reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.UP);
                    times_atRightVel = 2;
                    corretor = 0;
                }else if (bola3 != ColorSensor.BALL.NONE) {
                    reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.UP);
                    times_atRightVel = 3;
                    corretor = -1;
                     post_shoot = true;
                     shoot_split_sen = false;
                }else{
                     post_shoot = true;
                     shoot_split_sen = false;
                     times_atRightVel = 3;
                 }
            }

            if (times_atRightVel == 1 && right_vel && timer_split.seconds() > timer) {
                timer_split.reset();
                if (bola2 != ColorSensor.BALL.NONE) {
                    reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.UP);
                    times_atRightVel = 2;
                    corretor = 0;
                }else if (bola3 != ColorSensor.BALL.NONE) {
                    reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.UP);
                    times_atRightVel = 3;
                    corretor = -1;
                    post_shoot = true;
                    shoot_split_sen = false;
                }else{
                    post_shoot = true;
                    shoot_split_sen = false;
                    times_atRightVel = 3;
                }
            }

            if (times_atRightVel == 2 && right_vel && timer_split.seconds() > timer) {
                timer_split.reset();
                if (bola3 != ColorSensor.BALL.NONE) {
                    reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.UP);
                    times_atRightVel = 3;
                    corretor = -1;
                    post_shoot = true;
                    shoot_split_sen = false;
                }else{
                    post_shoot = true;
                    shoot_split_sen = false;
                    times_atRightVel = 3;
                }

            }

        }else shoot_type = 0;
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

        }else shoot_type = 0;
    }


    public void launch_all_togheter(){

        if(times_atRightVel < 3) {
            if (times_atRightVel == 0 && right_vel && right_pos) {
                timer_split.reset();

                reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.UP);
                reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.UP);
                times_atRightVel = 1;


            }

            if (times_atRightVel == 1 && timer_split.seconds() > 0.008) {
                timer_split.reset();
                reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.UP);
                times_atRightVel = 3;

                post_shoot = true;
                shoot_all = false;

            }


        }else shoot_type = 0;

    }

    public void launch_sequence(double timer){

        if(times_atRightVel < 3) {
            if (times_atRightVel == 0 && right_vel && right_pos) {

                timer_split.reset();
                if (bola1 == ColorSensor.BALL.NONE && bola2 == ColorSensor.BALL.NONE && bola3 == ColorSensor.BALL.NONE) {
                    post_shoot = true;
                    shoot_sequence = false;
                    times_atRightVel = 3;

                }else if (shootOrder[0] == bola1){
                    reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.UP);
                    corretor = 1;
                    times_atRightVel = 1;
                    sensor.resetReservoir(1);
                }else if (shootOrder[0] == bola2){
                    reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.UP);
                    corretor = 0;
                    times_atRightVel = 1;
                    sensor.resetReservoir(2);
                }else if (shootOrder[0] == bola3){
                    reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.UP);
                    corretor = -1;
                    times_atRightVel = 1;
                    sensor.resetReservoir(3);
                } else{
                    reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.UP);
                    corretor = 1;
                    times_atRightVel = 1;
                    sensor.resetReservoir(1);
                }




            }

            if (times_atRightVel == 1 && right_vel && timer_split.seconds() > timer) {
                timer_split.reset();

                if (shootOrder[1] == bola1){
                    reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.UP);
                    corretor = 1;
                    times_atRightVel = 2;
                    sensor.resetReservoir(1);
                }else if (shootOrder[1] == bola2){
                    reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.UP);
                    corretor = 0;
                    times_atRightVel = 2;
                    sensor.resetReservoir(2);
                }else if (shootOrder[1] == bola3){
                    reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.UP);
                    corretor = -1;
                    times_atRightVel = 2;
                    sensor.resetReservoir(3);
                } else{
                    reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.UP);
                    corretor = 0;
                    times_atRightVel = 2;
                    sensor.resetReservoir(2);
                }
            }

            if (times_atRightVel == 2 && right_vel && timer_split.seconds() > timer) {
                timer_split.reset();

                if (shootOrder[2] == bola1){
                    reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.UP);
                    corretor = 1;
                    times_atRightVel = 3;
                    sensor.resetReservoir(1);
                }else if (shootOrder[2] == bola2){
                    reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.UP);
                    corretor = 0;
                    times_atRightVel = 3;
                    sensor.resetReservoir(2);
                }else if (shootOrder[2] == bola3){
                    reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.UP);
                    corretor = -1;
                    times_atRightVel = 3;
                    sensor.resetReservoir(3);
                } else{
                    reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.UP);
                    corretor = -1;
                    times_atRightVel = 3;
                    sensor.resetReservoir(3);
                }
                post_shoot = true;
                shoot_sequence = false;

            }

        }else shoot_type = 0;
    }

    public void launch_auto(int sequence, double timer){
        if (sequence == 1){
            if(times_atRightVel < 4) {
                if (times_atRightVel == 0 && right_vel && timer_split.seconds() > 0.1) {
                    timer_split.reset();
                    reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.UP);
                    times_atRightVel = 1;
                    corretor = 1;


                }

                if (times_atRightVel == 1 && right_vel && timer_split.seconds() > timer) {
                    timer_split.reset();
                    reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.UP);
                    times_atRightVel = 2;
                }

                if (times_atRightVel == 2 && right_vel && timer_split.seconds() > timer) {
                    timer_split.reset();
                    reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.UP);
                    times_atRightVel = 3;


                }if (times_atRightVel == 3  && timer_split.seconds() > timer) {
                    timer_split.reset();
                    times_atRightVel = 4;
                    post_shoot = true;

                }

            }
        }
        else if (sequence ==2) {
            if(times_atRightVel < 4) {
                if (times_atRightVel == 0 && right_vel ) {
                    timer_split.reset();
                    reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.UP);
                    times_atRightVel = 1;
                    corretor = 0;

                }

                if (times_atRightVel == 1 && right_vel && timer_split.seconds() > timer) {
                    timer_split.reset();
                    reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.UP);
                    times_atRightVel = 2;
                    corretor = 1;
                }

                if (times_atRightVel == 2 && right_vel && timer_split.seconds() > timer) {
                    timer_split.reset();
                    reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.UP);
                    times_atRightVel = 3;
                    corretor = -1;




                }if (times_atRightVel == 3  && timer_split.seconds() > timer) {
                    timer_split.reset();
                    times_atRightVel = 4;
                    post_shoot = true;

                }

            }
        } else if (sequence == 3) {
            if(times_atRightVel < 4) {
                if (times_atRightVel == 0 && right_vel ) {
                    timer_split.reset();
                    reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.UP);
                    times_atRightVel = 1;
                    corretor = -1;

                }

                if (times_atRightVel == 1 && right_vel && timer_split.seconds() > timer) {
                    timer_split.reset();
                    reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.UP);
                    times_atRightVel = 2;
                    corretor = 0;
                }

                if (times_atRightVel == 2 && right_vel && timer_split.seconds() > timer) {
                    timer_split.reset();
                    reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.UP);
                    times_atRightVel = 3;
                    corretor = 1;


                }
                if (times_atRightVel == 3  && timer_split.seconds() > timer) {
                    timer_split.reset();
                    times_atRightVel = 4;
                    post_shoot = true;

                }

            }
        }else if (sequence ==4) {
            if(times_atRightVel < 4) {
                if (times_atRightVel == 0 && right_vel ) {
                    timer_split.reset();
                    reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.UP);
                    times_atRightVel = 1;
                    corretor = 0;

                }

                if (times_atRightVel == 1 && right_vel && timer_split.seconds() > timer) {
                    timer_split.reset();
                    reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.UP);
                    times_atRightVel = 2;
                    corretor = -1;
                }

                if (times_atRightVel == 2 && right_vel && timer_split.seconds() > timer) {
                    timer_split.reset();
                    reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.UP);
                    times_atRightVel = 3;
                    corretor = 1;


                }if (times_atRightVel == 3  && timer_split.seconds() > timer) {
                    timer_split.reset();
                    times_atRightVel = 4;

                    post_shoot = true;
                }

            }
        }
        else if (sequence ==5) {
            if(times_atRightVel < 4) {
                if (times_atRightVel == 0 && right_vel) {
                    timer_split.reset();
                    reservatorio.servoPosition(reservatorio.Reservoir_1, ReservoirsState.UP);
                    times_atRightVel = 1;
                    corretor = 1;

                }

                if (times_atRightVel == 1 && right_vel && timer_split.seconds() > timer) {
                    timer_split.reset();
                    reservatorio.servoPosition(reservatorio.Reservoir_3, ReservoirsState.UP);
                    times_atRightVel = 2;
                    corretor = -1;
                }

                if (times_atRightVel == 2 && right_vel && timer_split.seconds() > timer) {
                    timer_split.reset();
                    reservatorio.servoPosition(reservatorio.Reservoir_2, ReservoirsState.UP);
                    times_atRightVel = 3;
                    corretor = 0;



                }if (times_atRightVel == 3 && timer_split.seconds() > timer) {
                    timer_split.reset();
                    times_atRightVel = 4;
                    post_shoot = true;

                }

            }
        }
    }

}
