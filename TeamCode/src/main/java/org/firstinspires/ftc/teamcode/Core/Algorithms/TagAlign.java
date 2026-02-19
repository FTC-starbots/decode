package org.firstinspires.ftc.teamcode.Core.Algorithms;

import org.firstinspires.ftc.teamcode.Core.Parts.LimeServo;

public class TagAlign {
   public boolean validation = false;
    public double width = 0;
    public double pixel = 0;
    LimeServo servo = new LimeServo();




    public void setValues(double CAMpix, boolean CAMstate, double CAMwidth){
            if (CAMstate) {
                validation = true;
                width = CAMwidth;
                pixel = CAMpix;
                // servo.isLimeAllianceTag(false);
            }else {
         validation = false;
         width = 0;
         pixel = 0;


            }

    }

}
