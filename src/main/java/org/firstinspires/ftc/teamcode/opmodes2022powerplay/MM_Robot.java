package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

public class MM_Robot {
    public MM_Drivetrain drivetrain;
    public MM_Slide slide;
    public MM_Collector collector;

    private MM_OpMode opMode;

    public MM_Robot(MM_OpMode opMode){
        this.opMode = opMode;
    }

    public void init(){
        drivetrain = new MM_Drivetrain(opMode);
        slide = new MM_Slide(opMode);
        collector = new MM_Collector(opMode);
    }


}
