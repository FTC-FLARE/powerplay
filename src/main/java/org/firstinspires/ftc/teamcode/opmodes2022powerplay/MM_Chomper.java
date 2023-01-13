package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Chomper {
    private final MM_OpMode opMode;
    private Servo grabber = null;

//    collector positions
    public static final double CLOSED = 0.07;
    public static final double OPEN = 1.0;

    private ElapsedTime timer = new ElapsedTime();
    private double position = OPEN;


    public MM_Chomper(MM_OpMode opMode) {
        this.opMode = opMode;
        grabber = opMode.hardwareMap.get(Servo.class, "Grabber");
    }


    public double getPosition() {
        return position;
    }

    public void changePosition(double position){
        grabber.setPosition(position);
        this.position = position;

    }
}
