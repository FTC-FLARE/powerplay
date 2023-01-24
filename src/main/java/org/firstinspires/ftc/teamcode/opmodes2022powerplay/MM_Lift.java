package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Lift {
    private final MM_OpMode opMode;
    public MM_Slide slide;
    public MM_Chomper chomper;
    public MM_Turner turner;

    ElapsedTime runtime = new ElapsedTime();

    public MM_Lift(MM_OpMode opMode){
        this.opMode = opMode;
        slide = new MM_Slide(opMode);
        chomper = new MM_Chomper(opMode.hardwareMap);
        turner = new MM_Turner(opMode);
    }

    public void autoStackCollect(int stackLevel){
        slide.moveTowardTarget(MM_Slide.STACK_LEVEL_INCREMENT * (stackLevel - 1) - 8);
        while (opMode.opModeIsActive() && !slide.reachedPosition()) {
            opMode.telemetry.update();
        }
        chomper.toggle();
        runtime.reset();
        opMode.waitSeconds(.8);
    }

    public void autoScore(boolean flipfirst, boolean lastMove, int sleeveColor){
        if (flipfirst){
            runtime.reset();
            opMode.waitSeconds(4);
            turner.changePosition(MM_Turner.BACK);
            runtime.reset();
            opMode.waitSeconds(1.25);
        }
        slide.waitToReachPosition(MM_Slide.SlidePosition.PIVOT_POSITION);
        chomper.toggle();
        runtime.reset();
        slide.waitToReachPosition(MM_Slide.SlidePosition.LOW);
        turner.changePosition(0.885);
        runtime.reset();
        opMode.waitSeconds(1.25);
    }

    public boolean reachedPositionTurner() {
        if (slide.getCurrentTicks() > 1100) {
            turner.changePosition(MM_Turner.SIDE);
        }
        return slide.reachedPosition();
    }

    public void driverControl(){
        slide.driverControl();
        runTurner(slide.tooLowToPivot());

        if (opMode.rightBumperPressed(opMode.GAMEPAD2)) {
            chomper.toggle();
        }
    }

    public void runTurner(boolean tooLowToPivot) {   // used in teleOp
        if (!tooLowToPivot) {
            if (opMode.dpadLeftPressed(opMode.GAMEPAD2)) {
                turner.startMoving(MM_Turner.BACK_TURN_INCREMENT);
            } else if (opMode.dpadRightPressed(opMode.GAMEPAD2)) {
                turner.startMoving(MM_Turner.FRONT_TURN_INCREMENT);
            }
        }

        if (turner.isMoving()) {
            turner.checkIfDoneMoving();
        }
    }
}
