package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

public class MM_Lift {
    private final MM_OpMode opMode;
    public MM_Slide slide;
    public MM_Chomper chomper;
    public MM_Turner turner;

    public MM_Lift(MM_OpMode opMode){
        this.opMode = opMode;
        slide = new MM_Slide(opMode);
        chomper = new MM_Chomper(opMode.hardwareMap);
        turner = new MM_Turner(opMode);
    }

    public void autoStackCollect(int stackLevel, boolean left){
        slide.waitToReachPosition(slide.lowerStackTicks(stackLevel));
        chomper.choke();
        opMode.waitSeconds(0.25);
        slide.waitToReachPosition(MM_Slide.SlidePosition.PIVOT_AUTO);
        if (left) {
            turner.changePosition(MM_Turner.SIDE);
        }
    }

    public void scoreCone() {
        chomper.release();
        opMode.waitSeconds(0.2);
        turner.changePosition(MM_Turner.FRONT);
        opMode.waitSeconds(0.1);
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
            } else if (opMode.leftJoystickPressed(opMode.GAMEPAD2)) {
                if (turner.getPosition() == MM_Turner.FRONT) {
                    turner.startMoving(MM_Turner.SIDE_TURN_INCREMENT_FRONT);
                } else {
                    turner.startMoving(MM_Turner.SIDE_TURN_INCREMENT_BACK);
                }
            }
        }

        if (turner.isMoving()) {
            turner.checkIfDoneMoving();
        }
    }
}
