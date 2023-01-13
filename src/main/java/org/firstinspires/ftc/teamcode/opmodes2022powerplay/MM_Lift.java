package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Lift {
    private final MM_OpMode opMode;
    public MM_Slide slide;
    public MM_Collector collector;
    public MM_Turner turner;

    ElapsedTime runtime = new ElapsedTime();

    public MM_Lift(MM_OpMode opMode){
        this.opMode = opMode;
        slide = new MM_Slide(opMode);
        collector = new MM_Collector(opMode, slide);
        turner = new MM_Turner(opMode);

    }

    public void autoStackCollect(int stackLevel){
        slide.setSlideTarget(MM_Slide.SlidePosition.STACK.ticks * (stackLevel - 1) - 8);
        slide.runCollector();
        while (opMode.opModeIsActive() && !slide.reachedPosition()) {
            opMode.telemetry.update();
        }
        collector.autoRunCollector();
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 0.8) {
            opMode.telemetry.update();
        }
    }

    public void autoScore(boolean flipfirst, boolean lastMove, int sleeveColor){
        if (flipfirst){
            collector.engageConesaver();
            runtime.reset();
            while (opMode.opModeIsActive() && runtime.seconds() < 0.4) {
            }
            turner.changeTurnerPosition(turner.BACK);
            runtime.reset();
            while (opMode.opModeIsActive() && runtime.seconds() < 1.25) {
            }
        }
        slide.waitToReachPosition(MM_Slide.SlidePosition.LOW_RELEASE);
        collector.autoRunCollector();
        runtime.reset();
        slide.waitToReachPosition(MM_Slide.SlidePosition.LOW);
        turner.changeTurnerPosition(0.885);
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 1.25){
        }

//        if (lastMove && sleeveColor == 0) {
//            drivetrain.microscopicDriveInches(1.5);
//            drivetrain.rotateToAngle(0);
//        }
//        if (!lastMove) {
//            runtime.reset();
//            while (opMode.opModeIsActive() && runtime.seconds() < 1.25){
//            }
//        }
    }

}
