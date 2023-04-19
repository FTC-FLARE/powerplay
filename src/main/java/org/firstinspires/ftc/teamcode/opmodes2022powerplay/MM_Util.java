package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

public class MM_Util {
    private static final double WHEEL_DIAMETER = 2;  // odometry wheels in inches 2.0365 2 for strafe
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    private static final double TICKS_PER_REVOLUTION = 8192; //8192
    private static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE);

    static double voltageToInches(double voltage) {
        return voltage * 87.13491 - 12.0424;
    }

    static int inchesToTicks(double inches) {
        return (int) (inches * TICKS_PER_INCH);
    }

    static double ticksToInches(double ticks) {
        return ticks / TICKS_PER_INCH;
    }
}
