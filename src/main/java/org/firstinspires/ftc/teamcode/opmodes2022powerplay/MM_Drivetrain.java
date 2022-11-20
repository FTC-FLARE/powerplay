package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MM_Drivetrain {
    private MM_OpMode opMode;

    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotorEx leftEncoder = null;
    private DcMotorEx rightEncoder = null;
    private DcMotorEx backEncoder = null;

    static final int FAST = 0;
    static final int SLOW = 1;
    static final int SUPER_SLOW = 2;
    private int slowMode = 0;

    private double flPower = 0;
    private double frPower = 0;
    private double blPower = 0;
    private double brPower = 0;

    public MM_Drivetrain(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void driveWithSticks() {
        double drive = -opMode.gamepad1.left_stick_y;
        double turn = opMode.gamepad1.right_stick_x;
        double strafe = opMode.gamepad1.left_stick_x;

        flPower = (drive + turn + strafe);
        frPower = (drive - turn - strafe);
        blPower = (drive + turn - strafe);
        brPower = (drive - turn + strafe);

        normalize();
        handleSlowMode();
        setMotorPower(flPower, frPower, blPower, brPower);
    }

    private void init() {
        frontLeftDrive = opMode.hardwareMap.get(DcMotor.class, "FLMotor");
        frontRightDrive = opMode.hardwareMap.get(DcMotor.class, "FRMotor");
        backLeftDrive = opMode.hardwareMap.get(DcMotor.class, "BLMotor");
        backRightDrive = opMode.hardwareMap.get(DcMotor.class, "BRMotor");

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftEncoder = opMode.hardwareMap.get(DcMotorEx.class,"FLMotor");
        rightEncoder = opMode.hardwareMap.get(DcMotorEx.class, "FRMotor");
        backEncoder = opMode.hardwareMap.get(DcMotorEx.class, "BLMotor");

    }

    private void setMotorPower(double flPower, double frPower, double blPower, double brPower) {
        frontLeftDrive.setPower(flPower);
        frontRightDrive.setPower(frPower);
        backLeftDrive.setPower(blPower);
        backRightDrive.setPower(brPower);

        opMode.telemetry.addData("front left power:", flPower);
        opMode.telemetry.addData("front right power:", frPower);
        opMode.telemetry.addData("back left power:,", blPower);
        opMode.telemetry.addData("back right power:",  brPower);
    }

    private void normalize() {
        double max = Math.max(Math.abs(flPower), Math.abs(frPower));
        max = Math.max(max, Math.abs(blPower));
        max =  Math.max(max, Math.abs(brPower));

        if (max > 1) {
            flPower = flPower/max;
            frPower = frPower/max;
            blPower = blPower/max;
            brPower = brPower/max;
        }
    }

    private void handleSlowMode() {
        if (opMode.aPressed(opMode.GAMEPAD1)) {
            if (slowMode == SUPER_SLOW) {
                slowMode = FAST;
            }else {
                slowMode += 1;
            }
        }
        if (slowMode == SLOW) {
            flPower = flPower * 0.65;
            frPower = frPower * 0.65;
            blPower = blPower * 0.65;
            brPower = brPower * 0.65;
        } else if (slowMode == SUPER_SLOW) {
            flPower = flPower * 0.35;
            frPower = frPower * 0.35;
            blPower = blPower * 0.35;
            brPower = brPower * 0.35;
        }
        opMode.telemetry.addData("Slowmode level", slowMode);
    }
}
