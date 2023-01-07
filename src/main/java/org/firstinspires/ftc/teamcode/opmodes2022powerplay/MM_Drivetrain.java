package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MM_Drivetrain {
    private final MM_OpMode opMode;

    BNO055IMU imu;

    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;

    private DcMotorEx leftEncoder = null;
    private DcMotorEx rightEncoder = null;
    private DcMotorEx backEncoder = null;

    private Servo leftOdomLift = null;
    private Servo rightOdomLift = null;
    private Servo backOdomLift = null;
    private Servo distanceServo = null;

    private DistanceSensor distance = null;

    private final ElapsedTime runtime = new ElapsedTime();

    private static final int RIGHT = 1;
    private static final int LEFT = -1;
    private static final double SECONDS_PER_DEGREE = 0.025;//??
    private static final double STRAIGHTEN_P = .0780;
    private static final double STRAFE_P = .089;
    private static final double CORRECTION_COEFFICIENT = 0.000055; //Gain per tick
    public static final double SLOW_MULTIPLIER = 0.65;
    public static final double SUPER_SLOW_MULTIPLIER = 0.35;
    static final int FAST = 0;
    static final int SLOW = 1;
    static final int SUPER_SLOW = 2;
    static final int DRIVE = 0;
    static final int STRAFE = 1;

    private int slowMode = SLOW;
    private int previousSlowMode = SLOW;

    private double flPower = 0;
    private double frPower = 0;
    private double blPower = 0;
    private double brPower = 0;
    private double leftDrivePower = 0;
    private double rightDrivePower = 0;

    private int leftCurrentTicks = 0;
    private int rightCurrentTicks = 0;
    private int backCurrentTicks = 0;
    private int leftPriorEncoderTarget = 0;
    private int rightPriorEncoderTarget = 0;
    private int backPriorEncoderTarget = 0;

    private double priorAngle = 0;

    public MM_Drivetrain(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void driveInches(double inches) {
        prepareToDrive(inches);
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 5 && !reachedPositionDrive()) {
            opMode.telemetry.addData("inches target", inches);
            opMode.telemetry.update();
        }
    }

    public void microscopicDriveInches(double inches) {
        prepareToDrive(inches);
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 2 && !reachedPositionMicroscopicDrive()) {
            opMode.telemetry.addData("inches target", inches);
            opMode.telemetry.update();
        }
    }

    public void strafeInches(double inches) {
        prepareToStrafe(inches);
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 5 && !reachedPositionStrafe()) {
            opMode.telemetry.addData("inches target", inches);
            opMode.telemetry.update();
        }
    }

    public void prepareToDrive(double inches) {
        int leftTargetTicks = leftPriorEncoderTarget + MM_Util.inchesToTicks(inches);
        int rightTargetTicks = rightPriorEncoderTarget + MM_Util.inchesToTicks(inches);

        opMode.pLeftDriveController.setInputRange(leftPriorEncoderTarget, leftTargetTicks);
        opMode.pRightDriveController.setInputRange(rightPriorEncoderTarget, rightTargetTicks);
        opMode.pLeftDriveController.setSetpoint(leftTargetTicks);
        opMode.pRightDriveController.setSetpoint(rightTargetTicks);
        leftPriorEncoderTarget = leftTargetTicks;
        rightPriorEncoderTarget = rightTargetTicks;
    }

    public void prepareToStrafe(double inches) {
        int backTargetTicks = backPriorEncoderTarget + MM_Util.inchesToTicks(inches);

        opMode.pBackDriveController.setInputRange(backPriorEncoderTarget, backTargetTicks);
        opMode.pBackDriveController.setSetpoint(backTargetTicks);
        backPriorEncoderTarget = backTargetTicks;
    }

    public boolean reachedPositionDrive() { //this also sets the motor power
        setStraightPower();
        if (opMode.pLeftDriveController.reachedTarget() || opMode.pRightDriveController.reachedTarget()) {
            stop();
            return true;
        }
        return false;
    }

    public boolean reachedPositionMicroscopicDrive() {
        setMicroscopicStraightPower();
        if (Math.abs(leftEncoder.getCurrentPosition() - leftPriorEncoderTarget) < 175 || Math.abs(rightEncoder.getCurrentPosition() - rightPriorEncoderTarget) < 175) {
            stop();
            return true;
        }
        return false;
    }

    public boolean reachedPositionStrafe() {
        setStrafePower();
        if (opMode.pBackDriveController.reachedTarget()) {
            stop();
            return true;
        }
        return false;
    }

    private void setStraightPower() {
        leftCurrentTicks = leftEncoder.getCurrentPosition();
        rightCurrentTicks = rightEncoder.getCurrentPosition();

        leftDrivePower = opMode.pLeftDriveController.calculatePower(leftCurrentTicks);
        rightDrivePower = opMode.pRightDriveController.calculatePower(rightCurrentTicks);

        flPower = leftDrivePower;
        frPower = rightDrivePower;
        blPower = leftDrivePower;
        brPower = rightDrivePower;

        angleStraighten(STRAIGHTEN_P, leftDrivePower, rightDrivePower);
        normalize();
        setMotorPower(flPower, frPower, blPower, brPower);
    }

    private void setMicroscopicStraightPower() {
        leftCurrentTicks = leftEncoder.getCurrentPosition();
        rightCurrentTicks = rightEncoder.getCurrentPosition();

        if (leftCurrentTicks > leftPriorEncoderTarget) {
            flPower = -0.16;
            frPower = -0.16;
            blPower = -0.16;
            brPower = -0.16;
        } else {
            flPower = 0.16;
            frPower = 0.16;
            blPower = 0.16;
            brPower = 0.16;
        }
        angleStraighten(STRAIGHTEN_P, flPower, frPower);
        normalize();
        setMotorPower(flPower, frPower, blPower, brPower);
    }

    private void setStrafePower() {
        backCurrentTicks = -backEncoder.getCurrentPosition();

        double calculatedPower = opMode.pBackDriveController.calculatePower(backCurrentTicks);
        opMode.telemetry.addData("calc power", calculatedPower);
        flPower = -calculatedPower;
        frPower = calculatedPower;
        blPower = calculatedPower;
        brPower = -calculatedPower;

        encoderCorrect(calculatedPower, STRAFE);
        angleStraighten(STRAFE_P, calculatedPower, calculatedPower);
        normalize();
        setMotorPower(flPower, frPower, blPower, brPower);
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

        opMode.telemetry.addData("first heading", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
    }

    private void switchEncoderMode(DcMotor.RunMode runMode) {
        frontLeftDrive.setMode(runMode);
        frontRightDrive.setMode(runMode);
        backLeftDrive.setMode(runMode);
        backRightDrive.setMode(runMode);
        leftEncoder.setMode(runMode);
        rightEncoder.setMode(runMode);
        backEncoder.setMode(runMode);
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

    private void stop() {
        setMotorPower(0,0,0,0);
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
        if (opMode.aPressed(opMode.GAMEPAD1) || opMode.rightBumperPressed(opMode.GAMEPAD1)){
            if (slowMode == FAST) {
                slowMode = SLOW;

            } else if(slowMode == SLOW) {
                slowMode = FAST;
            }else{ // must have been super-slow
                slowMode = previousSlowMode;
            }
        }
        if (opMode.bPressed(opMode.GAMEPAD1)) {
            if (slowMode == SUPER_SLOW) {
                slowMode = previousSlowMode;
            } else {
                previousSlowMode = slowMode;
                slowMode = SUPER_SLOW;
            }
        }

        double speedFactor = 1;
        if (slowMode == SLOW) {
            speedFactor = SLOW_MULTIPLIER;
        } else if (slowMode == SUPER_SLOW) {
            speedFactor = SUPER_SLOW_MULTIPLIER;
        }

        flPower *= speedFactor;
        frPower *= speedFactor;
        blPower *= speedFactor;
        brPower *= speedFactor;

        opMode.telemetry.addData("Slowmode level", slowMode);
    }

    public void rotateToAngle(double targetAngle){
        double timeOut = Math.max(2, Math.abs(SECONDS_PER_DEGREE * targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle));

        int rightStartingTicks = rightEncoder.getCurrentPosition();
        int leftStartingTicks = leftEncoder.getCurrentPosition();
        int backStartingTicks = backEncoder.getCurrentPosition();

        opMode.pTurnController.setInputRange(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, targetAngle);
        opMode.pTurnController.setSetpoint(targetAngle);
        runtime.reset();

        do {
            double turnPower = Math.abs(opMode.pTurnController.calculatePower(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle));

            if(correctedAngle(opMode.pTurnController.getCurrentError()) > 0){
                setMotorPower(-turnPower, turnPower, -turnPower, turnPower);
            }else {
                setMotorPower(turnPower, -turnPower, turnPower, -turnPower);
            }
            opMode.telemetry.addData("target reached", opMode.pTurnController.reachedTarget());
            opMode.telemetry.update();

        } while (opMode.opModeIsActive() && !opMode.pTurnController.reachedTarget() && runtime.seconds() < timeOut);

        stop();
        priorAngle = targetAngle;
        rightPriorEncoderTarget = rightPriorEncoderTarget - rightStartingTicks + rightEncoder.getCurrentPosition();
        leftPriorEncoderTarget = leftPriorEncoderTarget - leftStartingTicks + leftEncoder.getCurrentPosition();
        backPriorEncoderTarget = backPriorEncoderTarget - backStartingTicks + backEncoder.getCurrentPosition();
    }

    private void encoderCorrect(double calculatedPower, int movement) { //TODO RENAME
        if (movement == STRAFE) {
            leftCurrentTicks = leftEncoder.getCurrentPosition();
            rightCurrentTicks = rightEncoder.getCurrentPosition();

            double leftError = leftPriorEncoderTarget - leftCurrentTicks;
            double rightError =  rightPriorEncoderTarget - rightCurrentTicks;
            //modeled after straighten
            flPower = flPower + (leftError * CORRECTION_COEFFICIENT * Math.abs(calculatedPower));
            frPower = frPower + (rightError * CORRECTION_COEFFICIENT * Math.abs(calculatedPower));
            blPower = blPower + (leftError * CORRECTION_COEFFICIENT * Math.abs(calculatedPower));
            brPower = brPower + (rightError * CORRECTION_COEFFICIENT * Math.abs(calculatedPower));
        } else {
            backCurrentTicks = -backEncoder.getCurrentPosition();

            double backError = backPriorEncoderTarget - backCurrentTicks;

            flPower = flPower - (backError * CORRECTION_COEFFICIENT * Math.abs(calculatedPower));
            frPower = frPower + (backError * CORRECTION_COEFFICIENT * Math.abs(calculatedPower));
            blPower = blPower - (backError * CORRECTION_COEFFICIENT * Math.abs(calculatedPower));
            brPower = brPower + (backError * CORRECTION_COEFFICIENT * Math.abs(calculatedPower));
        }
    }

    private void angleStraighten(double pCoefficient, double leftCalculated, double rightCalculated) {
        double headingError = correctedAngle(priorAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);

        flPower = flPower - (headingError * pCoefficient * Math.abs(leftCalculated));
        frPower = frPower + (headingError * pCoefficient * Math.abs(rightCalculated));
        blPower = blPower - (headingError * pCoefficient * Math.abs(leftCalculated));
        brPower = brPower + (headingError * pCoefficient * Math.abs(rightCalculated));
    }

    public double getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private double correctedAngle(double angle) {
        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    public boolean withinJunctionRange() {
        return distance.getDistance(DistanceUnit.INCH) < 4;
    }

    public boolean correctForJunction(int direction) {
        strafe(direction);
        runtime.reset();
        double startingDistance = distance.getDistance(DistanceUnit.INCH);
        double currentDistance = startingDistance;
        while (opMode.opModeIsActive() && runtime.seconds() < 3) {
            if(currentDistance > 4){
                if (runtime.seconds() > 1.5 ) {
                    strafe(-direction);
                }
            }else{
                stop();
                return true;
            }
            currentDistance = distance.getDistance(DistanceUnit.INCH);
        }

        stop();
        return false;

    }

    public void strafe(int direction) {
        //left is negative
        double power = 0.26 * direction;
        flPower = power;
        frPower = -power;
        blPower = -power;
        brPower = power;
        angleStraighten(STRAIGHTEN_P, power, power);
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

        initServos();

        leftEncoder = opMode.hardwareMap.get(DcMotorEx.class,"BRMotor"); // port 3
        rightEncoder = opMode.hardwareMap.get(DcMotorEx.class, "FLMotor"); // port 0
        backEncoder = opMode.hardwareMap.get(DcMotorEx.class, "BLMotor"); // port 2

        switchEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        switchEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        distance = opMode.hardwareMap.get(DistanceSensor.class, "distance");
    }

    private void initServos(){
        distanceServo = opMode.hardwareMap.get(Servo.class, "distanceServo");
        if(opMode.getClass() == MM_TeleOp.class){
            leftOdomLift = opMode.hardwareMap.get(Servo.class,"leftOdometryLift");
            rightOdomLift = opMode.hardwareMap.get(Servo.class,"rightOdometryLift");
            backOdomLift = opMode.hardwareMap.get(Servo.class,"backOdometryLift");

            leftOdomLift.setPosition(1);
            rightOdomLift.setPosition(0);
            backOdomLift.setPosition(1);
            distanceServo.setPosition(1);
        } else {
            distanceServo.setPosition(0);
        }
    }
}
/*    public void correctForJunction() {
        double startingDistance = distance.getDistance(DistanceUnit.INCH);
        if (startingDistance > 4) {
            strafe(RIGHT);
            runtime.reset();
            double currentDistance = distance.getDistance(DistanceUnit.INCH);
            while (currentDistance > 4) {
                currentDistance = distance.getDistance(DistanceUnit.INCH);
                if (runtime.seconds() > 1) {
                    if (currentDistance > startingDistance) {
                        strafe(LEFT);
                    }
                }
                if (runtime.seconds() > 3) {
                    currentDistance = 0;
                    //set an abort variable
                }
            }
            stop();
        }
    }*/

