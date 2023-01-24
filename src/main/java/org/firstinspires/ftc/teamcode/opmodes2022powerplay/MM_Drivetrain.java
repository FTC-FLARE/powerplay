package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.hardware.bosch.BNO055IMU;
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
    private Servo indicator = null;
    private Servo scorer = null;
    private Servo distanceServo = null;

    private DistanceSensor distance = null;

    private final ElapsedTime runtime = new ElapsedTime();

    private static final int RIGHT = -1;
    private static final int LEFT = 1;
    private static final int FORWARD = 1;
    private static final int BACKWARD = -1;
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
    private boolean negative = false;
    private boolean positive = false;
    private boolean bothPosandNeg = false;
    private boolean backwardsMode = false;

    private int leftCurrentTicks = 0;
    private int rightCurrentTicks = 0;
    private int backCurrentTicks = 0;
    private int leftPriorEncoderTarget = 0;
    private int rightPriorEncoderTarget = 0;
    private int backPriorEncoderTarget = 0;
    private int leftCorrectionEncoderTarget = 0;
    private int rightCorrectionEncoderTarget = 0;
    private int kickInMove = DRIVE;
    private int direction = 0;
    private int kickInTicks = 0;
    private double strafePower = 0;
    private boolean strafeIn = false;
    private boolean driveIn = false;

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

    public void diagonalDriveInches(double forwardInches, double strafeInches) {
        diagonalDriveInches(forwardInches, strafeInches, 100,0);
    }

    public void diagonalDriveInches(double forwardInches, double strafeInches, int move, int percentKickIn) {
        prepareToDiagonalDrive(forwardInches, strafeInches, percentKickIn, move);
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 10 && !reachedPositionDiagonalDrive()) {
            opMode.telemetry.addData("forward inches target", forwardInches);
            opMode.telemetry.addData("strafe inches target", strafeInches);
            opMode.telemetry.update();
        }
    }

    public void microscopicDriveInches(double inches) {
        prepareToDrive(inches);
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 1.5 && !reachedPositionMicroscopicDrive()) {
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

        negative = false;
        positive = false;
        bothPosandNeg = false;
    }

    public void prepareToDiagonalDrive(double forwardInches, double strafeInches, int kickInPercent, int kickInMove) {
        int backTargetTicks = backPriorEncoderTarget + MM_Util.inchesToTicks(strafeInches);
        int leftTargetTicks = leftPriorEncoderTarget + MM_Util.inchesToTicks(forwardInches);
        int rightTargetTicks = rightPriorEncoderTarget + MM_Util.inchesToTicks(forwardInches);

        this.kickInMove = kickInMove;
        if (kickInMove == DRIVE) {
            if (strafeInches < 0) {
                direction = RIGHT;
            } else {
                direction = LEFT;
            }
            kickInTicks = kickInPercent * (backTargetTicks - backPriorEncoderTarget) + backPriorEncoderTarget;
            kickInTicks /= 100;
            strafeIn = true;
            driveIn = false;
        } else if (kickInMove == STRAFE) {
            if (forwardInches < 0) {
                direction = BACKWARD;
            } else {
                direction = FORWARD;
            }
            kickInTicks = kickInPercent * (leftTargetTicks - leftPriorEncoderTarget) + leftPriorEncoderTarget;
            kickInTicks /= 100;
            strafeIn = false;
            driveIn = true;
        } else {
            strafeIn = true;
            driveIn = true;
        }

        opMode.pLeftDiagDriveController.setInputRange(leftPriorEncoderTarget, leftTargetTicks);
        opMode.pRightDiagDriveController.setInputRange(rightPriorEncoderTarget, rightTargetTicks);
        opMode.pBackDriveController.setInputRange(backPriorEncoderTarget, backTargetTicks);
        opMode.pLeftDiagDriveController.setSetpoint(leftTargetTicks);
        opMode.pRightDiagDriveController.setSetpoint(rightTargetTicks);
        opMode.pBackDriveController.setSetpoint(backTargetTicks);
        opMode.pLeftDriveController.setInputRange(leftPriorEncoderTarget, leftTargetTicks);
        opMode.pRightDriveController.setInputRange(rightPriorEncoderTarget, rightTargetTicks);
        opMode.pLeftDriveController.setSetpoint(leftTargetTicks);
        opMode.pRightDriveController.setSetpoint(rightTargetTicks);
        leftCorrectionEncoderTarget = leftPriorEncoderTarget;
        rightCorrectionEncoderTarget = rightPriorEncoderTarget;
        leftPriorEncoderTarget = leftTargetTicks;
        rightPriorEncoderTarget = rightTargetTicks;
        backPriorEncoderTarget = backTargetTicks;

        double number = opMode.pRightDriveController.calculatePower(10000);
        double number2 = opMode.pLeftDriveController.calculatePower(10000);
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

    public boolean reachedPositionDiagonalDrive() {
        setDiagonalPower();

        if (strafeIn) {
            strafeIn = !opMode.pBackDriveController.reachedTarget();
        }
        if (driveIn) {
            if (strafeIn) {
                if (opMode.pLeftDiagDriveController.reachedTarget() || opMode.pRightDiagDriveController.reachedTarget()) {
                    driveIn = false;
                    rightCorrectionEncoderTarget = rightPriorEncoderTarget;
                    leftCorrectionEncoderTarget = leftPriorEncoderTarget;
                }
            } else {
                if (opMode.pLeftDriveController.reachedTarget() || opMode.pRightDriveController.reachedTarget()) {
                    driveIn = false;
                    rightCorrectionEncoderTarget = rightPriorEncoderTarget;
                    leftCorrectionEncoderTarget = leftPriorEncoderTarget;
                }
            }
        }
        if (!driveIn && !strafeIn) {
            stop();
            return true;
        }
        return false;
    }

    public boolean reachedPositionMicroscopicDrive() {
        setMicroscopicStraightPower();
        if (Math.abs(leftEncoder.getCurrentPosition() - leftPriorEncoderTarget) < 210 || Math.abs(rightEncoder.getCurrentPosition() - rightPriorEncoderTarget) < 210) {
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

    private void setDiagonalPower() {
        leftCurrentTicks = leftEncoder.getCurrentPosition();
        rightCurrentTicks = rightEncoder.getCurrentPosition();
        backCurrentTicks = -backEncoder.getCurrentPosition();

        checkKickIn();
        if (driveIn) {
            if (strafeIn) {
                leftDrivePower = opMode.pLeftDiagDriveController.calculatePower(leftCurrentTicks);
                rightDrivePower = opMode.pRightDiagDriveController.calculatePower(rightCurrentTicks);
            } else {
                leftDrivePower = opMode.pLeftDriveController.calculatePower(leftCurrentTicks);
                rightDrivePower = opMode.pRightDriveController.calculatePower(rightCurrentTicks);
            }
        } else {
            leftDrivePower = 0;
            rightDrivePower = 0;
        }

        if (strafeIn) {
            strafePower = opMode.pBackDriveController.calculatePower(backCurrentTicks);
        } else {
            strafePower = 0;
        }
        opMode.telemetry.addData("strafein", strafeIn);
        opMode.telemetry.addData("drivein", driveIn);
        opMode.telemetry.addData("right drive power", rightDrivePower);
        opMode.telemetry.addData("left drive power", leftDrivePower);
        opMode.telemetry.addData("calc power", strafePower);
        opMode.telemetry.addData("kickinticks", kickInTicks);

        flPower = -strafePower + leftDrivePower;
        frPower = strafePower + rightDrivePower;
        blPower = strafePower + leftDrivePower;
        brPower = -strafePower + rightDrivePower;

        if (!driveIn) {
            encoderCorrect(strafePower, STRAFE, leftCorrectionEncoderTarget, rightCorrectionEncoderTarget);
        }
        angleStraighten(STRAIGHTEN_P, flPower, frPower);
        normalizeAutos();
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
            positive = true;
        } else {
            flPower = 0.16;
            frPower = 0.16;
            blPower = 0.16;
            brPower = 0.16;
            negative = true;
        } if (!bothPosandNeg && positive && negative) {
            leftPriorEncoderTarget += 250;
            rightPriorEncoderTarget += 250;
            bothPosandNeg = true;
        }

        opMode.telemetry.addData("leftPriorEncoder", leftPriorEncoderTarget);
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

        encoderCorrect(calculatedPower, STRAFE, leftPriorEncoderTarget, rightPriorEncoderTarget);
        angleStraighten(STRAFE_P, calculatedPower, calculatedPower);
        normalize();
        setMotorPower(flPower, frPower, blPower, brPower);
    }

    public void driveWithSticks() {
        double drive = -opMode.gamepad1.left_stick_y;
        double turn = opMode.gamepad1.right_stick_x;
        double strafe = opMode.gamepad1.left_stick_x;


        if(opMode.leftBumperPressed(opMode.GAMEPAD1)){
            backwardsMode = !backwardsMode;
        }
        if (backwardsMode){
            drive = -drive;
            strafe = -strafe;
        }


        flPower = (drive + turn + strafe);
        frPower = (drive - turn - strafe);
        blPower = (drive + turn - strafe);
        brPower = (drive - turn + strafe);

        normalize();
        handleSlowMode();
        if (opMode.gamepad1.right_trigger > 0.1){
            if (slowMode == 1){
                setMotorPower(flPower * .5, frPower * .5, blPower * .5, brPower * .5);

            }else {
                setMotorPower(flPower * .35, frPower * .35, blPower * .35, brPower * .35);
            }

        }else {
            setMotorPower(flPower, frPower, blPower, brPower);
        }

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

    private void normalizeAutos() {
        if (strafeIn && driveIn) {
            flPower /= 1.25;
            frPower /= 1.25;
            blPower /= 1.25;
            brPower /= 1.25;
        }
        normalize();
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

    private void checkKickIn() {
        if (!strafeIn || !driveIn) {
            if (kickInMove == STRAFE) {
                if (direction == BACKWARD) {
                    if (leftCurrentTicks < kickInTicks) {
                        strafeIn = true;
                    }
                } else {
                    if (leftCurrentTicks > kickInTicks) {
                        strafeIn = true;
                    }
                }
            } else {
                if (direction == RIGHT) {
                    if (backCurrentTicks < kickInTicks) {
                        driveIn = true;
                    }
                } else {
                    if (backCurrentTicks > kickInTicks) {
                        driveIn = true;
                    }
                }
            }
        }
    }

    private void encoderCorrect(double calculatedPower, int movement, int leftTarget, int rightTarget) { //TODO RENAME
        if (movement == STRAFE) {
            leftCurrentTicks = leftEncoder.getCurrentPosition();
            rightCurrentTicks = rightEncoder.getCurrentPosition();

            double leftError = leftTarget - leftCurrentTicks;
            double rightError =  rightTarget - rightCurrentTicks;
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
        return distance.getDistance(DistanceUnit.INCH) < 5;
    }

    public boolean correctForJunction(int direction) {
        strafe(direction);
        runtime.reset();
        double startingDistance = distance.getDistance(DistanceUnit.INCH);
        double currentDistance = startingDistance;
        while (opMode.opModeIsActive() && runtime.seconds() < 3) {
            if(currentDistance > 5){
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
        frPower = -power + (0.045 * -direction);
        blPower = -power + (0.045 * -direction);
        brPower = power;
        //angleStraighten(STRAIGHTEN_P, power, power);
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
            indicator = opMode.hardwareMap.get(Servo.class, "autoScore");
            leftOdomLift = opMode.hardwareMap.get(Servo.class,"leftOdometryLift");
            rightOdomLift = opMode.hardwareMap.get(Servo.class,"rightOdometryLift");
            backOdomLift = opMode.hardwareMap.get(Servo.class,"backOdometryLift");

            leftOdomLift.setPosition(1);
            rightOdomLift.setPosition(0);
            backOdomLift.setPosition(1);
            distanceServo.setPosition(1);
            indicator.setPosition(1);
        } else {
            scorer = opMode.hardwareMap.get(Servo.class, "autoScore");
            distanceServo.setPosition(0);
            scorer.setPosition(1);
        }
    }

    public void flipDistanceServo() {
        distanceServo.setPosition(1);
    }

    public void scoreAndUnscore() {
        scorer.setPosition(0);
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 0.65) {
        }
        scorer.setPosition(0.15);
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 0.1) {
        }
        scorer.setPosition(1);
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

