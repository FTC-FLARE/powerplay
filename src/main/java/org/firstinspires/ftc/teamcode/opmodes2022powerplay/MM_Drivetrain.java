package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
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

    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;

    private DcMotorEx leftEncoder = null;
    private DcMotorEx rightEncoder = null;
    private DcMotorEx backEncoder = null;

    private Servo leftOdomLift = null;
    private Servo rightOdomLift = null;
    private Servo backOdomLift = null;
    private Servo indicator = null;
    private Servo scorer = null;
    private Servo distanceServo = null;

    private ColorSensor rightTapeSensor = null;
    private ColorSensor leftTapeSensor = null;
    private DistanceSensor distance = null;

    private final ElapsedTime runtime = new ElapsedTime();

    public static final double MIN_STRAFE_POWER = 0.292;
    private static final double SECONDS_PER_DEGREE = 0.025;//??
    private static final double STRAIGHTEN_P = .0840; //.0780
    private static final double STRAFE_P = .089;
    private static final double CORRECTION_COEFFICIENT = 0.000055; //Gain per tick
    private static final double DISTANCE_P_COEFFICIENT = 0.013;
    public static final double SLOW_MULTIPLIER = 0.65;
    public static final double SUPER_SLOW_MULTIPLIER = 0.35;
    public static final double MIN_DRIVE_POWER = 0.16;

    private static final int RIGHT = -1;
    private static final int LEFT = 1;
    private static final int FORWARD = 1;
    private static final int BACKWARD = -1;
    private static final int FAST = 0;
    private static final int SLOW = 1;
    private static final int SUPER_SLOW = 2;
    public static final int DRIVE = 0;
    public static final int STRAFE = 1;

    private static final double LEFT_TAPE_RED = 280;
    private static final double RIGHT_TAPE_RED = 300;
    private static final double LEFT_TAPE_BLUE = 300;
    private static final double RIGHT_TAPE_BLUE = 320;

    private int slowMode = SLOW;
    private int previousSlowMode = SLOW;
    private boolean backwardsMode = false;

    private double flPower = 0;
    private double frPower = 0;
    private double blPower = 0;
    private double brPower = 0;
    private double leftDrivePower = 0;
    private double rightDrivePower = 0;
    private double strafePower = 0;

    private int leftCurrentTicks = 0;
    private int rightCurrentTicks = 0;
    private int backCurrentTicks = 0;
    private int leftPriorEncoderTarget = 0;
    private int rightPriorEncoderTarget = 0;
    private int backPriorEncoderTarget = 0;
    private double priorAngle = 0;
    private int leftStart = 0;
    private int rightStart = 0;
    private int secondMove = DRIVE;
    private int direction = 0;
    private int kickInTicks = 0;
    private boolean colorKickOut = false;
    private boolean strafing = false;
    private boolean driving = false;

    private double leftTapeVal = 0;
    private double rightTapeVal = 0;
    private boolean tapeHandled = false;
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
        while (opMode.opModeIsActive() && runtime.seconds() < 1.5 && !reachedPositionMicroscopicDrive()) {
            opMode.telemetry.addData("inches target", inches);
            opMode.telemetry.update();
        }
    }

    public void diagonalDriveInches(double forwardInches, double strafeInches) {
        diagonalDriveInches(forwardInches, strafeInches, 100,0);
    }

    public void diagonalDriveInches(double forwardInches, double strafeInches, int move, int percentKickIn) {
        prepareToDiagonalDrive(forwardInches, strafeInches, percentKickIn, move, false);
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 10 && !reachedPositionDiagonalDrive()) {
            opMode.telemetry.addData("forward inches target", forwardInches);
            opMode.telemetry.addData("strafe inches target", strafeInches);
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

    public void microscopicStrafeInches(double inches) {
        prepareToStrafe(inches);
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 0.7 && !reachedPositionMicroscopicStrafe()) {
            opMode.telemetry.addData("inches target", inches);
            opMode.telemetry.update();
        }
    }

    public void followTapeToStack() {
        prepareToTapeDrive();
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 5 && !reachedPositionTapeDrive()) {
            opMode.telemetry.update();
        }
    }

    public void prepareToDrive(double inches) {
        int leftTargetTicks = leftPriorEncoderTarget + MM_Util.inchesToTicks(inches);
        int rightTargetTicks = rightPriorEncoderTarget + MM_Util.inchesToTicks(inches);

        opMode.pLeftDriveController.setup(leftPriorEncoderTarget, leftTargetTicks);
        opMode.pRightDriveController.setup(rightPriorEncoderTarget, rightTargetTicks);
        leftPriorEncoderTarget = leftTargetTicks;
        rightPriorEncoderTarget = rightTargetTicks;
    }

    public void prepareToStrafe(double inches) {
        int backTargetTicks = backPriorEncoderTarget + MM_Util.inchesToTicks(inches);

        opMode.pBackDriveController.setup(backPriorEncoderTarget, backTargetTicks);
        backPriorEncoderTarget = backTargetTicks;
    }

    public void prepareToDiagonalDrive(double driveInches, double strafeInches, int kickInPercent, int secondMove, boolean colorKickOut) {
        int backTargetTicks = backPriorEncoderTarget + MM_Util.inchesToTicks(strafeInches);
        int leftTargetTicks = leftPriorEncoderTarget + MM_Util.inchesToTicks(driveInches);
        int rightTargetTicks = rightPriorEncoderTarget + MM_Util.inchesToTicks(driveInches);

        this.colorKickOut = colorKickOut;
        this.secondMove = secondMove;
        if (secondMove == DRIVE) {
            if (strafeInches < 0) {
                direction = RIGHT;
            } else {
                direction = LEFT;
            }
            kickInTicks = (kickInPercent/100) * (backTargetTicks - backPriorEncoderTarget) + backPriorEncoderTarget;
            strafing = true;
            driving = false;
        } else if (secondMove == STRAFE) {
            if (driveInches < 0) {
                direction = BACKWARD;
            } else {
                direction = FORWARD;
            }
            kickInTicks = (kickInPercent/100) * (leftTargetTicks - leftPriorEncoderTarget) + leftPriorEncoderTarget;
            strafing = false;
            driving = true;
        } else {
            strafing = true;
            driving = true;
        }

        opMode.pLeftDiagDriveController.setup(leftPriorEncoderTarget, leftTargetTicks);
        opMode.pRightDiagDriveController.setup(rightPriorEncoderTarget, rightTargetTicks);
        opMode.pLeftDriveController.setup(leftPriorEncoderTarget, leftTargetTicks);
        opMode.pRightDriveController.setup(rightPriorEncoderTarget, leftTargetTicks);
        opMode.pBackDriveController.setup(backPriorEncoderTarget, backTargetTicks);
        
        leftStart = leftPriorEncoderTarget;
        rightStart = rightPriorEncoderTarget;
        leftPriorEncoderTarget = leftTargetTicks;
        rightPriorEncoderTarget = rightTargetTicks;
        backPriorEncoderTarget = backTargetTicks;

        //used to update controllers to prevent stoppage
        opMode.pRightDriveController.calculatePower(10000);
        opMode.pLeftDriveController.calculatePower(10000);
    }

    public void prepareToTapeDrive() {
        tapeHandled = false;
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
        if (Math.abs(leftEncoder.getCurrentPosition() - leftPriorEncoderTarget) < 210 || Math.abs(rightEncoder.getCurrentPosition() - rightPriorEncoderTarget) < 210) {
            stop();
            return true;
        }
        return false;
    }

    public boolean reachedPositionDiagonalDrive() {
        setDiagonalPower();

        if (colorKickOut) {
            return checkColors();
        }
        if (strafing) {
            strafing = !opMode.pBackDriveController.reachedTarget();
        }
        if (driving) {
            if (strafing) {
                if (opMode.pLeftDiagDriveController.reachedTarget() || opMode.pRightDiagDriveController.reachedTarget()) {
                    driving = false;
                    rightStart = rightPriorEncoderTarget;
                    leftStart = leftPriorEncoderTarget;
                }
            } else {
                if (opMode.pLeftDriveController.reachedTarget() || opMode.pRightDriveController.reachedTarget()) {
                    driving = false;
                    rightStart = rightPriorEncoderTarget;
                    leftStart = leftPriorEncoderTarget;
                }
            }
        }

        if (!driving && !strafing) {
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

    public boolean reachedPositionMicroscopicStrafe() {
        setMicroscopicStrafePower();
        opMode.pBackDriveController.calculatePower(backCurrentTicks);
        if (opMode.pBackDriveController.reachedTarget()) {
            stop();
            return true;
        }
        return false;
    }

    public boolean reachedPositionTapeDrive() {
        setTapePower();
        double frontDistance = getFrontDistance();
        if (frontDistance < 4.2 && frontDistance > 3.2) {
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

        setPowerVariables(leftDrivePower, rightDrivePower, leftDrivePower, rightDrivePower);

        angleStraighten(STRAIGHTEN_P, leftDrivePower, rightDrivePower);
        normalize();
        setMotorPower(flPower, frPower, blPower, brPower);
    }

    private void setMicroscopicStraightPower() {
        leftCurrentTicks = leftEncoder.getCurrentPosition();
        rightCurrentTicks = rightEncoder.getCurrentPosition();

        if (leftCurrentTicks > leftPriorEncoderTarget) {
            setPowerVariables(-MIN_DRIVE_POWER, -MIN_DRIVE_POWER, -MIN_DRIVE_POWER, -MIN_DRIVE_POWER);
        } else {
            setPowerVariables(MIN_DRIVE_POWER, MIN_DRIVE_POWER, MIN_DRIVE_POWER, MIN_DRIVE_POWER);
        }

        opMode.telemetry.addData("leftPriorEncoder", leftPriorEncoderTarget);
        angleStraighten(STRAIGHTEN_P, flPower, frPower);
        normalize();
        setMotorPower(flPower, frPower, blPower, brPower);
    }

    private void setDiagonalPower() {
        leftCurrentTicks = leftEncoder.getCurrentPosition();
        rightCurrentTicks = rightEncoder.getCurrentPosition();
        backCurrentTicks = -backEncoder.getCurrentPosition();

        checkKickIn();
        leftDrivePower = 0;
        rightDrivePower = 0;
        strafePower = 0;
        if (driving) {
            if (strafing) {
                leftDrivePower = opMode.pLeftDiagDriveController.calculatePower(leftCurrentTicks);
                rightDrivePower = opMode.pRightDiagDriveController.calculatePower(rightCurrentTicks);
            } else {
                leftDrivePower = opMode.pLeftDriveController.calculatePower(leftCurrentTicks);
                rightDrivePower = opMode.pRightDriveController.calculatePower(rightCurrentTicks);
            }
        }
        if (strafing) {
            strafePower = opMode.pBackDriveController.calculatePower(backCurrentTicks);
        }

        setPowerVariables(-strafePower + leftDrivePower, strafePower + rightDrivePower, strafePower + leftDrivePower, -strafePower + rightDrivePower);

        if (!driving) {
            strafeCorrect(strafePower, leftStart, rightStart);
        }
        angleStraighten(STRAIGHTEN_P, flPower, frPower);
        normalizeAutos();
        setMotorPower(flPower, frPower, blPower, brPower);

        opMode.telemetry.addData("strafin'", strafing);
        opMode.telemetry.addData("drivin'", driving);
        opMode.telemetry.addData("kickinticks", kickInTicks);
    }

    private void setStrafePower() {
        backCurrentTicks = -backEncoder.getCurrentPosition();

        double calculatedPower = opMode.pBackDriveController.calculatePower(backCurrentTicks);
        opMode.telemetry.addData("calc power", calculatedPower);

        setPowerVariables(-calculatedPower, calculatedPower, calculatedPower, -calculatedPower);

        strafeCorrect(calculatedPower, leftPriorEncoderTarget, rightPriorEncoderTarget);
        angleStraighten(STRAFE_P, calculatedPower, calculatedPower);
        normalize();
        setMotorPower(flPower, frPower, blPower, brPower);
    }

    private void setMicroscopicStrafePower() {
        backCurrentTicks = -backEncoder.getCurrentPosition();
        strafePower = 0.235;
        if (backCurrentTicks > backPriorEncoderTarget) {
            strafePower *= -1;
        }

        setPowerVariables(-strafePower, strafePower, strafePower, -strafePower);

        angleStraighten(STRAIGHTEN_P, flPower, frPower);
        normalize();
        setMotorPower(flPower, frPower, blPower, brPower);
    }
    
    private void setTapePower() {
        if (tapeHandled) {
            double frontDistance = getFrontDistance();
            double power = (getFrontDistance() * DISTANCE_P_COEFFICIENT) + MIN_DRIVE_POWER;
            if (frontDistance < 3.2) {
                power *= -1;
            }
            setPowerVariables(power, power, power, power);
        } else {
            leftDrivePower = opMode.pLeftDriveController.calculatePower(leftEncoder.getCurrentPosition());
            rightDrivePower = opMode.pRightDriveController.calculatePower(rightEncoder.getCurrentPosition());
            setPowerVariables(leftDrivePower, rightDrivePower, leftDrivePower, rightDrivePower);
        }
        correctForTape2();
        angleStraighten(STRAIGHTEN_P, flPower, frPower);
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
            indicator.setPosition(0.52);
        } else {
            indicator.setPosition(0.62);
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

    public void rotateToMicroscopicAngle(double targetAngle){
        double timeOut = 0.7;

        int rightStartingTicks = rightEncoder.getCurrentPosition();
        int leftStartingTicks = leftEncoder.getCurrentPosition();
        int backStartingTicks = backEncoder.getCurrentPosition();

        opMode.pMicroscopicTurnController.setInputRange(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, targetAngle);
        opMode.pMicroscopicTurnController.setSetpoint(targetAngle);
        runtime.reset();

        do {
            double turnPower = Math.abs(opMode.pMicroscopicTurnController.calculatePower(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle));

            if(correctedAngle(opMode.pMicroscopicTurnController.getCurrentError()) > 0){
                setMotorPower(-turnPower, turnPower, -turnPower, turnPower);
            }else {
                setMotorPower(turnPower, -turnPower, turnPower, -turnPower);
            }
            opMode.telemetry.addData("target reached", opMode.pMicroscopicTurnController.reachedTarget());
            opMode.telemetry.update();

        } while (opMode.opModeIsActive() && !opMode.pMicroscopicTurnController.reachedTarget() && runtime.seconds() < timeOut);

        stop();
        priorAngle = targetAngle;
        rightPriorEncoderTarget = rightPriorEncoderTarget - rightStartingTicks + rightEncoder.getCurrentPosition();
        leftPriorEncoderTarget = leftPriorEncoderTarget - leftStartingTicks + leftEncoder.getCurrentPosition();
        backPriorEncoderTarget = backPriorEncoderTarget - backStartingTicks + backEncoder.getCurrentPosition();
    }

    private void checkKickIn() {
        if (!(strafing && driving)) {
            if (secondMove == STRAFE) {
                if (direction == BACKWARD) {
                    if (leftCurrentTicks < kickInTicks) {
                        strafing = true;
                    }
                } else {
                    if (leftCurrentTicks > kickInTicks) {
                        strafing = true;
                    }
                }
            } else {
                if (direction == RIGHT) {
                    if (backCurrentTicks < kickInTicks) {
                        driving = true;
                    }
                } else {
                    if (backCurrentTicks > kickInTicks) {
                        driving = true;
                    }
                }
            }
        }
    }

    private void strafeCorrect(double calculatedPower, int leftTarget, int rightTarget) {
        leftCurrentTicks = leftEncoder.getCurrentPosition();
        rightCurrentTicks = rightEncoder.getCurrentPosition();

        double leftCorrectPower = (leftTarget - leftCurrentTicks) * CORRECTION_COEFFICIENT * Math.abs(calculatedPower) ;
        double rightCorrectPower =  (rightTarget - rightCurrentTicks) * CORRECTION_COEFFICIENT * Math.abs(calculatedPower);
        //modeled after straighten
        flPower += leftCorrectPower;
        frPower += rightCorrectPower;
        blPower += leftCorrectPower;
        brPower += rightCorrectPower;
    }

    private void correctForTape2() {
        double correctPower = tapeCorrectPower(tapeError(LEFT), tapeError(RIGHT));

        flPower += correctPower;
        frPower -= correctPower;
        blPower -= correctPower;
        brPower += correctPower;
    }

    private double tapeError(int sensorSide) {
        if (opMode.alliance == MM_EOCVDetection.RED) {
            if (sensorSide == LEFT) {
                return LEFT_TAPE_RED - leftTapeSensor.red();
            }
            return RIGHT_TAPE_RED - rightTapeSensor.red();
        }
        if (sensorSide == LEFT) {
            return LEFT_TAPE_BLUE - leftTapeSensor.blue();
        }
        return RIGHT_TAPE_BLUE - rightTapeSensor.blue();
    }

    private double tapeCorrectPower(double leftDifference, double rightDifference) {
        double correctPower = 0;
        if (leftDifference > 0 || rightDifference > 0) {
            correctPower = 0.2;
            double comparedError = Math.abs(leftDifference - rightDifference);
            if (comparedError < 40) {
                if (leftDifference < 120) {
                    correctPower = -0.05;
                }
            } else if (comparedError < 80) {
                correctPower = 0.1;
                if (leftDifference > rightDifference) {
                    correctPower = -correctPower;
                }
            } else if (comparedError < 120) {
                correctPower = 0.15;
                if (leftDifference > rightDifference) {
                    correctPower = -correctPower;
                }
            } else if (leftDifference < rightDifference) {
                correctPower = -correctPower;
            }
        } else {
            tapeHandled = true;
        }
        return correctPower;
    }

    private boolean checkColors() {
        if (opMode.alliance == MM_EOCVDetection.BLUE) {
            return (leftTapeSensor.blue() > 200 || rightTapeSensor.blue() > 220);
        } else {
            return (leftTapeSensor.red() > 180 || rightTapeSensor.red() > 200);
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

    public boolean stuckOnCone() {
        return Math.abs(leftEncoder.getCurrentPosition() - leftPriorEncoderTarget) > 4000;
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

    public boolean correctForCone() {
        leftCurrentTicks = leftEncoder.getCurrentPosition();
        rightCurrentTicks = rightEncoder.getCurrentPosition();
        boolean corrected = true;
        if (getFrontDistance() > 4.2) {
            drive(FORWARD);
            corrected = false;
        } else if (getFrontDistance() < 3.2) {
            drive(BACKWARD);
            corrected = false;
        }

        runtime.reset();
        while (opMode.opModeIsActive() && !corrected && runtime.seconds() < 3) {
            double distance = getFrontDistance();
            corrected = (distance < 4.2 && distance > 2.8);
        }
        leftPriorEncoderTarget = leftPriorEncoderTarget + (leftEncoder.getCurrentPosition() - leftCurrentTicks);
        rightPriorEncoderTarget = rightPriorEncoderTarget + (rightEncoder.getCurrentPosition() - rightCurrentTicks);
        stop();
        return corrected;
    }

    public void correctForTape() {
        //if during the drive, strafe with a P coefficent maybe or just add powers somehow, you will have to change the prior encoders tho
        backCurrentTicks = backEncoder.getCurrentPosition();
        leftCurrentTicks = leftEncoder.getCurrentPosition();
        rightCurrentTicks = rightEncoder.getCurrentPosition();

        boolean corrected = true;
        if (opMode.alliance == MM_OpMode.BLUE) {
            int direction = 0;
            if (leftTapeSensor.blue() < 300) {
                strafe(RIGHT);
                corrected = false;
                direction = RIGHT;
            } else if (rightTapeSensor.blue() < 320) {
                strafe(LEFT);
                corrected = false;
                direction = LEFT;
            }
            runtime.reset();
            while (opMode.opModeIsActive() && !corrected && runtime.seconds() < 2.5) {
                corrected = (rightTapeSensor.blue() > 290 && leftTapeSensor.blue() > 310);
                if (runtime.seconds() > 1.25) {
                    strafe(-direction);
                }
            }
        } else {
            int direction = 0;
            if (leftTapeSensor.red() < 280) { //albany 200
                strafe(RIGHT);
                corrected = false;
                direction = RIGHT;
            } else if (rightTapeSensor.red() < 300) { //albany 230
                strafe(LEFT);
                corrected = false;
                direction = LEFT;
            }
            runtime.reset();
            while (opMode.opModeIsActive() && !corrected && runtime.seconds() < 2.5) {
                corrected = (rightTapeSensor.red() > 270 && leftTapeSensor.red() > 290); //albany 190, 220
                opMode.telemetry.addData("left tape", leftTapeSensor.red());
                opMode.telemetry.addData("right tape", rightTapeSensor.red());
                opMode.telemetry.update();
                if (runtime.seconds() > 1.25) {
                    strafe(-direction);
                }
            }
        }

        backPriorEncoderTarget = backPriorEncoderTarget + (backEncoder.getCurrentPosition() - backCurrentTicks);
        leftPriorEncoderTarget = leftPriorEncoderTarget + (leftEncoder.getCurrentPosition() - leftCurrentTicks);
        rightPriorEncoderTarget = rightPriorEncoderTarget + (rightEncoder.getCurrentPosition() - rightCurrentTicks);
        stop();
    }

    public void strafe(int direction) {
        //right is negative
        double power = -MIN_STRAFE_POWER * direction;
        flPower = power;
        frPower = -power + (0.045 * -direction);
        blPower = -power + (0.045 * -direction);
        brPower = power;
        //angleStraighten(STRAIGHTEN_P, power, power);
        setMotorPower(flPower, frPower, blPower, brPower);
    }

    public void drive(int direction) {
        double power = 0.20 * direction;
        setMotorPower(power, power, power, power);
    }

    private void setPowerVariables(double flPower, double frPower, double blPower, double brPower) {
        this.flPower = flPower;
        this.frPower = frPower;
        this.blPower = blPower;
        this.brPower = brPower;
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
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }
    }

    private void normalizeAutos() {
        if (strafing && driving) {
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

    public void resetEncoders() {
        switchEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        switchEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backPriorEncoderTarget = 0;
        leftPriorEncoderTarget = 0;
        rightPriorEncoderTarget = 0;
    }

    private void init() {
        frontLeftDrive = opMode.hardwareMap.get(DcMotorEx.class, "FLMotor");
        frontRightDrive = opMode.hardwareMap.get(DcMotorEx.class, "FRMotor");
        backLeftDrive = opMode.hardwareMap.get(DcMotorEx.class, "BLMotor");
        backRightDrive = opMode.hardwareMap.get(DcMotorEx.class, "BRMotor");

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

        distance = opMode.hardwareMap.get(DistanceSensor.class, "coneSensor");
        rightTapeSensor = opMode.hardwareMap.get(ColorSensor.class, "rightTapeSensor");
        leftTapeSensor = opMode.hardwareMap.get(ColorSensor.class, "leftTapeSensor");
    }

    private void initServos(){
        if(opMode.getClass() == MM_TeleOp.class){
            indicator = opMode.hardwareMap.get(Servo.class, "floppyServo");
            leftOdomLift = opMode.hardwareMap.get(Servo.class,"leftOdometryLift");
            rightOdomLift = opMode.hardwareMap.get(Servo.class,"rightOdometryLift");
            backOdomLift = opMode.hardwareMap.get(Servo.class,"backOdometryLift");

            leftOdomLift.setPosition(1);
            rightOdomLift.setPosition(0);
            backOdomLift.setPosition(1);
            indicator.setPosition(1);
        } else {
            scorer = opMode.hardwareMap.get(Servo.class, "floppyServo");
            scorer.setPosition(1);
        }
    }

    public void initializeGyroAndEncoders() {
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();
        gyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu  = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyroParameters);
        resetEncoders();
    }

    public void autoScore() {
        scorer.setPosition(0);
        opMode.waitSeconds(0.65);
        scorer.setPosition(0.15);
        opMode.waitSeconds(0.2);
        scorer.setPosition(0.62);
    }

    public void getScorerOutOfTheWay() {
        scorer.setPosition(0.62);
    }

    public double getFrontDistance() {
        return distance.getDistance(DistanceUnit.INCH);
    }
}