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
    private DistanceSensor detectorOfTheScaryYellowJunctions = null;


    private final ElapsedTime runtime = new ElapsedTime();

    public static final double MIN_STRAFE_POWER = 0.4;
    private static final double SECONDS_PER_DEGREE = 0.025;//??
    private static final double STRAIGHTEN_P = .0840; //.0780
    private static final double STRAFE_P = .089;
    private static final double CORRECTION_COEFFICIENT = 0.000055; //Gain per tick
    private static final double DISTANCE_P_COEFFICIENT = 0.0345;
    private static final double TAPE_P_COEFFICIENT = 0.000677; // .000377
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

    private static final int TAPE_BLUE = 360;
    private static final int TAPE_RED = 245;  // Need correct value
    private final int ALLIANCE_TAPE_TARGET;  // Set in constructor
    private final int MIN_TAPE_TARGET;  // Set in constructor
    private static final int TAPE_TOLERANCE_BLUE = 110;
    private static final int TAPE_TOLERANCE_RED = 70;
    private static final double MAX_TAPE_POWER = 0.45;
    private static final double STACK_DISTANCE = 5.2;
    private static final double STACK_DISTANCE_TOLERANCE = 0.2;

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
    private boolean distanceKickOut = false;
    private boolean distanceKickedOut = false;
    private boolean strafing = false;
    private boolean driving = false;

    private double leftTapeVal = 0;
    private double rightTapeVal = 0;
    private boolean bothWereOnTape = false;

    public MM_Drivetrain(MM_OpMode opMode) {
        this.opMode = opMode;
        init();

        if (opMode.alliance == MM_OpMode.RED){
            ALLIANCE_TAPE_TARGET = TAPE_RED;
            MIN_TAPE_TARGET = ALLIANCE_TAPE_TARGET - TAPE_TOLERANCE_RED;
        } else {
            ALLIANCE_TAPE_TARGET = TAPE_BLUE;
            MIN_TAPE_TARGET = ALLIANCE_TAPE_TARGET - TAPE_TOLERANCE_BLUE;
        }

    }

    public void driveInches(double inches) {
        prepareToDrive(inches, false);
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 5 && !reachedPositionDrive()) {
            opMode.telemetry.addData("inches target", inches);
            opMode.telemetry.update();
        }
    }

    public void microscopicDriveInches(double inches) {
        prepareToDrive(inches,false);
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 1.5 && !reachedPositionMicroscopicDrive()) {
            opMode.telemetry.addData("inches target", inches);
            opMode.telemetry.update();
        }
    }

    public void diagonalDriveInches(double forwardInches, double strafeInches) {
        diagonalDriveInches(forwardInches, strafeInches, 100,0, false);
    }

    public void diagonalDriveInches(double forwardInches, double strafeInches, int move, int percentKickIn, boolean colorKickOut) {
        prepareToDiagonalDrive(forwardInches, strafeInches, percentKickIn, move, colorKickOut);
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

    public void prepareToDrive(double inches, boolean distanceKickOut) {
        this.distanceKickOut = distanceKickOut;
        distanceKickedOut = false;
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
            kickInTicks = ((kickInPercent) * (backTargetTicks - backPriorEncoderTarget) + backPriorEncoderTarget)/100;
            strafing = true;
            driving = false;
        } else if (secondMove == STRAFE) {
            if (driveInches < 0) {
                direction = BACKWARD;
            } else {
                direction = FORWARD;
            }
            kickInTicks = ((kickInPercent) * (leftTargetTicks - leftPriorEncoderTarget) + leftPriorEncoderTarget)/100;
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
        bothWereOnTape = false;
    }

    public boolean reachedPositionDrive() { //this also sets the motor power
        if (!distanceKickedOut) {
            setStraightPower();
        }

        if (distanceKickOut) {
            if (withinJunctionRange() || distanceKickedOut) {
                stop();
                distanceKickedOut = true;
                return true;
            }
        }

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

        if (driving) {
            if (colorKickOut) {
                return checkColors();
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

    private void setStraightPower() {
        leftCurrentTicks = leftEncoder.getCurrentPosition();
        rightCurrentTicks = rightEncoder.getCurrentPosition();

        leftDrivePower = opMode.pLeftDriveController.calculatePower(leftCurrentTicks);
        rightDrivePower = opMode.pRightDriveController.calculatePower(rightCurrentTicks);

        setPowerVariables(leftDrivePower, rightDrivePower, leftDrivePower, rightDrivePower);

        angleStraighten(STRAIGHTEN_P, leftDrivePower, rightDrivePower);
        normalize(1);
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
        normalize(1);
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
        normalize(1);
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
        normalize(1);
        setMotorPower(flPower, frPower, blPower, brPower);
    }
    
    public void driveWithSticks() {
        double drive = -opMode.gamepad1.left_stick_y;
        double turn = opMode.gamepad1.right_stick_x;
        double strafe = opMode.gamepad1.left_stick_x;
        opMode.telemetry.addData("dis", getFrontDistance());
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

        normalize(1);
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

        opMode.telemetry.addData("left color", leftTapeSensor.red());
        opMode.telemetry.addData("right color", rightTapeSensor.red());

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

    public boolean followTapeToStack() {
        bothWereOnTape = false;

        runtime.reset();
        while (opMode.opModeIsActive() && !reachedPositionTapeDrive()) {
            opMode.telemetry.update();
            if (runtime.seconds() > 5) return false;
        }

        return true;
    }

    public boolean reachedPositionTapeDrive() {
        double frontDistance = getFrontDistance();
        double distanceError = frontDistance - STACK_DISTANCE;
        opMode.telemetry.addData("Distance", frontDistance);

        if (Math.abs(distanceError) <= STACK_DISTANCE_TOLERANCE) {
            stop();
            opMode.telemetry.addLine("*** At stopping distance");
            return true;
        }
        setTapePower(distanceError);
        return false;
    }

    private void setTapePower(double distanceError) {
/*        if (bothWereOnTape) {  // set base power according to distance from stack
            double power = Math.max(MIN_DRIVE_POWER, distanceError * DISTANCE_P_COEFFICIENT);
            if (distanceError < 0) {
                opMode.telemetry.addLine("*** Too close to stack");
                power *= -1;
            }
            setPowerVariables(power, power, power, power);
        } else {  // set base power according to P-Controllers & encoder targets
            leftDrivePower = (opMode.pLeftDriveController.calculatePower(leftEncoder.getCurrentPosition()))/1.3;
            rightDrivePower = (opMode.pRightDriveController.calculatePower(rightEncoder.getCurrentPosition()))/1.3;
            setPowerVariables(leftDrivePower, rightDrivePower, leftDrivePower, rightDrivePower);
        }*/
        double power = Math.max(MIN_DRIVE_POWER, distanceError * DISTANCE_P_COEFFICIENT);
        if (distanceError < 0) {
            opMode.telemetry.addLine("*** Too close to stack");
            power *= -1;
        }
        setPowerVariables(power, power, power, power);
        tapeCorrect();
        angleStraighten(STRAIGHTEN_P, flPower, frPower);
        normalize(MAX_TAPE_POWER);
        setMotorPower(flPower, frPower, blPower, brPower);
    }

    private void tapeCorrect() {
        double leftTapeValue = tapeValue(leftTapeSensor);
        double rightTapeValue = tapeValue(rightTapeSensor);
        double correctPower = (rightTapeValue - leftTapeValue) * TAPE_P_COEFFICIENT;

        if (leftTapeValue >= MIN_TAPE_TARGET && rightTapeValue >= MIN_TAPE_TARGET){
            bothWereOnTape = true;
        }

        flPower += correctPower;
        frPower -= correctPower;
        blPower -= correctPower;
        brPower += correctPower;

        opMode.telemetry.addData("Left Blueness", leftTapeValue);
        opMode.telemetry.addData("Right Blueness", rightTapeValue);
        opMode.telemetry.addData("Tape Power", "%.2f", correctPower);
    }

    public int tapeValue(ColorSensor colorSensor) {
        if (opMode.alliance == MM_EOCVDetection.RED) {
            return colorSensor.red();
        } else{
            return colorSensor.blue();
        }
    }

    private boolean checkColors() {
        if (opMode.alliance == MM_EOCVDetection.BLUE) {
            return (leftTapeSensor.blue() > MIN_TAPE_TARGET || rightTapeSensor.blue() > MIN_TAPE_TARGET);
        } else {
            return (leftTapeSensor.red() > MIN_TAPE_TARGET || rightTapeSensor.red() > MIN_TAPE_TARGET);
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
        return detectorOfTheScaryYellowJunctions.getDistance(DistanceUnit.INCH) < 5;
    }

    public boolean alignedWithJunction() {
        if (!withinJunctionRange()) {
            runtime.reset();
            double startingDistance = detectorOfTheScaryYellowJunctions.getDistance(DistanceUnit.INCH);
            double currentDistance = startingDistance;

            leftPriorEncoderTarget = leftEncoder.getCurrentPosition();
            rightPriorEncoderTarget = rightEncoder.getCurrentPosition();

            while (opMode.opModeIsActive() && currentDistance > 3) {
                if(runtime.seconds() > 5){
                    stop();
                    return true;
                }
                strafe(LEFT);
//                normalize(MIN_STRAFE_POWER);
                currentDistance = detectorOfTheScaryYellowJunctions.getDistance(DistanceUnit.INCH);
            }
            stop();
            return false;
        }
        return true;
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
        frPower = -power;
        blPower = -power;
        brPower = power;

        strafeCorrect(power, leftPriorEncoderTarget, rightPriorEncoderTarget);

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

    private void normalize(double maxPower) {
        double max = Math.max(Math.abs(flPower), Math.abs(frPower));
        max = Math.max(max, Math.abs(blPower));
        max =  Math.max(max, Math.abs(brPower));

        if (max > maxPower) {
            flPower = (flPower * maxPower)/max;
            frPower = (frPower * maxPower)/max;
            blPower = (blPower * maxPower)/max;
            brPower = (brPower * maxPower)/max;
//            flPower /= max;
//            frPower /= max;
//            blPower /= max;
//            brPower /= max;
        }
    }

    private void normalizeAutos() {
        if (strafing && driving) {
            flPower /= 1.25;
            frPower /= 1.25;
            blPower /= 1.25;
            brPower /= 1.25;
        }
        normalize(1);
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
        detectorOfTheScaryYellowJunctions = opMode.hardwareMap.get(DistanceSensor.class, "detectorOfTheScaryYellowJunctions");

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
            indicator.setPosition(0);
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
        scorer.setPosition(0.6);
        strafeInches(-11.3);
        scorer.setPosition(0.45);
        opMode.waitSeconds(0.25);
        scorer.setPosition(0.22);
        opMode.waitSeconds(0.65);
        scorer.setPosition(0.35);
        opMode.waitSeconds(0.2);
        scorer.setPosition(0.6);
        opMode.waitSeconds(0.2);
        scorer.setPosition(0);
    }

    public void getScorerOutOfTheWay() {
        scorer.setPosition(0);
    }

    public double getFrontDistance() {
        return distance.getDistance(DistanceUnit.INCH);
    }

    public double getJunctionDistance() {
        return detectorOfTheScaryYellowJunctions.getDistance(DistanceUnit.INCH);
    }

    public boolean withinJunctionRange() {
        return getJunctionDistance() < 9;
    }

    public int tempGetLeftBlue() {
        return leftTapeSensor.red();
    }

    public int tempGetRightBlue() {
        return rightTapeSensor.red();
    }

}