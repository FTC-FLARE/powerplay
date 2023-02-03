package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class MM_OpMode extends LinearOpMode {
    public MM_Robot robot = new MM_Robot(this);
    public MM_P_Controller pTurnController = new MM_P_Controller(this, 1, TURN_P_COEFFICIENT);
    public MM_P_Controller pMicroscopicTurnController = new MM_P_Controller(this, 35, TURN_P_COEFFICIENT);
    public MM_P_Controller pLeftDriveController = new MM_P_Controller(this,2.2, DRIVE_P_COEFFICIENT);
    public MM_P_Controller pRightDriveController = new MM_P_Controller(this,2.2, DRIVE_P_COEFFICIENT);
    public MM_P_Controller pLeftDiagDriveController = new MM_P_Controller(this,1.6, DRIVE_P_COEFFICIENT);
    public MM_P_Controller pRightDiagDriveController = new MM_P_Controller(this,1.6, DRIVE_P_COEFFICIENT);
    public MM_P_Controller pBackDriveController = new MM_P_Controller(this,2.6, STRAFE_P_COEFFICIENT);

    public Gamepad gamepad1Current = new Gamepad();
    public Gamepad gamepad1Prior = new Gamepad();
    public Gamepad gamepad2Current = new Gamepad();
    public Gamepad gamepad2Prior = new Gamepad();

    static final double DRIVE_P_COEFFICIENT = 0.00001598;
    static final double STRAFE_P_COEFFICIENT = 0.0000555;
    static final double TURN_P_COEFFICIENT = .015;

    static final int GAMEPAD1 = 0;
    static final int GAMEPAD2 = 1;
    static final int BLUE = 1;
    static final int RED = 0;

    public int alliance = RED;

    private final ElapsedTime runtime = new ElapsedTime();

    public boolean aPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            return gamepad1Current.a && !gamepad1Prior.a;
        } else {
            return gamepad2Current.a && !gamepad2Prior.a;
        }
    }

    public boolean bPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            return gamepad1Current.b && !gamepad1Prior.b;
        } else {
            return gamepad2Current.b && !gamepad2Prior.b;
        }
    }

    public boolean yPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            return gamepad1Current.y && !gamepad1Prior.y;
        } else {
            return gamepad2Current.y && !gamepad2Prior.y;
        }
    }

    public boolean xPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            return gamepad1Current.x && !gamepad1Prior.x;
        } else {
            return gamepad2Current.x && !gamepad2Prior.x;
        }
    }

    public boolean dpadDownPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            return gamepad1Current.dpad_down && !gamepad1Prior.dpad_down;
        } else {
            return gamepad2Current.dpad_down && !gamepad2Prior.dpad_down;
        }
    }

    public boolean dpadLeftPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            return gamepad1Current.dpad_left && !gamepad1Prior.dpad_left;
        } else {
            return gamepad2Current.dpad_left && !gamepad2Prior.dpad_left;
        }
    }

    public boolean dpadRightPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            return gamepad1Current.dpad_right && !gamepad1Prior.dpad_right;
        } else {
            return gamepad2Current.dpad_right && !gamepad2Prior.dpad_right;
        }
    }

    public boolean dpadUpPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            return gamepad1Current.dpad_up && !gamepad1Prior.dpad_up;
        } else {
            return gamepad2Current.dpad_up && !gamepad2Prior.dpad_up;
        }
    }

    public boolean leftJoystickPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            return gamepad1Current.left_stick_button && !gamepad1Prior.left_stick_button;
        } else {
            return gamepad2Current.left_stick_button && !gamepad2Prior.left_stick_button;
        }
    }

    public boolean leftBumperPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            return gamepad1Current.left_bumper && !gamepad1Prior.left_bumper;
        } else {
            return gamepad2Current.left_bumper && !gamepad2Prior.left_bumper;
        }
    }

    public boolean rightBumperPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            return gamepad1Current.right_bumper && !gamepad1Prior.right_bumper;
        } else {
            return gamepad2Current.right_bumper && !gamepad2Prior.right_bumper;
        }
    }

    public boolean leftStickYDownPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            return -gamepad1Current.right_stick_y > 0.1 && !(-gamepad1Prior.right_stick_y > 0.1);
        } else {
            return -gamepad2Current.right_stick_y > 0.1 && !(-gamepad2Prior.right_stick_y > 0.1);
        }
    }

    public boolean leftStickYUpPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            return -gamepad1Current.right_stick_y < 0.1 && !(-gamepad1Prior.right_stick_y < 0.1);
        } else {
            return -gamepad2Current.right_stick_y < -0.1 && !(-gamepad2Prior.right_stick_y < -0.1);
        }
    }

    public void waitSeconds(double seconds) {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < seconds){
        }
    }
}