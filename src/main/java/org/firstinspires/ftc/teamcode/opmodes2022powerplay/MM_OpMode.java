package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class MM_OpMode extends LinearOpMode {
    public MM_Robot robot = new MM_Robot(this);

    public Gamepad gamepad1Current = new Gamepad();
    public Gamepad gamepad1Prior = new Gamepad();
    public Gamepad gamepad2Current = new Gamepad();
    public Gamepad gamepad2Prior = new Gamepad();

    final int GAMEPAD1 = 0;
    final int GAMEPAD2 = 1;

    final int COLLECT = 0;
    final int GROUND = 1;
    final int LOW = 2;
    final int MEDIUM = 3;
    final int HIGH = 4;

    public boolean aPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            if (gamepad1Current.a && !gamepad1Prior.a) {
                return true;
            }
            return false;
        } else {
            if (gamepad2Current.a && !gamepad2Prior.a) {
                return true;
            }
            return false;
        }
    }
    public boolean bPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            if (gamepad1Current.b && !gamepad1Prior.b) {
                return true;
            }
            return false;
        } else {
            if (gamepad2Current.b && !gamepad2Prior.b) {
                return true;
            }
            return false;
        }
    }

    public boolean yPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            if (gamepad1Current.y && !gamepad1Prior.y) {
                return true;
            }
            return false;
        } else {
            if (gamepad2Current.y && !gamepad2Prior.y) {
                return true;
            }
            return false;
        }
    }

    public boolean xPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            if (gamepad1Current.x && !gamepad1Prior.x) {
                return true;
            }
            return false;
        } else {
            if (gamepad2Current.x && !gamepad2Prior.x) {
                return true;
            }
            return false;
        }
    }

    public boolean dpadDownPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            if (gamepad1Current.dpad_down && !gamepad1Prior.dpad_down) {
                return true;
            }
            return false;
        } else {
            if (gamepad2Current.dpad_down && !gamepad2Prior.dpad_down) {
                return true;
            }
            return false;
        }
    }

    public boolean dpadLeftPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            if (gamepad1Current.dpad_left && !gamepad1Prior.dpad_left) {
                return true;
            }
            return false;
        } else {
            if (gamepad2Current.dpad_left && !gamepad2Prior.dpad_left) {
                return true;
            }
            return false;
        }
    }

    public boolean dpadRightPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            if (gamepad1Current.dpad_right && !gamepad1Prior.dpad_right) {
                return true;
            }
            return false;
        } else {
            if (gamepad2Current.dpad_right && !gamepad2Prior.dpad_right) {
                return true;
            }
            return false;
        }
    }

    public boolean leftBumperPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            if (gamepad1Current.left_bumper && !gamepad1Prior.left_bumper) {
                return true;
            }
            return false;
        } else {
            if (gamepad2Current.left_bumper && !gamepad2Prior.left_bumper) {
                return true;
            }
            return false;
        }
    }

    public boolean rightBumperPressed(int gamepad) {
        if (gamepad == GAMEPAD1) {
            if (gamepad1Current.right_bumper && !gamepad1Prior.right_bumper) {
            return true;
            }
            return false;
        } else {
            if (gamepad2Current.right_bumper && !gamepad2Prior.right_bumper) {
                return true;
            }
        }
        return false;
    }
}

