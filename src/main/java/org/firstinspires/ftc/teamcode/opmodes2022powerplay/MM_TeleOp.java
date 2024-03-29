package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp", group="MM")
public class MM_TeleOp extends MM_OpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("Please wait for Initialization");
        telemetry.update();
        robot.init();
        robot.lift.chomper.choke();
        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updateController();
            robot.drivetrain.driveWithSticks();
            robot.lift.driverControl();
            telemetry.update();
        }
    }
}
