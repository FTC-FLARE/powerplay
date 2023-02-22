/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
general comments
as you get closer to glass wall it increases in accuracy
40 in goes between 39 and 41.5
30 in goes betwewen 29.5 and 31
10 in goes between 9.8 and 10.2

glass wall reading greater inches than a solid background
ex. 29 with solid, 32 without. same distances
 */
@Config
@TeleOp(name = "Ultrasonic Distance", group = "Test")
public class ultrasonicDistance extends LinearOpMode {

    private AnalogInput sensorRange;
    private DistanceSensor distanceSensor;
    public static int FILTERSIZE = 20;

    private double sum = 0;
    private double avgInches = 0;
    private int loopTracker = 0;
    private double lastTerms[] = new double[FILTERSIZE + 1];

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // you can use this as a regular DistanceSensor.
        sensorRange = hardwareMap.get(AnalogInput.class, "sonarFront");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "coneSensor");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
       // Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
        //change to constant
        while (loopTracker < FILTERSIZE+1) {
            lastTerms[loopTracker] = 0;
            loopTracker += 1;
        }
        loopTracker = 1;
        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            double inches = MM_Util.voltageToInches(sensorRange.getVoltage());
            sum += inches;
            sum -= lastTerms[loopTracker];
            lastTerms[loopTracker] = inches;

            avgInches = sum/getCurrentReading();

            telemetry.addData("Inches", inches);
            telemetry.addData("avgInches", avgInches);
            telemetry.addData("Loop", loopTracker);
            telemetry.addData("rev 2m", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
            handleloopTracker();
        }
    }

    double getCurrentReading() {
        if (lastTerms[FILTERSIZE] == 0) {
            return loopTracker;
        } else {
            return FILTERSIZE;
        }
    }

    void handleloopTracker() {
        loopTracker += 1;
        if (loopTracker == FILTERSIZE + 1) {
            loopTracker = 1;
        }
    }
}