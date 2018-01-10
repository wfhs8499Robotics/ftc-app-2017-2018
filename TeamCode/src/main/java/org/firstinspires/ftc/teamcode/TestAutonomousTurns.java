/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/*
 *
 * This is a LinearOpMode that executes turnWheels routines for turning of the
 * robot in autonomous mode for testing purposes.
 *
 */
@Autonomous(name = "Test Autonomous Turns", group = "Autonomous OpMode")
public class TestAutonomousTurns extends LinearOpMode {
    private TurnWheels turnWheels = new TurnWheels();

    @Override
    public void runOpMode() throws InterruptedException {

        turnWheels.init(hardwareMap);
        telemetry.addData("Right Bumper", "Right 90");
        telemetry.addData("Left Bumper", "Left 90");
        telemetry.addData("Right Trigger", "Right 33");
        telemetry.addData("Left Trigger", "Left 33");
        telemetry.addData("X", "Right 180");
        telemetry.addData("A", "Left 180");
        telemetry.addData("Y", "Forward straight 12 inches");
        telemetry.addData("B", "Reverse straight 12 inches");
        telemetry.update();

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            if(gamepad1.right_bumper){
                turnWheels.right90();
            }
            if(gamepad1.left_bumper){
                turnWheels.left90();
            }
            if(gamepad1.right_trigger > 0){
                turnWheels.right33();
            }
            if(gamepad1.left_trigger > 0){
                turnWheels.left33();
            }
            if(gamepad1.x){
                turnWheels.right180();
            }
            if(gamepad1.a){
                turnWheels.left180();
            }
            if(gamepad1.y){
                turnWheels.encoderDrive(.3,12,12,10);
            }
            if(gamepad1.b){
                turnWheels.encoderDrive(.3,-12,-12,10);
            }

            // send the info back to driver station using telemetry function.
            telemetry.addData("Operation", "Complete");
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
