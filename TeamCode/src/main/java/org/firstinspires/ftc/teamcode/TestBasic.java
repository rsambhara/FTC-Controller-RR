/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.math.RoundingMode;
import java.text.DecimalFormat;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Test motor", group="Linear Opmode")
//@Disabled
public class TestBasic extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft = null;
    //private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    //private DcMotor backRight = null;
    private DcMotor elevator = null;
    private DcMotor claw = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        //frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        //backRight = hardwareMap.get(DcMotor.class, "backright");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        claw = hardwareMap.get(DcMotor.class, "claw");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            //double power = gamepad1.right_stick_y * gamepad1.right_stick_y;
//            double leftx = gamepad1.right_stick_x * 0.4;
            double leftx = gamepad1.left_stick_x * 0.5;
            double lefty = gamepad1.left_stick_y * 0.5;
            double rightx = gamepad1.right_stick_x * 0.5;
           double righty = gamepad1.right_stick_y * 0.5;
            double elePower = gamepad1.left_stick_y * 0.8;

            // Elevator and Claw

            double clawx = gamepad2.left_stick_x * 0.5;
            double elevatory = gamepad2.right_stick_y * 0.5;

            DecimalFormat df = new DecimalFormat("#.###");
            df.setRoundingMode(RoundingMode.FLOOR);
            leftx = Double.parseDouble(df.format(leftx));
            lefty = Double.parseDouble(df.format(lefty));
            rightx = Double.parseDouble(df.format(rightx));
            righty = Double.parseDouble(df.format(righty));
            // elevator and claw movement
            clawx = Double.parseDouble(df.format(clawx));
            elevatory = Double.parseDouble(df.format(elevatory));

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            frontLeft.setPower(-leftx);
            //frontRight.setPower(rightx);
            //backRight.setPower(lefty);
            backLeft.setPower(-righty);

            // elevator and claw
            elevator.setPower(elevatory);
            claw.setPower(clawx);



            // Send calculated power to wheels

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Rex Motor power: ", power);
            telemetry.addData("Front Leftx: ", leftx);
            telemetry.addData("Front Rightx: ", rightx);
            telemetry.addData("Back Lefty: ", lefty);
            telemetry.addData("Back Righty", righty);
            telemetry.addData("Claw Movement", clawx);
            telemetry.addData("Elevator Y", elevatory);
            //telemetry.speak("Give more MORE POWER!!!!!");
            telemetry.update();
        }
    }
}
