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

package org.firstinspires.ftc.teamcode.voyagers.positioning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.voyagers.util.Beam;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Drive Trace", group = "Linear Opmode")
public class TraceOp extends LinearOpMode
{
	private DcMotor leftDrive;
	private DcMotor rightDrive;
	private DcMotor leftFrontDrive;
	private DcMotor rightFrontDrive;

	@Override
	public void runOpMode()
	{
		telemetry.update();

		Beam.init(this.telemetry);

		// Initialize the hardware variables. Note that the strings used here as parameters
		// to 'get' must correspond to the names assigned during the robot configuration
		// step (using the FTC Robot Controller app on the phone).
		leftDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
		rightDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
		leftFrontDrive = hardwareMap.get(DcMotor.class, "left_forward_drive");
		rightFrontDrive = hardwareMap.get(DcMotor.class, "right_forward_drive");

		// Most robots need the motor on one side to be reversed to drive forward
		// Reverse the motor that runs backwards when connected directly to the battery
		leftDrive.setDirection(DcMotor.Direction.FORWARD);
		rightDrive.setDirection(DcMotor.Direction.REVERSE);
		leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
		rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);

		leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		// Wait for the game to start (driver presses PLAY)
		waitForStart();

		// run until the end of the match (driver presses STOP)
		while (opModeIsActive())
		{
			// driver 1 control
			double cDrive = gamepad1.dpad_up ? -0.1 : gamepad1.dpad_down ? 0.1 : 0;
			double cTurn = gamepad1.dpad_right ? 0.1 : gamepad1.dpad_left ? -0.1 : 0;
			boolean cDriveTurboMode = gamepad1.right_trigger > 0.5;

			double scale = cDriveTurboMode ? -2 : -1;
			double drive = scale * 0.45 * -cDrive;
			double turn = scale * 0.7 * cTurn;

			double leftPower = Range.clip(drive + turn, -1.0, 1.0);
			double rightPower = Range.clip(drive - turn, -1.0, 1.0);
			double leftFPower = Range.clip(-drive - 0.3 * turn, -1.0, 1.0);
			double rightFPower = Range.clip(-drive + 0.3 * turn, -1.0, 1.0);
			leftDrive.setPower(leftPower);
			rightDrive.setPower(rightPower);
			leftFrontDrive.setPower(leftFPower);
			rightFrontDrive.setPower(rightFPower);

			Beam.it("LF", leftFrontDrive.getCurrentPosition());
			Beam.it("LB", leftDrive.getCurrentPosition());
			Beam.it("RF", rightFrontDrive.getCurrentPosition());
			Beam.it("RB", rightDrive.getCurrentPosition());
			Beam.flush();
		}
	}
}
