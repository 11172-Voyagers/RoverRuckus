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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
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

@TeleOp(name = "Driver OP", group = "Linear Opmode")
public class DriverOP extends LinearOpMode
{
	// Declare OpMode members.
	private ElapsedTime runtime = new ElapsedTime();
	private DcMotor leftDrive;
	private DcMotor rightDrive;
	private DcMotor leftFrontDrive;
	private DcMotor rightFrontDrive;
	private DcMotor leftLinear;
	private DcMotor rightLinear;
	private DcMotor leftArm;
	private DcMotor rightArm;

	private CRServo leftGrip;
	private CRServo rightGrip;

	private CRServo leftAntiSlip;
	private CRServo rightAntiSlip;

	private CRServo combine;
	private CRServo leftArmLinear;
	private CRServo rightArmLinear;

	@Override
	public void runOpMode()
	{
		telemetry.addData("Status", "Initialized");
		telemetry.update();

		Beam.init(this.telemetry);

		// Initialize the hardware variables. Note that the strings used here as parameters
		// to 'get' must correspond to the names assigned during the robot configuration
		// step (using the FTC Robot Controller app on the phone).
		leftDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
		rightDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
		leftFrontDrive = hardwareMap.get(DcMotor.class, "left_forward_drive");
		rightFrontDrive = hardwareMap.get(DcMotor.class, "right_forward_drive");
		leftLinear = hardwareMap.get(DcMotor.class, "leftLinear");
		rightLinear = hardwareMap.get(DcMotor.class, "rightLinear");
		leftArm = hardwareMap.get(DcMotor.class, "leftArm");
		rightArm = hardwareMap.get(DcMotor.class, "rightArm");

		leftGrip = hardwareMap.get(CRServo.class, "leftGrip");
		rightGrip = hardwareMap.get(CRServo.class, "rightGrip");
		combine = hardwareMap.get(CRServo.class, "combine");

		leftAntiSlip = hardwareMap.get(CRServo.class, "leftAntiSlip");
		rightAntiSlip = hardwareMap.get(CRServo.class, "rightAntiSlip");

		leftArmLinear = hardwareMap.get(CRServo.class, "leftArmLinear");
		rightArmLinear = hardwareMap.get(CRServo.class, "rightArmLinear");

		// Most robots need the motor on one side to be reversed to drive forward
		// Reverse the motor that runs backwards when connected directly to the battery
		leftDrive.setDirection(DcMotor.Direction.FORWARD);
		rightDrive.setDirection(DcMotor.Direction.REVERSE);
		leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
		rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);

		leftLinear.setDirection(DcMotor.Direction.FORWARD);
		rightLinear.setDirection(DcMotor.Direction.REVERSE);
		leftArm.setDirection(DcMotor.Direction.FORWARD);
		rightArm.setDirection(DcMotor.Direction.REVERSE);
		leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		leftArmLinear.setDirection(CRServo.Direction.REVERSE);
		rightArmLinear.setDirection(CRServo.Direction.FORWARD);
		leftAntiSlip.setDirection(CRServo.Direction.REVERSE);
		rightAntiSlip.setDirection(CRServo.Direction.FORWARD);
		combine.setDirection(CRServo.Direction.FORWARD);

		leftArmLinear.setPower(0);
		rightArmLinear.setPower(0);

		// Wait for the game to start (driver presses PLAY)
		waitForStart();
		runtime.reset();

		// run until the end of the match (driver presses STOP)
		while (opModeIsActive())
		{
			// driver 1 control
			double cDrive = gamepad1.left_stick_y;
			double cTurn = gamepad1.right_stick_x;
			boolean cDriveTurboMode = gamepad1.right_trigger > 0.5;
			boolean cLinearDown = gamepad1.dpad_up;
			boolean cLinearUp = gamepad1.dpad_down;
			boolean cAntiSlipIn = gamepad1.dpad_left;
			boolean cAntiSlipOut = gamepad1.dpad_right;

			// driver 2 control
			double cArm = gamepad2.left_stick_y;
			boolean cArmTurboMode = gamepad2.y;
			double cRightGripOpen = gamepad2.left_trigger;
			double cLeftGripOpen = gamepad2.right_trigger;
			boolean cRightGripClose = gamepad2.left_bumper;
			boolean cLeftGripClose = gamepad2.right_bumper;
			boolean cCombineReverse = gamepad2.x;
			boolean cArmExtend = gamepad2.dpad_down;
			boolean cArmRetract = gamepad2.dpad_up;

			double scale = cDriveTurboMode ? -2 : -1;
			double drive = scale * 0.45 * -cDrive;
			double turn = scale * 0.7 * cTurn;

			double leftPower = Range.clip(drive + turn, -1.0, 1.0);
			double rightPower = Range.clip(drive - turn, -1.0, 1.0);
			double leftFPower = Range.clip(-drive, -1.0, 1.0);
			double rightFPower = Range.clip(-drive, -1.0, 1.0);
			leftDrive.setPower(leftPower);
			rightDrive.setPower(rightPower);
			leftFrontDrive.setPower(leftFPower);
			rightFrontDrive.setPower(rightFPower);

			leftArm.setPower(cArm / (cArmTurboMode ? 1f : 2f));
			rightArm.setPower(cArm / (cArmTurboMode ? 1f : 2f));

			double linear = cLinearDown ? -1 : (cLinearUp ? 1 : 0);
			leftLinear.setPower(linear);
			rightLinear.setPower(linear);

			leftGrip.setPower(cRightGripClose ? 1 : (cRightGripOpen > 0.5 ? -1 : 0));
			rightGrip.setPower(cLeftGripClose ? -1 : (cLeftGripOpen > 0.5 ? 1 : 0));

			double asPower = cAntiSlipIn ? 1 : (cAntiSlipOut ? -1 : 0);
			leftAntiSlip.setPower(asPower);
			rightAntiSlip.setPower(asPower);

			combine.setPower(cCombineReverse ? 1 : -1);

			double armLinear = cArmExtend ? -1 : (cArmRetract ? 1 : 0);
			leftArmLinear.setPower(armLinear);
			rightArmLinear.setPower(armLinear);

			Beam.it("leftPower", leftPower);
			Beam.it("rightPower", rightPower);
			Beam.it("leftFrontPower", leftFPower);
			Beam.it("rightFrontPower", rightFPower);
			Beam.flush();
		}
	}
}
