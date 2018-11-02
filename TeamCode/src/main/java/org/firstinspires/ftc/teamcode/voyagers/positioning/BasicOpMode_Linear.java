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
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.voyagers.util.Beam;
import org.firstinspires.ftc.teamcode.voyagers.util.MiniPID;

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
public class BasicOpMode_Linear extends LinearOpMode
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
	private Servo leftGrip;
	private Servo rightGrip;
	private CRServo leftArmLinear;
	private CRServo rightArmLinear;

	private AnalogInput armPot;

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
		leftGrip = hardwareMap.get(Servo.class, "leftGrip");
		rightGrip = hardwareMap.get(Servo.class, "rightGrip");
		leftArmLinear = hardwareMap.get(CRServo.class, "leftArmLinear");
		rightArmLinear = hardwareMap.get(CRServo.class, "rightArmLinear");

		armPot = hardwareMap.get(AnalogInput.class, "armPot");

		// Most robots need the motor on one side to be reversed to drive forward
		// Reverse the motor that runs backwards when connected directly to the battery
		leftDrive.setDirection(DcMotor.Direction.FORWARD);
		rightDrive.setDirection(DcMotor.Direction.REVERSE);
		leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
		rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
		leftLinear.setDirection(DcMotor.Direction.FORWARD);
		rightLinear.setDirection(DcMotor.Direction.REVERSE);
		leftArm.setDirection(DcMotor.Direction.FORWARD);
		rightArm.setDirection(DcMotor.Direction.REVERSE);

		leftArmLinear.setDirection(CRServo.Direction.FORWARD);
		rightArmLinear.setDirection(CRServo.Direction.REVERSE);

		// Wait for the game to start (driver presses PLAY)
		waitForStart();
		runtime.reset();

		double armStraightUp = 0.9;
		double armStraightDown = 2.46;

		MiniPID miniPID = new MiniPID(1.4, 0, 0.3);
		miniPID.setOutputLimits(1);
		miniPID.setSetpointRange(1);
		miniPID.setSetpoint(armStraightDown);

		double arm = armStraightDown;

		// run until the end of the match (driver presses STOP)
		while (opModeIsActive())
		{
			double scale = (gamepad1.right_trigger > 0.5 ? 2 : gamepad1.left_trigger > 0.5 ? 0.5 : 1);
			double drive = scale * 0.45 * gamepad1.left_stick_y;
			double turn = scale * 0.35 * -gamepad1.right_stick_x;

			int fb = (gamepad1.right_trigger > 0.0 ? 1 : -1);
			double leftPower = Range.clip(drive + turn, -1.0, 1.0);
			double rightPower = Range.clip(drive - turn, -1.0, 1.0);
			double leftFPower = Range.clip(drive * -1 * fb + turn, -1.0, 1.0);
			double rightFPower = Range.clip(drive * -1 * fb - turn, -1.0, 1.0);
			leftDrive.setPower(leftPower * fb);
			rightDrive.setPower(rightPower * fb);
			leftFrontDrive.setPower(leftFPower);
			rightFrontDrive.setPower(rightFPower);

			arm += gamepad2.right_stick_y / 100;

			if (!gamepad2.x)
			{
				double armPower = miniPID.getOutput(armPot.getVoltage(), arm);
				leftArm.setPower(-armPower);
				rightArm.setPower(-armPower);
			}

			double linear = gamepad1.dpad_up ? -1 : (gamepad1.dpad_down ? 1 : 0);
			linear += gamepad2.dpad_up ? -1 : (gamepad2.dpad_down ? 1 : 0);
			leftLinear.setPower(linear);
			rightLinear.setPower(linear);

			double leftGripPosition = Range.clip(gamepad2.left_trigger, 0.38, 0.65);
			double rightGripPosition = Range.clip(gamepad2.right_trigger, 0, 0.55);

			leftGrip.setPosition(1 - leftGripPosition);
			rightGrip.setPosition(rightGripPosition);

			double armLinear = gamepad2.left_bumper ? -1 : (gamepad2.right_bumper ? 1 : 0);
			leftArmLinear.setPower(armLinear);
			rightArmLinear.setPower(armLinear);

			Beam.it("leftPower", leftPower);
			Beam.it("rightPower", rightPower);
			Beam.it("arm", arm);
			Beam.it("linear", linear);
			Beam.it("leftGripPosition", leftGripPosition);
			Beam.it("rightGripPosition", rightGripPosition);
			Beam.it("armLinear", armLinear);
			Beam.it("armPot", armPot.getVoltage());
			Beam.flush();
		}
	}
}
