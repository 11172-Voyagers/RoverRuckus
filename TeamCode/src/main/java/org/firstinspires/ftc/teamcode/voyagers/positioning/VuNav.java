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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.voyagers.util.Beam;
import org.firstinspires.ftc.teamcode.voyagers.util.MiniPID;

/**
 * This 2016-2017 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 * <p>
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 * <p>
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 * <p>
 * This example assumes a "diamond" field configuration where the red and blue alliance stations
 * are adjacent on the corner of the field furthest from the audience.
 * From the Audience perspective, the Red driver station is on the right.
 * The two vision target are located on the two walls closest to the audience, facing in.
 * The Stones are on the RED side of the field, and the Chips are on the Blue side.
 * <p>
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * 		see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 * 		<p>
 * 		Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * 		Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * 		<p>
 * 		IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * 		is explained below.
 */

@Autonomous(name = "Concept: Vuforia Navigation", group = "Concept")
public class VuNav extends LinearOpMode
{
	public static final String TAG = "Vuforia Navigation Sample";
	private DcMotor leftDrive;
	private DcMotor rightDrive;
	private DcMotor leftFrontDrive;
	private DcMotor rightFrontDrive;
	private DcMotor leftLinear;
	private DcMotor rightLinear;
	private DcMotor leftArm;
	private DcMotor rightArm;
	private AnalogInput armPot;
	private CRServo leftArmLinear;
	private CRServo rightArmLinear;

	@Override
	public void runOpMode()
	{
		//Voyagers.initOpMode(TAG, this);

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
		armPot = hardwareMap.get(AnalogInput.class, "armPot");
		leftArmLinear = hardwareMap.get(CRServo.class, "leftArmLinear");
		rightArmLinear = hardwareMap.get(CRServo.class, "rightArmLinear");

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

		waitForStart();

		//FieldMarkers.startTracking();

		long startTime = System.currentTimeMillis();
		long startTimeExtend = -1;

		double armStraightUp = 0.9;
		double armStraightDown = 2.5;

		MiniPID miniPID = new MiniPID(1.4, 0, 0);
		miniPID.setOutputLimits(1);
		miniPID.setSetpointRange(1);
		miniPID.setSetpoint(armStraightDown);

		double arm = armPot.getVoltage();
		arm = Range.clip(arm, armStraightUp, armStraightDown);

		while (opModeIsActive())
		{
			//OpenGLMatrix lastLocation = FieldMarkers.getLocation(telemetry);

			//			if (lastLocation != null)
			//				Beam.it("Pos", FieldMarkers.formatMatrix(lastLocation));
			//			else
			//				Beam.it("Pos", "Unknown");

			if (System.currentTimeMillis() - startTime < 5600)
			{
				leftLinear.setPower(-1);
				rightLinear.setPower(-1);
			}
			else
			{
				leftLinear.setPower(0);
				rightLinear.setPower(0);
			}

			if (System.currentTimeMillis() - startTime > 5000 && System.currentTimeMillis() - startTime < 6200)
			{
				leftFrontDrive.setPower(1);
				rightFrontDrive.setPower(1);
				leftDrive.setPower(-1);
				rightDrive.setPower(-1);
			}
			else
			{
				leftFrontDrive.setPower(0);
				rightFrontDrive.setPower(0);
				leftDrive.setPower(0);
				rightDrive.setPower(0);
			}

			double armPower = 0;
			arm = (armStraightUp + armStraightDown) / 2;
			armPower = miniPID.getOutput(armPot.getVoltage(), arm);

			if (Math.abs(arm - armStraightUp) < 0.2)
			{
				if (startTimeExtend == -1)
					startTimeExtend = System.currentTimeMillis();

				if (System.currentTimeMillis() - startTimeExtend < 16000)
				{
					leftArmLinear.setPower(1);
					rightArmLinear.setPower(1);
				}
				else
				{
					leftArmLinear.setPower(0);
					rightArmLinear.setPower(0);
				}
			}

			leftArm.setPower(-armPower);
			rightArm.setPower(-armPower);

			Beam.flush();
		}
	}
}
