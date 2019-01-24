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
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.voyagers.util.Beam;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.*;

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

@Autonomous(name = "Auto", group = "Concept")
public class VuNav extends LinearOpMode
{
	public static final String TAG = "Vuforia Navigation Sample";
	private DcMotor leftDrive;
	private DcMotor rightDrive;
	private DcMotor leftFrontDrive;
	private DcMotor rightFrontDrive;
	private DcMotor leftLinear;
	private DcMotor rightLinear;
	private VuforiaLocalizer vuforia;
	private TFObjectDetector tfod;

	private void initVuforia()
	{
		/*
		 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
		 */
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
		parameters.vuforiaLicenseKey = "AbYs9VP/////AAABmQ/oqRZ3Y0HUgMxWT7WY7TYgCX++dmMODHSX6UBVOIyJ4IDy0zQlwWXB3dulOwewS1ojObAk7FBzdE3sgh1PU7Ovw8NaWKhA1LfrJS1zfgAvOAFdzMhfhoeRFZfChBkKXxVG0Nk8Rla+iYvCldDIhbJA98oUNB8fE/KEx9rDVvLHnxXI8L7PQYUKShZdH5qHb/A99YohcgXhUiEBwBSzWYcKAKZinXxVR1zCDcsC3vO+g+is6MZ3y9bWcXCpBsm95uc9m5Ad4jzCggpcKRW75SJIffXaM6YLMaIiipjjJgkYalKbnj39iNpjR0vJhrAHzRid/uP5jvnbTIF/BzE+e0049eoSh6F6nFBkEQEIG3ZD";

		parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

		//  Instantiate the Vuforia engine
		vuforia = ClassFactory.getInstance().createVuforia(parameters);

		// Loading trackables is not necessary for the Tensor Flow Object Detection engine.
	}

	private void initTfod()
	{
		int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

		TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();
		tfodParameters.useObjectTracker = false;
		tfodParameters.minimumConfidence = 0.75;
		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
		tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
	}

	@Override
	public void runOpMode()
	{
		//Voyagers.initOpMode(TAG, this);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		Beam.init(this.telemetry);

		initVuforia();

		if (ClassFactory.getInstance().canCreateTFObjectDetector())
		{
			initTfod();
		}
		else
		{
			telemetry.addData("Sorry!", "This device is not compatible with TFOD");
		}

		if (tfod != null)
		{
			tfod.activate();
		}

		// Initialize the hardware variables. Note that the strings used here as parameters
		// to 'get' must correspond to the names assigned during the robot configuration
		// step (using the FTC Robot Controller app on the phone).
		leftDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
		rightDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
		leftFrontDrive = hardwareMap.get(DcMotor.class, "left_forward_drive");
		rightFrontDrive = hardwareMap.get(DcMotor.class, "right_forward_drive");
		leftLinear = hardwareMap.get(DcMotor.class, "leftLinear");
		rightLinear = hardwareMap.get(DcMotor.class, "rightLinear");

		leftDrive.setDirection(DcMotor.Direction.FORWARD);
		rightDrive.setDirection(DcMotor.Direction.FORWARD);
		leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
		rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);

		leftLinear.setDirection(DcMotor.Direction.FORWARD);
		rightLinear.setDirection(DcMotor.Direction.REVERSE);

		waitForStart();

		//FieldMarkers.startTracking();

		long startTime = System.currentTimeMillis();
		long startTimeExtend = -1;

		int countLeft = 0;
		int countCenter = 0;
		int countRight = 0;
		boolean recognize = false;

		while (opModeIsActive())
		{
			// getUpdatedRecognitions() will return null if no new information is available since
			// the last time that call was made.
			List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
			if (updatedRecognitions != null && recognize)
			{
				telemetry.addData("Minerals: ", updatedRecognitions.size());

				if (updatedRecognitions.size() == 2)
				{
					int goldMineralX = -1;
					int silverMineral1X = -1;
					for (Recognition recognition : updatedRecognitions)
					{
						if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
						{
							goldMineralX = (int)recognition.getLeft();
						}
						else
						{
							silverMineral1X = (int)recognition.getLeft();
						}
					}

					if (goldMineralX != -1 && silverMineral1X != -1)
					{
						if (goldMineralX < silverMineral1X)
						{
							telemetry.addData("Gold Mineral Position", "Left");
							countLeft++;
						}
						else
						{
							telemetry.addData("Gold Mineral Position", "Center");
							countCenter++;
						}
					}
					else if (goldMineralX == -1)
					{
						telemetry.addData("Gold Mineral Position", "Right");
						countRight++;
					}
				}
			}

			if (System.currentTimeMillis() - startTime < 1650)
			{
				leftLinear.setPower(1);
				rightLinear.setPower(1);
			}
			else
			{
				recognize = true;
				leftLinear.setPower(0);
				rightLinear.setPower(0);
			}

			if (System.currentTimeMillis() - startTime > 5000 && System.currentTimeMillis() - startTime < 6800)
			{
				recognize = false;
				int direction = Math.max(Math.max(countLeft, countCenter), countRight);
				double scaleLeft = 1;
				double scaleRight = 1;
				if (direction == countLeft)
				{
					scaleLeft = 0.6;
				}
				else if (direction == countRight)
				{
					scaleRight = 0.6;
				}
				leftFrontDrive.setPower(0.7 * scaleLeft);
				rightFrontDrive.setPower(0.7 * scaleRight);
				leftDrive.setPower(-1 * scaleLeft);
				rightDrive.setPower(-0.7 * scaleRight);
			}
			else
			{
				leftFrontDrive.setPower(0);
				rightFrontDrive.setPower(0);
				leftDrive.setPower(0);
				rightDrive.setPower(0);
			}

			if (startTimeExtend == -1)
				startTimeExtend = System.currentTimeMillis();

			Beam.flush();
		}
	}
}