package org.firstinspires.ftc.teamcode.voyagers.positioning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.voyagers.util.Beam;
import org.firstinspires.ftc.teamcode.voyagers.util.Timeline;
import org.firstinspires.ftc.teamcode.voyagers.util.TimelineEvent;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.*;

@Autonomous(name = "Auto", group = "Concept")
public class AutoCrater extends LinearOpMode
{
	private DcMotor leftDrive;
	private DcMotor rightDrive;
	private DcMotor leftFrontDrive;
	private DcMotor rightFrontDrive;
	private DcMotor leftLinear;
	private DcMotor rightLinear;
	private VuforiaLocalizer vuforia;
	private TFObjectDetector tfod;

	private Timeline t;

	private int countLeft = 0;
	private int countCenter = 0;
	private int countRight = 0;
	private boolean recognize = false;

	@Override
	public void runOpMode()
	{
		Beam.init(this.telemetry);

		initVuforia();
		initTfod();
		tfod.activate();

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

		ArrayList<TimelineEvent> events = new ArrayList<>();

		// linear move down
		events.add(new TimelineEvent(0, (o) -> {
			leftLinear.setPower(1);
			rightLinear.setPower(1);
		}));

		// linear stop
		events.add(new TimelineEvent(2100, (o) -> {
			leftLinear.setPower(0);
			rightLinear.setPower(0);
		}));

		// start recognizing
		events.add(new TimelineEvent(2100, (o) -> {
			recognize = true;
		}));

		// sample
		events.add(new TimelineEvent(4000, (o) -> {
			recognize = false;
			int direction = Math.max(Math.max(countLeft, countCenter), countRight);
			double scaleLeft = 1;
			double scaleRight = 1;
			if (direction == countLeft)
			{
				scaleLeft = 0.3;
				telemetry.addData("Gold Mineral Position", "Left");
			}
			else if (direction == countRight)
			{
				scaleRight = 0.3;
				telemetry.addData("Gold Mineral Position", "Right");
			}
			else
				telemetry.addData("Gold Mineral Position", "Center");

			leftDrive.setPower(scaleLeft);
			rightDrive.setPower(scaleRight);
			leftFrontDrive.setPower(scaleLeft);
			rightFrontDrive.setPower(scaleRight);
		}));

		// halt
		events.add(new TimelineEvent(8000, (o) -> {
			leftDrive.setPower(0);
			rightDrive.setPower(0);
			leftFrontDrive.setPower(0);
			rightFrontDrive.setPower(0);
		}));

		t = new Timeline(events);
		waitForStart();

		while (opModeIsActive())
		{
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

			telemetry.addData("Left", countLeft);
			telemetry.addData("Center", countCenter);
			telemetry.addData("Right", countRight);

			t.tick();

			Beam.flush();
		}
	}

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
		tfodParameters.minimumConfidence = 0.55;
		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
		tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
	}
}
