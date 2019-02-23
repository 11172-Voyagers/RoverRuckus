package org.firstinspires.ftc.teamcode.voyagers.positioning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
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

@Autonomous(name = "AutoDepot", group = "Concept")
public class AutoDepot extends LinearOpMode
{
	private DcMotor leftDrive;
	private DcMotor rightDrive;
	private DcMotor leftFrontDrive;
	private DcMotor rightFrontDrive;
	private DcMotor leftLinear;
	private DcMotor rightLinear;
	private DcMotor leftArm;
	private DcMotor rightArm;
	private CRServo leftAntiSlip;
	private CRServo rightAntiSlip;
	private CRServo marker;
	private VuforiaLocalizer vuforia;
	private TFObjectDetector tfod;

	private Timeline t;

	private int countLeft = 0;
	private int countCenter = 0;
	private int countRight = 0;
	private boolean recognize = false;

	private String strStatus = "Init";

	@Override
	public void runOpMode()
	{
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
		leftArm = hardwareMap.get(DcMotor.class, "leftArm");
		rightArm = hardwareMap.get(DcMotor.class, "rightArm");

		rightLinear = hardwareMap.get(DcMotor.class, "rightLinear");

		leftAntiSlip = hardwareMap.get(CRServo.class, "leftAntiSlip");
		rightAntiSlip = hardwareMap.get(CRServo.class, "rightAntiSlip");
		marker = hardwareMap.get(CRServo.class, "marker");

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

		leftAntiSlip.setDirection(CRServo.Direction.REVERSE);
		rightAntiSlip.setDirection(CRServo.Direction.FORWARD);

		marker.setDirection(CRServo.Direction.FORWARD);

		leftLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rightLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		ArrayList<TimelineEvent> events = new ArrayList<>();

		int lalPos = leftLinear.getCurrentPosition();
		int larPos = rightLinear.getCurrentPosition();

		// start antislip, arm movement
		events.add(new TimelineEvent(0, (o) -> {
			status("Antislip Init");
			leftAntiSlip.setPower(1);
			rightAntiSlip.setPower(1);

			leftArm.setPower(-0.25f);
			rightArm.setPower(-0.25f);
		}));

		// stop arm movement
		events.add(new TimelineEvent(500, (o) -> {
			status("Arm End");
			leftArm.setPower(0);
			rightArm.setPower(0);
		}));

		// stop antislip
		events.add(new TimelineEvent(3800, (o) -> {
			status("Antislip End");
			leftAntiSlip.setPower(0);
			rightAntiSlip.setPower(0);
		}));

		// linear move down
		events.add(new TimelineEvent(3500, (o) -> {
			status("Linear Down");
			leftLinear.setTargetPosition(lalPos + 1250);
			rightLinear.setTargetPosition(larPos + 1250);
			leftLinear.setPower(1);
			rightLinear.setPower(1);
		}));

		// move to be recognizing
		events.add(new TimelineEvent(7500, (o) -> {
			status("Moving to recognize");
			drive(0, 0.5f, false);
		}));

		// start recognizing
		events.add(new TimelineEvent(8100, (o) -> {
			status("Recognizing");
			recognize = true;
			drive(0, 0, false);
		}));

		// unmove to be recognizing
		events.add(new TimelineEvent(9500, (o) -> {
			status("Unmoving to recognize");
			recognize = false;
			drive(0, -0.5f, false);
		}));

		// sample
		events.add(new TimelineEvent(10000, (o) -> {
			status("Sampling");
			int direction = Math.max(Math.max(countLeft, countCenter), countRight);
			float turn = 0;
			if (direction == countLeft)
			{
				turn = -0.57f;
			}
			else if (direction == countRight)
			{
				turn = 0.57f;
			}

			drive(-1, turn, false);
		}));

		// sample
		events.add(new TimelineEvent(11800, (o) -> {
			status("Driving");
			int direction = Math.max(Math.max(countLeft, countCenter), countRight);
			float turn = 0;
			if (direction == countLeft)
			{
				turn = -0.6f;
			}
			else if (direction == countRight)
			{
				turn = 0.6f;
			}

			drive(-1, -1.05f * turn, false);
		}));

		// halt
		events.add(new TimelineEvent(13600, (o) -> {
			status("Halt");
			drive(0, 0, false);
		}));

		// halt
		events.add(new TimelineEvent(13600, (o) -> {
			status("marker out");
			marker.setPower(1);
		}));

		// halt
		events.add(new TimelineEvent(14800, (o) -> {
			status("marker stop");
			marker.setPower(0);
		}));

		t = new Timeline(events);
		waitForStart();

		t.start();

		while (opModeIsActive())
		{
			if (recognize)
			{
				List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
				telemetry.addData("ur", updatedRecognitions == null);
				if (updatedRecognitions != null)
				{
					telemetry.addData("Count", updatedRecognitions.size());

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
								telemetry.addData("Gold Mineral Position", "Center");
								countCenter++;
							}
							else
							{
								telemetry.addData("Gold Mineral Position", "Right");
								countRight++;
							}
						}
						else if (goldMineralX == -1)
						{
							telemetry.addData("Gold Mineral Position", "Left");
							countLeft++;
						}
					}
				}
			}

			telemetry.addData("Left", countLeft);
			telemetry.addData("Center", countCenter);
			telemetry.addData("Right", countRight);

			telemetry.addData("Status", strStatus);

			int direction = Math.max(Math.max(countLeft, countCenter), countRight);
			if (direction == countLeft)
			{
				telemetry.addData("Gold Mineral Position", "Left");
			}
			else if (direction == countRight)
			{
				telemetry.addData("Gold Mineral Position", "Right");
			}
			else
				telemetry.addData("Gold Mineral Position", "Center");

			t.tick();

			Beam.flush();
		}
	}

	private void status(String status)
	{
		strStatus = status;
		Beam.it("Status", status);
	}

	private void drive(float cDrive, float cTurn, boolean cDriveTurboMode)
	{
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
