package org.firstinspires.ftc.teamcode.voyagers.positioning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "TraceAuto", group = "Concept")
public class TraceAuto extends LinearOpMode
{
	private DcMotor leftDrive;
	private DcMotor rightDrive;
	private DcMotor leftFrontDrive;
	private DcMotor rightFrontDrive;

	int[] waypointsLf = { 622, -658, 2653, -40, 3618 };
	int[] waypointsLb = { -608, 1399, -2613, -371, -3603 };
	int[] waypointsRf = { 626, 628, 2640, -1276, 3572 };
	int[] waypointsRb = { -626, -1285, -2620, 1780, -3525 };

	int waypointIndex = 0;

	@Override
	public void runOpMode()
	{
		leftDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
		rightDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
		leftFrontDrive = hardwareMap.get(DcMotor.class, "left_forward_drive");
		rightFrontDrive = hardwareMap.get(DcMotor.class, "right_forward_drive");

		leftDrive.setDirection(DcMotor.Direction.FORWARD);
		rightDrive.setDirection(DcMotor.Direction.REVERSE);
		leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
		rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);

		leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		int posLf = leftFrontDrive.getCurrentPosition();
		int posLb = leftDrive.getCurrentPosition();
		int posRf = rightFrontDrive.getCurrentPosition();
		int posRb = rightDrive.getCurrentPosition();

		waitForStart();

		leftFrontDrive.setPower(0.2);
		leftDrive.setPower(0.2);
		rightFrontDrive.setPower(0.2);
		rightDrive.setPower(0.2);

		while (opModeIsActive())
		{
			posLf += waypointsLf[waypointIndex];
			posLb += waypointsLb[waypointIndex];
			posRf += waypointsRf[waypointIndex];
			posRb += waypointsRb[waypointIndex];

			leftFrontDrive.setTargetPosition(posLf);
			leftDrive.setTargetPosition(posLb);
			rightFrontDrive.setTargetPosition(posRf);
			rightDrive.setTargetPosition(posRb);

			waypointIndex++;
			while (leftDriveisBusy() || rightDriveisBusy() || leftFrontDriveisBusy() || rightFrontDriveisBusy())
				;
			if (waypointIndex >= waypointsLb.length)
				break;
		}
	}

	private boolean rightFrontDriveisBusy()
	{
		return Math.abs(rightFrontDrive.getCurrentPosition() - rightFrontDrive.getTargetPosition()) > 20;
	}

	private boolean leftFrontDriveisBusy()
	{
		return Math.abs(leftFrontDrive.getCurrentPosition() - leftFrontDrive.getTargetPosition()) > 20;
	}

	private boolean rightDriveisBusy()
	{
		return Math.abs(rightDrive.getCurrentPosition() - rightDrive.getTargetPosition()) > 20;
	}

	private boolean leftDriveisBusy()
	{
		return Math.abs(leftDrive.getCurrentPosition() - leftDrive.getTargetPosition()) > 20;
	}
}
