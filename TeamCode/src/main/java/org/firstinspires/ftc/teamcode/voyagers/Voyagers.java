package org.firstinspires.ftc.teamcode.voyagers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.voyagers.util.Beam;
import org.firstinspires.ftc.teamcode.voyagers.util.FieldMarkers;
import org.firstinspires.ftc.teamcode.voyagers.util.RobotInfo;

/**
 * Created by Administrator on 9/22/2018.
 */

public class Voyagers
{
	public static final Voyagers instance = new Voyagers();

	private final RobotInfo robotInfo;

	private Voyagers()
	{
		robotInfo = new RobotInfo(1, 1, 1, 0, 0, 0);
	}

	public RobotInfo getRobotInfo()
	{
		return robotInfo;
	}

	public static void initOpMode(String tag, OpMode opMode)
	{
		FieldMarkers.init(opMode.hardwareMap, tag);
		Beam.init(opMode.telemetry);
	}
}
