package org.firstinspires.ftc.teamcode.voyagers.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Administrator on 9/22/2018.
 */

public class Beam
{
	private static Telemetry telemetry;

	public static void init(Telemetry tele)
	{
		telemetry = tele;
	}

	public static void it(String title, Object message)
	{
		telemetry.addData(title, message);
		telemetry.update();
	}
}