package org.firstinspires.ftc.teamcode.voyagers.util;

import org.firstinspires.ftc.robotcore.external.Consumer;

public class TimelineEvent
{
	public final long time;
	public final Consumer<TimelineEvent> action;

	public TimelineEvent(long time, Consumer<TimelineEvent> action)
	{
		this.time = time;
		this.action = action;
	}
}
