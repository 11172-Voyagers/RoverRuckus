package org.firstinspires.ftc.teamcode.voyagers.util;

import java.util.ArrayList;
import java.util.Iterator;

public class Timeline
{
	private final ArrayList<TimelineEvent> staticevents;
	private boolean running;
	private long startTime;
	private ArrayList<TimelineEvent> events;

	public Timeline(ArrayList<TimelineEvent> events)
	{
		this.events = staticevents = events;
	}

	public void start()
	{
		events = (ArrayList<TimelineEvent>)staticevents.clone();
		startTime = System.currentTimeMillis();
		running = true;
	}

	public void tick()
	{
		if (!running)
			return;
		long delta = System.currentTimeMillis() - startTime;
		for (Iterator<TimelineEvent> i = events.iterator(); i.hasNext(); )
		{
			TimelineEvent e = i.next();
			if (delta >= e.time)
			{
				e.action.accept(e);
				i.remove();
			}
		}
	}
}

