package org.firstinspires.ftc.teamcode.voyagers.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.voyagers.Voyagers;

/**
 * Created by Administrator on 9/22/2018.
 */

public class FieldMarkers
{
	private static OpenGLMatrix lastLocation;

	private static VuforiaTrackables navMarkers;

	public static void init(HardwareMap hardwareMap, String tag)
	{
		lastLocation = null;

		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
		parameters.vuforiaLicenseKey = "AbYs9VP/////AAABmQ/oqRZ3Y0HUgMxWT7WY7TYgCX++dmMODHSX6UBVOIyJ4IDy0zQlwWXB3dulOwewS1ojObAk7FBzdE3sgh1PU7Ovw8NaWKhA1LfrJS1zfgAvOAFdzMhfhoeRFZfChBkKXxVG0Nk8Rla+iYvCldDIhbJA98oUNB8fE/KEx9rDVvLHnxXI8L7PQYUKShZdH5qHb/A99YohcgXhUiEBwBSzWYcKAKZinXxVR1zCDcsC3vO+g+is6MZ3y9bWcXCpBsm95uc9m5Ad4jzCggpcKRW75SJIffXaM6YLMaIiipjjJgkYalKbnj39iNpjR0vJhrAHzRid/uP5jvnbTIF/BzE+e0049eoSh6F6nFBkEQEIG3ZD";

		parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
		VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

		navMarkers = vuforia.loadTrackablesFromAsset("RoverRuckusNavMarkers");
		VuforiaTrackable tgtStars = navMarkers.get(0);
		tgtStars.setName("stars");

		VuforiaTrackable tgtRover = navMarkers.get(1);
		tgtRover.setName("rover");

		VuforiaTrackable tgtMoon = navMarkers.get(2);
		tgtMoon.setName("moon");

		VuforiaTrackable tgtMars = navMarkers.get(3);
		tgtMars.setName("mars");

		float metersPerInch = 0.0254f;
		float metersFieldWidth = (12 * 12 - 2) * metersPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

		OpenGLMatrix locTgtStars = OpenGLMatrix.translation(0, metersFieldWidth / 2, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES, 90, 90, 0));
		tgtStars.setLocation(locTgtStars);
		RobotLog.ii(tag, "Stars=%s", formatMatrix(locTgtStars));

		OpenGLMatrix locTgtRover = OpenGLMatrix.translation(-metersFieldWidth / 2, 0, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES, 90, 90, 0));
		tgtRover.setLocation(locTgtRover);
		RobotLog.ii(tag, "Rover=%s", formatMatrix(locTgtRover));

		OpenGLMatrix locTgtMoon = OpenGLMatrix.translation(metersFieldWidth / 2, 0, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES, 90, 0, 0));
		tgtMoon.setLocation(locTgtMoon);
		RobotLog.ii(tag, "Moon=%s", formatMatrix(locTgtMoon));

		OpenGLMatrix locTgtMars = OpenGLMatrix.translation(0, -metersFieldWidth / 2, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES, 90, 90, 0));
		tgtMars.setLocation(locTgtMars);
		RobotLog.ii(tag, "Stars=%s", formatMatrix(locTgtMars));

		/**
		 * Create a transformation matrix describing where the phone is on the robot. Here, we
		 * put the phone on the right hand side of the robot with the screen facing in (see our
		 * choice of BACK camera above) and in landscape mode. Starting from alignment between the
		 * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
		 *
		 * When determining whether a rotation is positive or negative, consider yourself as looking
		 * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
		 * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
		 * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
		 * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
		 */
		RobotInfo info = Voyagers.instance.getRobotInfo();
		OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix.translation(info.phoneCamX, info.phoneCamY, info.phoneCamZ).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.YZY, AngleUnit.DEGREES, -90, 0, 0));
		RobotLog.ii(tag, "phone=%s", formatMatrix(phoneLocationOnRobot));

		/**
		 * Let the trackable listeners we care about know where the phone is. We know that each
		 * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
		 * we have not ourselves installed a listener of a different type.
		 */
		((VuforiaTrackableDefaultListener)tgtStars.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
		((VuforiaTrackableDefaultListener)tgtRover.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
		((VuforiaTrackableDefaultListener)tgtMoon.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
		((VuforiaTrackableDefaultListener)tgtMars.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
	}

	/**
	 * A simple utility that extracts positioning information from a transformation matrix
	 * and formats it in a form palatable to a human being.
	 */
	public static String formatMatrix(OpenGLMatrix transformationMatrix)
	{
		return transformationMatrix.formatAsTransform();
	}

	public static void startTracking()
	{
		navMarkers.activate();
	}

	public static OpenGLMatrix getLocation(Telemetry telemetry)
	{
		for (VuforiaTrackable trackable : navMarkers)
		{
			telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");

			OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
			if (robotLocationTransform != null)
			{
				lastLocation = robotLocationTransform;
			}
		}

		return lastLocation;
	}
}
