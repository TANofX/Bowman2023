package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.beans.Transient;
import java.util.List;

import javax.accessibility.AccessibleAttributeSequence;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class GreekArmTest {
	private static final double delta = 0.25;

	@Test
	public void testGreekArm() throws Exception {

	}

	@Test
	public void testCalculateKinematics() throws Exception {
		Translation3d neutralPosition = GreekArm.calculateKinematics(0, 0);

		assertEquals(58.519, neutralPosition.getX(), delta, "Neutral X");
		assertEquals(38.386, neutralPosition.getZ(), delta, "Neutral Z");

		Translation3d fullHeight = GreekArm.calculateKinematics(90.0, 0);

		assertEquals(0.0, fullHeight.getX(), delta, "Full X");
		assertEquals(96.905, fullHeight.getZ(), delta, "Full Z");

		Translation3d doubleRightAngle = GreekArm.calculateKinematics(90.0, 90.0);

		assertEquals(-31.5, doubleRightAngle.getX(), delta, "Right X");
		assertEquals(65.605, doubleRightAngle.getZ(), delta, "Right Z");
	}

	@Test
	public void testCalculateInverseKinematics() throws Exception {
		List<Rotation2d> neutral = GreekArm.calculateInverseKinematics(58.5186, 38.386);

		assertEquals(0.0, neutral.get(0).getRadians(), delta, "Neutral Shoulder");
		assertEquals(0.0, neutral.get(1).getRadians(), delta, "Neutral Elbow");

		List<Rotation2d> top = GreekArm.calculateInverseKinematics(0.0, 96.905);
		assertEquals(Math.PI / 2.0, top.get(0).getRadians(), delta, "Top Shoulder");
		assertEquals(0.0, top.get(1).getRadians(), delta, "Top Elbow");
	}

	@Test
	public void testKinematics() throws Exception {
		for (double x = 0; x < 55.0; x += 0.1) {
			for (double y = 0; y < 75.0; y += 0.1) {
				List<Rotation2d> angles = GreekArm.calculateInverseKinematics(x, y);
				Translation3d endEffector = GreekArm.calculateKinematics(angles.get(0).getDegrees(), angles.get(1).getDegrees());

				System.out.println(endEffector);
				if (!Double.isNaN(angles.get(0).getRadians()) && !Double.isNaN(angles.get(1).getRadians())) {
					assertEquals(x, endEffector.getX(), delta, "X");
					assertEquals(y, endEffector.getZ(), delta, "Z");
				}
			}
		}
	}

	@Test
	public void testPeriodic() throws Exception {

	}

}
