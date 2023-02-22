package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.beans.Transient;
import java.util.List;

import javax.accessibility.AccessibleAttributeSequence;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

import frc.robot.Constants;

public class GreekArmTest {
	private static final double delta = 0.25;

	@Test
	public void testGreekArm() throws Exception {

	}

	@Test
	public void testCalculateKinematics() throws Exception {
		Translation3d neutralPosition = GreekArm.calculateKinematics(0, 0);

		assertEquals(Constants.ARTEMIS_LENGTH + Constants.APOLLO_LENGTH, neutralPosition.getX(), delta, "Neutral X");
		assertEquals(Constants.HERCULES_LENGTH, neutralPosition.getZ(), delta, "Neutral Z");

		Translation3d fullHeight = GreekArm.calculateKinematics(90.0, 0);

		assertEquals(0.0, fullHeight.getX(), delta, "Full X");
		assertEquals(Constants.HERCULES_LENGTH + Constants.ARTEMIS_LENGTH + Constants.APOLLO_LENGTH, fullHeight.getZ(), delta, "Full Z");

		Translation3d doubleRightAngle = GreekArm.calculateKinematics(90.0, 90.0);

		assertEquals(-Constants.ARTEMIS_LENGTH, doubleRightAngle.getX(), delta, "Right X");
		assertEquals(Constants.HERCULES_LENGTH + Constants.APOLLO_LENGTH, doubleRightAngle.getZ(), delta, "Right Z");
	}

	@Test
	public void testCalculateInverseKinematics() throws Exception {
		List<Rotation2d> neutral = GreekArm.calculateInverseKinematics(Constants.ARTEMIS_LENGTH + Constants.APOLLO_LENGTH, Constants.HERCULES_LENGTH);

		assertEquals(0.0, neutral.get(0).getRadians(), delta, "Neutral Shoulder");
		assertEquals(0.0, neutral.get(1).getRadians(), delta, "Neutral Elbow");

		List<Rotation2d> top = GreekArm.calculateInverseKinematics(0.0, Constants.HERCULES_LENGTH + Constants.ARTEMIS_LENGTH + Constants.APOLLO_LENGTH);
		assertEquals(Math.PI / 2.0, top.get(0).getRadians(), delta, "Top Shoulder");
		assertEquals(0.0, top.get(1).getRadians(), delta, "Top Elbow");
	}

	@Test
	public void testKinematics() throws Exception {
		for (double x = 0; x < 1.0; x += 0.1) {
			for (double y = 0; y < 1.0; y += 0.1) {
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
	public void testSpeedCalcs() {
		GreekArm anArm = new GreekArm();

		anArm.setEndEffectorSpeeds(-0.01, 0.0);

		GreekArm.JointSpeeds tSpeeds = anArm.getTargetJointSpeeds();
		System.out.println(tSpeeds);

		Translation3d armPos = GreekArm.calculateKinematics(tSpeeds.getShoulderSpeed().getDegrees(), tSpeeds.getElbowSpeed().getDegrees());
		System.out.println(armPos);
	}

	@Test
	public void testPeriodic() throws Exception {

	}

}
