package common.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;

import common.Logger;
import common.math.MathUtils;
import common.math.Vector2;
import common.robot.input.Axis;
import common.robot.subsystems.HolonomicDrivetrain;
import common.control.PidConstants;

@Deprecated
@SuppressWarnings("removal")
public final class HolonomicDriveCommand extends CommandBase {
	private static final double ROTATION_END_TIMEOUT = 0.5;
	private static final Logger LOGGER = new Logger(HolonomicDriveCommand.class);

	private final HolonomicDrivetrain drivetrain;
	private final Axis forwardAxis;
	private final Axis strafeAxis;
	private final Axis rotationAxis;
	private final Button fieldOrientedOverrideButton;

	private final Timer rotationEndTimer = new Timer();
	private boolean waitingForRotationTimer = false;

	private final PIDController angleController;
	private double angleControllerOutput = 0;

	public HolonomicDriveCommand(HolonomicDrivetrain drivetrain, Axis forwardAxis, Axis strafeAxis, Axis rotationAxis,
			Button fieldOrientedOverrideButton) {
		this(drivetrain, forwardAxis, strafeAxis, rotationAxis, fieldOrientedOverrideButton, new PidConstants(0, 0, 0));
	}

	public HolonomicDriveCommand(HolonomicDrivetrain drivetrain, Axis forwardAxis, Axis strafeAxis, Axis rotationAxis,
			Button fieldOrientedOverrideButton, PidConstants constants) {
		this.drivetrain = drivetrain;
		this.forwardAxis = forwardAxis;
		this.strafeAxis = strafeAxis;
		this.rotationAxis = rotationAxis;
		this.fieldOrientedOverrideButton = fieldOrientedOverrideButton;

		angleController = new PIDController(constants.p, constants.i, constants.d, new PIDSource() {
			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {
			}

			@Override
			public double pidGet() {
				return drivetrain.getGyroscope().getAngle().toDegrees();
			}

			@Override
			public PIDSourceType getPIDSourceType() {
				return PIDSourceType.kDisplacement;
			}
		}, output -> {
			angleControllerOutput = output;
		});
		angleController.setInputRange(0, 360);
		angleController.setContinuous(true);

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		waitingForRotationTimer = false;

		angleController.setSetpoint(drivetrain.getGyroscope().getAngle().toDegrees());
		angleController.enable();
	}

	@Override
	public void execute() {
		double forward = forwardAxis.get(true);
		double strafe = strafeAxis.get(true);
		double rotation = rotationAxis.get(true);
		boolean fieldOriented = !fieldOrientedOverrideButton.get();

		if (MathUtils.epsilonEquals(rotation, 0)) {
			if (waitingForRotationTimer) {
				if (rotationEndTimer.get() > ROTATION_END_TIMEOUT) {
					LOGGER.debug("Correcting angle");
					rotation = angleControllerOutput;

					if (rotationAxis.isInverted()) {
						rotation *= -1;
					}
				} else {
					angleController.setSetpoint(drivetrain.getGyroscope().getAngle().toDegrees());
				}
			} else {
				LOGGER.debug("Starting end wait");
				rotationEndTimer.reset();
				rotationEndTimer.start();
				waitingForRotationTimer = true;
			}
		} else {
			waitingForRotationTimer = false;
			angleController.setSetpoint(drivetrain.getGyroscope().getAngle().toDegrees());
		}

		drivetrain.holonomicDrive(new Vector2(forward, strafe), rotation, fieldOriented);
	}

	@Override
	public void end(boolean interrupted) {
		angleController.disable();
		drivetrain.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
