package redbacks.lib.hardware.mechanism.drivetrain.ctre;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CtreSwerveModuleWithPositionPidSteer extends SwerveModule {
    private final ClosedLoopOutputType steerClosedLoopOutputType;
    private final PositionVoltage angleVoltageSetter;
    private final PositionTorqueCurrentFOC angleTorqueSetter;

    public CtreSwerveModuleWithPositionPidSteer(SwerveModuleConstants constants, String canbusName) {
        super(constants, canbusName);

        steerClosedLoopOutputType = constants.SteerMotorClosedLoopOutput;
        angleVoltageSetter = new PositionVoltage(0).withUpdateFreqHz(0);
        angleTorqueSetter = new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);
    }

    @Override
    public void apply(SwerveModuleState state, DriveRequestType driveRequestType, SteerRequestType steerRequestType) {
        super.apply(state, driveRequestType, null);

        double angleToSetRotations = getTargetState().angle.getRotations();
        switch (steerClosedLoopOutputType) {
            case Voltage -> getSteerMotor().setControl(angleVoltageSetter.withPosition(angleToSetRotations));
            case TorqueCurrentFOC -> getSteerMotor().setControl(angleTorqueSetter.withPosition(angleToSetRotations));
        }
    }
}
