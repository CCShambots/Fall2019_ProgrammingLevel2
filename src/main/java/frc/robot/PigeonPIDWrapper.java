package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PigeonPIDWrapper implements PIDSource, PIDOutput {

    private PIDSourceType mSourceType;
    private PigeonIMU mPigeon;
    private double mOutput;

    public PigeonPIDWrapper(PigeonIMU in_pigeon) {
        mSourceType = PIDSourceType.kDisplacement;
        mPigeon = in_pigeon;
        mOutput = 0;
    }

    public void setPIDSourceType(PIDSourceType pidSource) {
        mSourceType = pidSource;
    }

    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    public double pidGet() {
        return mPigeon.getFusedHeading();
    }

    public void pidWrite(double output) {
        mOutput = output;
    }

}