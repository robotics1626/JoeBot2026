class SwerveModule {
    MotorController driveMotor;
    MotorController turnMotor;
    AbsoluteEncoder absEncoder;

    PIDController turnPID;
    PIDController drivePID;   // or motor-controller closed loop

    double angleOffsetRad;

    SwerveModule(driveID, turnID, encoderID, offsetRad) {
        driveMotor = new MotorController(driveID);
        turnMotor = new MotorController(turnID);
        absEncoder = new AbsoluteEncoder(encoderID);

        angleOffsetRad = offsetRad;

        turnPID = new PIDController(kP_turn, kI_turn, kD_turn);
        turnPID.enableContinuousInput(-PI, PI);

        drivePID = new PIDController(kP_drive, kI_drive, kD_drive);
    }

    double getAbsoluteAngleRad() {
        return wrapToMinusPiToPi(absEncoder.getRadians() - angleOffsetRad);
    }

    double getDriveVelocityMPS() {
        return convertMotorSensorToMPS(driveMotor.getVelocity());
    }

    double getDriveDistanceMeters() {
        return convertMotorSensorToMeters(driveMotor.getPosition());
    }

    SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveVelocityMPS(),
            new Rotation2d(getAbsoluteAngleRad())
        );
    }

    SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDriveDistanceMeters(),
            new Rotation2d(getAbsoluteAngleRad())
        );
    }

    void setDesiredState(SwerveModuleState desiredState) {
        // Prevent jitter at near-zero speed
        if (abs(desiredState.speedMetersPerSecond) < 0.01) {
            stop();
            return;
        }

        Rotation2d currentAngle = new Rotation2d(getAbsoluteAngleRad());

        // WPILib-recommended optimization step
        SwerveModuleState optimized =
            SwerveModuleState.optimize(desiredState, currentAngle);

        // Optional cosine compensation some teams add
        angleError = optimized.angle.minus(currentAngle).getRadians();
        optimized.speedMetersPerSecond *= cos(angleError);

        double driveOutput =
            drivePID.calculate(getDriveVelocityMPS(), optimized.speedMetersPerSecond);

        double turnOutput =
            turnPID.calculate(getAbsoluteAngleRad(), optimized.angles.getRadians());

        driveMotor.set(driveOutput);   // or setVelocity(...)
        turnMotor.set(turnOutput);
    }

    void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
