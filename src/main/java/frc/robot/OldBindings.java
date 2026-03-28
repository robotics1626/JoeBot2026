//   private void configureButtonBindings() {
//     // Default command, normal field-relative drive
//     // drive.setDefaultCommand(
//     // DriveCommands.joystickDrive(
//     // drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () ->
//     // -driver.getRightX()));

//     // Lock to 0° when A button is held
//     driver
//         .a()
//         .whileTrue(
//             DriveCommands.joystickDriveAtAngle(
//                 drive,
//                 () -> -applyLeftDeadband(driver.getLeftY()),
//                 () -> -applyLeftDeadband(driver.getLeftX()),
//                 () -> Rotation2d.kZero));

//     // Switch to X pattern when X button is pressed
//     driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

//     RobotModeTriggers.teleop()
//         .whileTrue(
//             new HeadingDrive(
//                 drive,
//                 () -> applyLeftDeadband(driver.getLeftX()),
//                 () -> applyLeftDeadband(driver.getLeftY()),
//                 () -> -applyRightDeadband(driver.getRightX()),
//                 () -> -applyRightDeadband(driver.getRightY()),
//                 MaxSpeed,
//                 MaxAngularRate,
//                 90));

//     // Reset gyro to 0° when B button is pressed
//     driver
//         .b()
//         .onTrue(
//             Commands.runOnce(
//                     () ->
//                         drive.setPose(
//                             new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
//                     drive)
//                 .ignoringDisable(true))
//         .onTrue(
//             new HeadingDrive(
//                 drive,
//                 () -> applyLeftDeadband(driver.getLeftX()),
//                 () -> applyLeftDeadband(driver.getLeftY()),
//                 () -> -applyRightDeadband(driver.getRightX()),
//                 () -> -applyRightDeadband(driver.getRightY()),
//                 MaxSpeed,
//                 MaxAngularRate,
//                 90));

//     // driver.rightTrigger().whileTrue(aimbot);
//     // Right trigger: align to hub using AlignToPose
//     driver
//         .rightTrigger()
//         .whileTrue(
//             new AlignHeadingToHub(
//                     drive,
//                     () -> -applyLeftDeadband(driver.getLeftY()),
//                     () -> -applyLeftDeadband(driver.getLeftX()),
//                     true)
//                 .alongWith(new AutoAimShooter(drive, vision, shooter)))
//         .onFalse(
//             new HeadingDrive(
//                 drive,
//                 () -> applyLeftDeadband(driver.getLeftX()),
//                 () -> applyLeftDeadband(driver.getLeftY()),
//                 () -> -applyRightDeadband(driver.getRightX()),
//                 () -> -applyRightDeadband(driver.getRightY()),
//                 MaxSpeed,
//                 MaxAngularRate,
//                 90));

//     /// Operator
//     operator.b().whileTrue(indexer.index().alongWith(intake.intake()));

//     operator.a().whileTrue(shooter.shootPrecent(1));

//     // operator.rightTrigger().whileTrue(shooter.shoot(3000));
//     // operator.rightTrigger().whileTrue(new AutoAimShooter(drive, vision,
//     // shooter));

//     operator.x().whileTrue(shooter.shoot(4000));
//     operator.y().whileTrue(shooter.shoot(6000));

//     operator.leftBumper().whileTrue(indexer.ohShit().alongWith(intake.out()));

//     operator.leftTrigger().whileTrue(indexer.indexFlow().alongWith(intake.intake()));

//     // make the justIndexer and the feed command run together w/ out the
//     // feedtoshooter command

//     operator.rightBumper().whileTrue(indexer.feedRunOnce()).onFalse(indexer.stopFeedRunOnce());
//     operator
//         .rightBumper()
//         .whileTrue(indexer.justIndexerRunOnce())
//         .onFalse(indexer.stopJustIndexerRunOnce());

//     operator.povUp().onTrue(shooter.stepShroud(5));

//     operator.povDown().onTrue(shooter.stepShroud(-5));

//     operator.povLeft().onTrue(shooter.shroud(0));

//     operator.povRight().onTrue(shooter.shroud(13));

//     // Testing buttons: increment/decrement shooter RPM
//     operator
//         .start()
//         .onTrue(
//             Commands.runOnce(
//                 () -> {
//                   testShooterRPM = Math.min(testShooterRPM + RPM_INCREMENT, RPM_MAX);
//                   shooter.shoot(testShooterRPM).schedule();
//                 }));

//     operator
//         .back()
//         .onTrue(
//             Commands.runOnce(
//                 () -> {
//                   testShooterRPM = Math.max(testShooterRPM - RPM_INCREMENT, RPM_MIN);
//                   shooter.shoot(testShooterRPM).schedule();
//                 }));

//   }