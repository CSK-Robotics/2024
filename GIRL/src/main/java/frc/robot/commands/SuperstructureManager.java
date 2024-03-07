package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SuperstructureManager {
    private final Arm m_arm;
    private final Climber m_climber;
    private final Shooter m_shooter;
    private final Intake m_intake;

    public SuperstructureManager(Arm arm, Climber climber, Shooter shooter, Intake intake) {
        this.m_arm = arm;
        this.m_climber = climber;
        this.m_shooter = shooter;
        this.m_intake = intake;
    }
/* 
    public Command groundIntake(boolean direction) {
        return direction ? this.m_intake.run(direction).alongWith(this.m_climber.stow().andThen(this.m_arm.handoff()))
                .andThen(this.m_shooter.handoff(direction).alongWith(this.m_intake.handoff(direction)))
                : this.m_climber.stow().andThen(this.m_arm.handoff())
                        .andThen(this.m_shooter.handoff(direction).alongWith(this.m_intake.handoff(direction)))
                        .andThen(this.m_intake.run(direction));
    }
*/
    public Command sourceIntake(boolean direction) {
        return this.m_arm.source().andThen(this.m_shooter.source());
    }

    public Command speakerShot(double range, boolean high) {
        if (hasGamePiece()) {
            return this.m_arm.shoot(0, 0, Arm.Goal.SPEAKER);
        }
        return null;
    }

    public Command ampShot(double range) {
        if (hasGamePiece()) {
            return this.m_arm.amp().andThen(this.m_shooter.shoot(0, null));
        }
        return null;
    }

    /*public Command climb(BooleanSupplier inPosition) {
        return this.m_climber.stow().andThen(this.m_arm.climb())
                .andThen(this.m_climber.climb().beforeStarting(new WaitUntilCommand(inPosition)));
    }*/

    /*public Command trap(BooleanSupplier inPosition) {
        return this.climb(inPosition);
    }*/

    public boolean hasGamePiece() {
        return m_intake.hasGamePiece() || m_shooter.hasGamePiece();
    }
}
