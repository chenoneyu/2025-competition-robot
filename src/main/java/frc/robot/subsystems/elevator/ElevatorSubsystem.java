package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

@Getter
public class ElevatorSubsystem extends SubsystemBase {

    public enum WantedState {
        OFF,
        BOTTOM,
        INTAKER_INTAKE,
        INTAKER_AVOID,
        FUNNEL_INTAKE,
        L1,
        L2,
        L3,
        L4,
        ZEROING
    }

    public enum SystemState {
        OFF,
        BOTTOM_STAYING,
        INTAKER_INTAKING,
        INTAKER_AVOIDING,
        FUNNEL_INTAKING,
        L1_STAYING,
        L2_STAYING,
        L3_STAYING,
        L4_STAYING,
        ZEROING
    }

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private WantedState wantedState = WantedState.OFF;
    private SystemState systemState = SystemState.OFF;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        // process inputs
        SystemState newState = handleStateTransition();
        if (newState != systemState) {
            Logger.recordOutput("Elevator/SystemState", newState.toString());
            systemState = newState;
        }

        // set movements based on state
        switch (systemState) {
            case BOTTOM_STAYING:
                io.setElevatorTarget(RobotConstants.ElevatorConstants.BOTTOM_HEIGHT);
                break;
            case INTAKER_INTAKING:
                io.setElevatorTarget(RobotConstants.ElevatorConstants.INTAKE_INTAKING_HEIGHT);
                break;
            case INTAKER_AVOIDING:
                io.setElevatorTarget(RobotConstants.ElevatorConstants.INTAKE_AVOIDING_HEIGHT);
                break;
            case FUNNEL_INTAKING:
                io.setElevatorTarget(RobotConstants.ElevatorConstants.FUNNEL_INTAKING_HEIGHT);
                break;
            case L1_STAYING:
                io.setElevatorTarget(RobotConstants.ElevatorConstants.L1_HEIGHT);
                break;
            case L2_STAYING:
                io.setElevatorTarget(RobotConstants.ElevatorConstants.L2_HEIGHT);
                break;
            case L3_STAYING:
                io.setElevatorTarget(RobotConstants.ElevatorConstants.L3_HEIGHT);
                break;
            case L4_STAYING:
                io.setElevatorTarget(RobotConstants.ElevatorConstants.L4_HEIGHT);
                break;
            case ZEROING:
                io.zeroingElevator();
                break;
            case OFF:
                io.elevatorOff();
                break;
            default:
                io.elevatorOff();
                break;
        }
    }


    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case BOTTOM -> {
                if (systemState == SystemState.INTAKER_INTAKING) {
                    if (true/*is intaker motor off*/){
                        yield SystemState.BOTTOM_STAYING;
                    }
                    yield SystemState.INTAKER_INTAKING;
                }
                if (systemState == SystemState.FUNNEL_INTAKING){
                    if (true/*is funnel motor off*/){
                        yield SystemState.BOTTOM_STAYING;
                    }
                    yield SystemState.FUNNEL_INTAKING;
                }
                if (systemState == SystemState.L1_STAYING || systemState == SystemState.L2_STAYING ||
                        systemState == SystemState.L3_STAYING || systemState == SystemState.L4_STAYING) {
                    if (true/*is shooter motor off*/) {
                        yield SystemState.BOTTOM_STAYING;
                    }
                    yield systemState;
                }
                if (systemState == SystemState.INTAKER_AVOIDING) {
                    if (true/*is intaker come in or out finished*/){
                        yield SystemState.BOTTOM_STAYING;
                    }
                    yield SystemState.INTAKER_AVOIDING;
                }
                if (systemState == SystemState.ZEROING) {
                    yield SystemState.ZEROING;
                }
            }
            case L1 -> {
                if (systemState == SystemState.BOTTOM_STAYING){
                    yield SystemState.L1_STAYING;
                }
                if (systemState == SystemState.INTAKER_INTAKING) {
                    if (true/*is intaker motor off*/){
                        yield SystemState.L1_STAYING;
                    }
                    yield SystemState.INTAKER_INTAKING;
                }
                if (systemState == SystemState.FUNNEL_INTAKING){
                    if (true/*is funnel motor off*/){
                        yield SystemState.L1_STAYING;
                    }
                    yield SystemState.FUNNEL_INTAKING;
                }
                if (systemState == SystemState.L1_STAYING || systemState == SystemState.L2_STAYING ||
                        systemState == SystemState.L3_STAYING || systemState == SystemState.L4_STAYING) {
                    if (true/*is shooter motor off*/) {
                        yield SystemState.L1_STAYING;
                    }
                    yield systemState;
                }
                if (systemState == SystemState.INTAKER_AVOIDING) {
                    if (true/*is intaker come in or out finished*/){
                        yield SystemState.L1_STAYING;
                    }
                    yield SystemState.INTAKER_AVOIDING;
                }
                if (systemState == SystemState.ZEROING) {
                    yield SystemState.ZEROING;
                }
            }
            case L2 -> {
                if (systemState == SystemState.BOTTOM_STAYING){
                    yield SystemState.L2_STAYING;
                }
                if (systemState == SystemState.INTAKER_INTAKING) {
                    if (true/*is intaker motor off*/){
                        yield SystemState.L2_STAYING;
                    }
                    yield SystemState.INTAKER_INTAKING;
                }
                if (systemState == SystemState.FUNNEL_INTAKING){
                    if (true/*is funnel motor off*/){
                        yield SystemState.L2_STAYING;
                    }
                    yield SystemState.FUNNEL_INTAKING;
                }
                if (systemState == SystemState.L1_STAYING || systemState == SystemState.L2_STAYING ||
                        systemState == SystemState.L3_STAYING || systemState == SystemState.L4_STAYING) {
                    if (true/*is shooter motor off*/) {
                        yield SystemState.L2_STAYING;
                    }
                    yield systemState;
                }
                if (systemState == SystemState.INTAKER_AVOIDING) {
                    if (true/*is intaker come in or out finished*/){
                        yield SystemState.L2_STAYING;
                    }
                    yield SystemState.INTAKER_AVOIDING;
                }
                if (systemState == SystemState.ZEROING) {
                    yield SystemState.ZEROING;
                }
            }
            case L3 -> {
                if (systemState == SystemState.BOTTOM_STAYING){
                    yield SystemState.L3_STAYING;
                }
                if (systemState == SystemState.INTAKER_INTAKING) {
                    if (true/*is intaker motor off*/){
                        yield SystemState.L3_STAYING;
                    }
                    yield SystemState.INTAKER_INTAKING;
                }
                if (systemState == SystemState.FUNNEL_INTAKING){
                    if (true/*is funnel motor off*/){
                        yield SystemState.L3_STAYING;
                    }
                    yield SystemState.FUNNEL_INTAKING;
                }
                if (systemState == SystemState.L1_STAYING || systemState == SystemState.L2_STAYING ||
                        systemState == SystemState.L3_STAYING || systemState == SystemState.L4_STAYING) {
                    if (true/*is shooter motor off*/) {
                        yield SystemState.L3_STAYING;
                    }
                    yield systemState;
                }
                if (systemState == SystemState.INTAKER_AVOIDING) {
                    if (true/*is intaker come in or out finished*/){
                        yield SystemState.L3_STAYING;
                    }
                    yield SystemState.INTAKER_AVOIDING;
                }
                if (systemState == SystemState.ZEROING) {
                    yield SystemState.ZEROING;
                }
            }
            case L4 -> {
                if (systemState == SystemState.BOTTOM_STAYING){
                    yield SystemState.L4_STAYING;
                }
                if (systemState == SystemState.INTAKER_INTAKING) {
                    if (true/*is intaker motor off*/){
                        yield SystemState.L4_STAYING;
                    }
                    yield SystemState.INTAKER_INTAKING;
                }
                if (systemState == SystemState.FUNNEL_INTAKING){
                    if (true/*is funnel motor off*/){
                        yield SystemState.L4_STAYING;
                    }
                    yield SystemState.FUNNEL_INTAKING;
                }
                if (systemState == SystemState.L1_STAYING || systemState == SystemState.L2_STAYING ||
                        systemState == SystemState.L3_STAYING || systemState == SystemState.L4_STAYING) {
                    if (true/*is shooter motor off*/) {
                        yield SystemState.L4_STAYING;
                    }
                    yield systemState;
                }
                if (systemState == SystemState.INTAKER_AVOIDING) {
                    if (true/*is intaker come in or out finished*/){
                        yield SystemState.L4_STAYING;
                    }
                    yield SystemState.INTAKER_AVOIDING;
                }
                if (systemState == SystemState.ZEROING) {
                    yield SystemState.ZEROING;
                }
            }
            case ZEROING -> {
                if (systemState == SystemState.BOTTOM_STAYING){
                    yield SystemState.ZEROING;
                }
                if (systemState == SystemState.INTAKER_INTAKING) {
                    if (true/*is intaker motor off*/){
                        yield SystemState.ZEROING;
                    }
                    yield SystemState.INTAKER_INTAKING;
                }
                if (systemState == SystemState.FUNNEL_INTAKING){
                    if (true/*is funnel motor off*/){
                        yield SystemState.ZEROING;
                    }
                    yield SystemState.FUNNEL_INTAKING;
                }
                if (systemState == SystemState.L1_STAYING || systemState == SystemState.L2_STAYING ||
                        systemState == SystemState.L3_STAYING || systemState == SystemState.L4_STAYING) {
                    if (true/*is shooter motor off*/) {
                        yield SystemState.ZEROING;
                    }
                    yield systemState;
                }
                if (systemState == SystemState.INTAKER_AVOIDING) {
                    if (true/*is intaker come in or out finished*/){
                        yield SystemState.ZEROING;
                    }
                    yield SystemState.INTAKER_AVOIDING;
                }
            }
            case INTAKER_AVOID -> {
                if (systemState == SystemState.BOTTOM_STAYING){
                    yield SystemState.INTAKER_AVOIDING;
                }
                if (systemState == SystemState.INTAKER_INTAKING) {
                    if (true/*is intaker motor off*/){
                        yield SystemState.INTAKER_AVOIDING;
                    }
                    yield SystemState.INTAKER_INTAKING;
                }
                if (systemState == SystemState.FUNNEL_INTAKING){
                    if (true/*is funnel motor off*/){
                        yield SystemState.INTAKER_AVOIDING;
                    }
                    yield SystemState.FUNNEL_INTAKING;
                }
                if (systemState == SystemState.L1_STAYING || systemState == SystemState.L2_STAYING ||
                        systemState == SystemState.L3_STAYING || systemState == SystemState.L4_STAYING) {
                    if (true/*is shooter motor off*/) {
                        yield SystemState.INTAKER_AVOIDING;
                    }
                    yield systemState;
                }
                if(systemState == SystemState.ZEROING){
                    yield SystemState.ZEROING;
                }
            }
            case FUNNEL_INTAKE -> {
                if (systemState == SystemState.BOTTOM_STAYING){
                    if(true/*no things in the shooter*/){
                        yield SystemState.FUNNEL_INTAKING;
                    }
                    yield SystemState.BOTTOM_STAYING;
                }
                if (systemState == SystemState.INTAKER_INTAKING) {
                    if (true/*is intaker motor off*/ && true /*no things in the shooter*/){
                        yield SystemState.FUNNEL_INTAKING;
                    }
                    yield SystemState.INTAKER_INTAKING;
                }
                if (systemState == SystemState.L1_STAYING || systemState == SystemState.L2_STAYING ||
                        systemState == SystemState.L3_STAYING || systemState == SystemState.L4_STAYING) {
                    if (true/*is shooter motor off*/ && true /*no things in the shooter*/) {
                        yield SystemState.FUNNEL_INTAKING;
                    }
                    yield systemState;
                }
                if (systemState == SystemState.INTAKER_AVOIDING) {
                    if (true/*is intaker come in or out finished*/ && true /*no things in the shooter*/){
                        yield SystemState.FUNNEL_INTAKING;
                    }
                    yield SystemState.INTAKER_AVOIDING;
                }
                if(systemState == SystemState.ZEROING){
                    yield SystemState.ZEROING;
                }
            }
            case INTAKER_INTAKE -> {
                if (systemState == SystemState.BOTTOM_STAYING) {
                    if (true/*no things in the shooter*/) {
                        yield SystemState.INTAKER_INTAKING;
                    }
                    yield SystemState.BOTTOM_STAYING;
                }
                if (systemState == SystemState.FUNNEL_INTAKING) {
                    if (true/*is funnel motor off*/ && true /*no things in the shooter*/) {
                        yield SystemState.INTAKER_INTAKING;
                    }
                    yield SystemState.FUNNEL_INTAKING;
                }
                if (systemState == SystemState.L1_STAYING || systemState == SystemState.L2_STAYING ||
                        systemState == SystemState.L3_STAYING || systemState == SystemState.L4_STAYING) {
                    if (true/*is shooter motor off*/ && true /*no things in the shooter*/) {
                        yield SystemState.INTAKER_AVOIDING;
                    }
                    yield systemState;
                }
                if (systemState == SystemState.INTAKER_AVOIDING) {
                    if (true/*is intaker come in or out finished*/ && true /*no things in the shooter*/) {
                        yield SystemState.INTAKER_INTAKING;
                    }
                    yield SystemState.INTAKER_AVOIDING;
                }
                if (systemState == SystemState.ZEROING) {
                    yield SystemState.ZEROING;
                }
            }
            case OFF -> SystemState.OFF;
        };
    }


}
