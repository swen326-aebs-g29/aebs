package aebs.simulator.integration;

import Implementions.WheelSpeedReading;
import Interfaces.Controls.BrakingControlInterface;
import Interfaces.Controls.IDriverInterface;
import aebs.simulator.world.WorldState;
import main.BrakingControlModule;
import nz.ac.vuw.swen326.aebs.core.AEBSController;
import nz.ac.vuw.swen326.aebs.core.ControllerDecision;
import nz.ac.vuw.swen326.aebs.core.SensorSnapshot;

public final class SimulatorAebsBridge {
    private final SimulatorSensorAdapter sensorAdapter = new SimulatorSensorAdapter();
    private final BridgeBrakeActuator brakeActuator = new BridgeBrakeActuator();
    private final BridgeDriverInterface driverInterface;
    private final AEBSController controller;

    private volatile WheelSpeedReading[] latestWheelReadings = new WheelSpeedReading[0];
    private volatile ControllerDecision lastDecision = ControllerDecision.idle("startup");

    public SimulatorAebsBridge(WorldState world) {
        this.driverInterface = new BridgeDriverInterface(world);
        this.controller = new AEBSController(
                new BrakingControlModule(brakeActuator, driverInterface),
                () -> latestWheelReadings
        );
    }

    public double update(
            aebs.simulator.perception.RadarReading[] radarReadings,
            aebs.simulator.perception.CameraReading[] cameraReadings,
            aebs.simulator.perception.WheelSpeedReading[] wheelSpeedReadings,
            boolean sensorsHealthy
    ) {
        latestWheelReadings = sensorAdapter.toCoreWheel(wheelSpeedReadings);
        SensorSnapshot snapshot = sensorAdapter.toCoreSnapshot(
                radarReadings,
                cameraReadings,
                wheelSpeedReadings,
                sensorsHealthy
        );

        lastDecision = controller.onSensorData(snapshot);
        if (!lastDecision.braking()) {
            brakeActuator.applyBrake(0.0);
        }

        if (lastDecision.braking()) {
            driverInterface.showStatus(lastDecision.reason());
        }

        return brakeActuator.brakeLevel();
    }

    public double disable() {
        lastDecision = ControllerDecision.idle("disabled");
        brakeActuator.applyBrake(0.0);
        driverInterface.showStatus("disabled");
        return 0.0;
    }

    public void reset() {
        latestWheelReadings = new WheelSpeedReading[0];
        disable();
    }

    public ControllerDecision lastDecision() {
        return lastDecision;
    }

    private static double clamp(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }

    private static final class BridgeBrakeActuator implements BrakingControlInterface {
        private volatile double brakeLevel = 0.0;

        @Override
        public void applyBrake(double brakeLevel) {
            this.brakeLevel = clamp(brakeLevel);
        }

        public double brakeLevel() {
            return brakeLevel;
        }
    }

    private static final class BridgeDriverInterface implements IDriverInterface {
        private final WorldState world;

        private BridgeDriverInterface(WorldState world) {
            this.world = world;
        }

        @Override
        public void showStatus(String statusMessage) {
            world.setDriverBrakeAlert(false);
        }

        @Override
        public void feedback(String feedbackMessage) {
            world.setDriverBrakeAlert(false);
        }

        @Override
        public void showWarning(String warningMessage) {
            world.setDriverBrakeAlert(true);
        }

        @Override
        public void playSound(String soundId) {
            // The simulator UI already renders alert state; no extra action needed here.
        }

        @Override
        public void setControl() {
            world.setDriverBrakeAlert(true);
        }
    }
}
