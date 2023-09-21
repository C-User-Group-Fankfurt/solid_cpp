#include <memory>
#include <vector>

struct EnvironmentModel {};
class Sensor {
 public:
  EnvironmentModel model_environment() {
    return EnvironmentModel{};
  }
};

namespace control_actuators {
using PowerTrainConnection = int;
using BrakeConnection = int;
using SteeringConnection = int;
using ControlSignals = int;

void control_power_train(const ControlSignals &, PowerTrainConnection &) {}
void control_brake(const ControlSignals &, BrakeConnection &) {}
void control_steering_wheel(const ControlSignals &, SteeringConnection &) {}
}

class Trajectory {
 public:
  void control_vehicle() {
    using namespace control_actuators;
    control_power_train(internals, power_train_connection);
    control_brake(internals, brake_connection);
    control_steering_wheel(internals, steering_wheel_connection);
  };
 private:
  using TrajectoryData = int;
  TrajectoryData internals{};

  control_actuators::PowerTrainConnection power_train_connection{};
  control_actuators::BrakeConnection brake_connection{};
  control_actuators::SteeringConnection steering_wheel_connection{};
};

class Planner {
 public:
  Trajectory plan_vehicle_behavior(const EnvironmentModel &/*environment*/) {
    return Trajectory{};
  }
};

class DrivingSystem {
 public:
  DrivingSystem(std::shared_ptr<Sensor> sensor,
                std::shared_ptr<Planner> planner) :
      sensor(std::move(sensor)),
      planner(std::move(planner)) {};

  void one_cycle() {
    auto environment_model = sensor->model_environment();
    auto vehicle_trajectory = planner->plan_vehicle_behavior(environment_model);
    vehicle_trajectory.control_vehicle();
  }

 private:
  std::shared_ptr<Sensor> sensor;
  std::shared_ptr<Planner> planner;
};

int main() {
  auto sensor = std::make_shared<Sensor>();
  auto planner = std::make_shared<Planner>();

  DrivingSystem driving_system(
      sensor, planner);
  driving_system.one_cycle();
}