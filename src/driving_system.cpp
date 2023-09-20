#include <memory>
#include <vector>

struct EnvironmentModel {};
class Sensor {
 public:
  EnvironmentModel model_environment() {
    return EnvironmentModel{};
  }
};

class Trajectory {
};

class Planner {
 public:
  Trajectory plan_vehicle_behavior(const EnvironmentModel &/*environment*/) {
    return Trajectory{};
  }
};

class Actors {
 public:
  void control_power_train(const Trajectory & /*trajectory*/) {}
  void control_brake(const Trajectory & /*trajectory*/) {}
  void control_steering_wheel(const Trajectory & /*trajectory*/) {}
};

class DrivingSystem {
 public:
  DrivingSystem(std::shared_ptr<Sensor> sensor,
                std::shared_ptr<Planner> planner,
                std::shared_ptr<Actors> actor) :
      sensor(std::move(sensor)),
      planner(std::move(planner)),
      actor(std::move(actor)) {};

  void one_cycle() {
    auto environment_model = sensor->model_environment();
    auto vehicle_trajectory = planner->plan_vehicle_behavior(environment_model);
    actor->control_brake(vehicle_trajectory);
    actor->control_power_train(vehicle_trajectory);
    actor->control_steering_wheel(vehicle_trajectory);
  }

 private:
  std::shared_ptr<Sensor> sensor;
  std::shared_ptr<Planner> planner;
  std::shared_ptr<Actors> actor;
};

int main() {
  auto sensor = std::make_shared<Sensor>();
  auto planner = std::make_shared<Planner>();
  auto actor = std::make_shared<Actors>();

  DrivingSystem driving_system(sensor, planner, actor);
  driving_system.one_cycle();
}