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
 private:
  // ...
};

class DrivingSystem {
 public:
  DrivingSystem(std::shared_ptr<Sensor> sensor,
                std::shared_ptr<Planner> planner,
                std::shared_ptr<Actors> actors) :
      sensor(std::move(sensor)),
      planner(std::move(planner)),
      actors(std::move(actors)) {};

  void one_cycle() {
    auto environment_model = sensor->model_environment();
    auto vehicle_trajectory = planner->plan_vehicle_behavior(environment_model);
    actors->control_brake(vehicle_trajectory);
    actors->control_power_train(vehicle_trajectory);
    actors->control_steering_wheel(vehicle_trajectory);
  }

 private:
  std::shared_ptr<Sensor> sensor;
  std::shared_ptr<Planner> planner;
  std::shared_ptr<Actors> actors;
};

int main() {
  auto sensor = std::make_shared<Sensor>();
  auto planner = std::make_shared<Planner>();
  auto actors = std::make_shared<Actors>();

  DrivingSystem driving_system(sensor, planner, actors);
  driving_system.one_cycle();
}
