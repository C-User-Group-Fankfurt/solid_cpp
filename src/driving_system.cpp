#include <memory>
#include <vector>

struct EnvironmentModel {};
class Sensor {
 public:
  EnvironmentModel model_environment() {
    return EnvironmentModel{};
  }
};

struct Trajectory {
  using TrajectoryData = int;
  TrajectoryData internals{};
};

class Planner {
 public:
  Trajectory plan_vehicle_behavior(const EnvironmentModel &/*environment*/) {
    return Trajectory{};
  }
};

class Actor {
 public:
  virtual ~Actor() = default;
  virtual void control_vehicle(const Trajectory & /*trajectory*/) = 0;
};

class PowerTrain final : public Actor {
 public:
  void control_vehicle(const Trajectory &) override {};
};

class Brake final : public Actor {
 public:
  void control_vehicle(const Trajectory &) override {};
};

class SteeringWheel final : public Actor {
 public:
  void control_vehicle(const Trajectory &) override {};
};

class DrivingSystem {
 public:
  using Actors = std::vector<std::shared_ptr<Actor>>;

  DrivingSystem(std::shared_ptr<Sensor> sensor,
                std::shared_ptr<Planner> planner,
                Actors actors) :
      sensor(std::move(sensor)),
      planner(std::move(planner)),
      actors(std::move(actors)) {};

  void one_cycle() {
    auto environment_model = sensor->model_environment();
    auto vehicle_trajectory = planner->plan_vehicle_behavior(environment_model);

    for (auto &actor : actors)
      actor->control_vehicle(vehicle_trajectory);
  }

 private:
  std::shared_ptr<Sensor> sensor;
  std::shared_ptr<Planner> planner;
  Actors actors;
};

int main() {
  auto sensor = std::make_shared<Sensor>();
  auto planner = std::make_shared<Planner>();

  auto power_train = std::make_shared<PowerTrain>();
  auto brake = std::make_shared<Brake>();
  auto steering_wheel = std::make_shared<SteeringWheel>();

  DrivingSystem driving_system(sensor, planner,
                               {power_train, brake, steering_wheel});
  driving_system.one_cycle();
}
