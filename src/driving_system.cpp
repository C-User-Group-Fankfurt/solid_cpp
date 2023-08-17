#include <memory>
#include <vector>

struct EnvironmentModel {};
class Sensor {
 public:
  EnvironmentModel model_environment() {
    return EnvironmentModel{};
  }
};

struct Trajectory {};
class Planner {
 public:
  Trajectory plan_vehicle_behavior(const EnvironmentModel &environment) {
    return Trajectory{};
  }
};

class Actor {
 public:
  virtual ~Actor() = default;
  virtual void follow_trajectory(const Trajectory &trjectory) = 0;
};

class DrivingSystem {
 public:
  DrivingSystem(std::shared_ptr<Sensor> sensor,
                std::shared_ptr<Planner> planner,
                std::vector<std::shared_ptr<Actor>> actors) :
      sensor(std::move(sensor)),
      planner(std::move(planner)),
      actors(std::move(actors)) {}

  void one_cycle() {
    auto environment_model = sensor->model_environment();
    auto vehicle_trajectory = planner->plan_vehicle_behavior(environment_model);
    act(vehicle_trajectory);
  }

 private:
  std::shared_ptr<Sensor> sensor;
  std::shared_ptr<Planner> planner;
  std::vector<std::shared_ptr<Actor>> actors;

  void act(const Trajectory &vehicle_trajectory) {
    for (auto &actor : actors) {
      actor->follow_trajectory(vehicle_trajectory);
    }
  }
};

class Brake : public Actor {
  // This might have a function to de-/activate emergency braking
  void follow_trajectory(const Trajectory &trajectory) override {
  }
};

class SteeringWheel : public Actor {
  void follow_trajectory(const Trajectory &trajectory) override {
  }
};

class TurnIndicator : public Actor {
  // This might need the environment model additionally to detect turns
  void follow_trajectory(const Trajectory &trajectory) override {
  }
};

int main(int, char **) {
  auto sensor = std::make_shared<Sensor>();
  auto planner = std::make_shared<Planner>();

  auto brake = std::make_shared<Brake>();
  auto steering_wheel = std::make_shared<SteeringWheel>();
  auto turn_indicator = std::make_shared<TurnIndicator>();

  DrivingSystem driving_system(
      sensor, planner,
      {brake, steering_wheel, turn_indicator});
  driving_system.one_cycle();
}