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

class Actor {
 public:
  virtual ~Actor() = default;
  virtual void control_vehicle(const Trajectory & /*trajectory*/) = 0;
  virtual void set_limit(double /*limit*/) = 0;
};

class PowerTrain final : public Actor {
 public:
  void control_vehicle(const Trajectory &) override {};
  void set_limit(double /*acceleration_limit*/) override {}
};

class Brake final : public Actor {
 public:
  void control_vehicle(const Trajectory &) override {};
  void set_limit(double deceleration_limit) override {
    max_acceleration = deceleration_limit;
  }
 private:
  double max_acceleration{0};
};

class SteeringWheel final : public Actor {
 public:
  void control_vehicle(const Trajectory &) override {};
  void set_limit(double torque_limit) override {
    max_torque = torque_limit;
  }
 private:
  double max_torque{0};
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

int main(int, char **) {
  auto sensor = std::make_shared<Sensor>();
  auto planner = std::make_shared<Planner>();

  auto power_train = std::make_shared<PowerTrain>();
  auto brake = std::make_shared<Brake>();
  brake->set_limit(20.0);
  auto steering_wheel = std::make_shared<SteeringWheel>();
  steering_wheel->set_limit(3);

  DrivingSystem driving_system(sensor, planner,
                               {power_train, brake, steering_wheel});
  driving_system.one_cycle();
}