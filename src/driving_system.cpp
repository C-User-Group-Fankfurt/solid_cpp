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
};


using MetresPerSquareSecond = double;
struct Acceleration {
  MetresPerSquareSecond value{0};
};

class PowerTrain final : public Actor {
 public:
  explicit PowerTrain(const Acceleration& acceleration_limit) : acceleration_limit(acceleration_limit) {}
  void control_vehicle(const Trajectory &) override {};
 private :
  Acceleration acceleration_limit;
};


class Brake final : public Actor {
 public:
  explicit Brake(const Acceleration& deceleration_limit) : deceleration_limit(deceleration_limit) {};
  void control_vehicle(const Trajectory &) override {};
 private:
  Acceleration deceleration_limit;
};


using NewtonMetre = double;
struct Torque {
  NewtonMetre value{0};
};

class SteeringWheel final : public Actor {
 public:
  explicit SteeringWheel(const Torque& torque_limit) : torque_limit(torque_limit) {}
  void control_vehicle(const Trajectory &) override {};
 private:
  Torque torque_limit;
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

  auto power_train = std::make_shared<PowerTrain>(
      Acceleration{MetresPerSquareSecond(13.6)});
  auto brake = std::make_shared<Brake>(
      Acceleration{MetresPerSquareSecond{21.0}});
  auto steering_wheel = std::make_shared<SteeringWheel>(
      Torque{NewtonMetre{3.0}});

  DrivingSystem driving_system(sensor, planner,
                               {power_train, brake, steering_wheel});
  driving_system.one_cycle();
}
