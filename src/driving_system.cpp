#include <memory>
#include <vector>

struct EnvironmentModel {};
class Sensor {
 public:
  EnvironmentModel model_environment() {
    return EnvironmentModel{};
  }
};

struct SteeringAngle {};
struct Acceleration {};
struct Torque {};

class Actor {
  void set_steering_angle(const SteeringAngle & /*steering_angle*/) {}
  void set_power_train(const Acceleration & /*acceleration*/) {}
  void set_brake_torque(const Torque & /*brake_torque*/) {}
};

class Trajectory {
 public:
  void drive(Actor & /*actor*/) {}
};

class Planner {
 public:
  Trajectory plan_vehicle_behavior(const EnvironmentModel &/*environment*/) {
    return Trajectory{};
  }

  void drive(Actor & /*actor*/) {}
};

class DrivingSystem {
 public:
  DrivingSystem(std::shared_ptr<Sensor> sensor,
                std::shared_ptr<Planner> planner,
                std::shared_ptr<Actor> actor) :
      sensor(std::move(sensor)),
      planner(std::move(planner)),
      actor(std::move(actor)) {}

  void one_cycle() {
    auto environment_model = sensor->model_environment();
    auto vehicle_trajectory = planner->plan_vehicle_behavior(environment_model);
    vehicle_trajectory.drive(*actor);
  }

 private:
  std::shared_ptr<Sensor> sensor;
  std::shared_ptr<Planner> planner;
  std::shared_ptr<Actor> actor;
};

int main(int, char **) {
  auto sensor = std::make_shared<Sensor>();
  auto planner = std::make_shared<Planner>();
  auto actor = std::make_shared<Actor>();

  DrivingSystem driving_system(
      sensor, planner, actor);
  driving_system.one_cycle();
}