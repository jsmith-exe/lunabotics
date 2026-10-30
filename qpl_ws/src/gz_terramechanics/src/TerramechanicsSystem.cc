#include "gz_terramechanics/TerramechanicsSystem.hh"
#include <omp.h>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/Link.hh>
#include <gz/transport/Node.hh>
#include <yaml-cpp/yaml.h>
// #include <gz/msgs/double_v.pb.h>
#include <gz/msgs/float_v.pb.h>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/components/RaycastData.hh>
#include <gz/sim/components/Physics.hh>
#include <gz/sim/components/ContactSensor.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/physics/Events.hh>

#include <set>
#include <iomanip> // debug only

// Queste variabili sono dichiarate nel codice ma non vengono mai utilizzate: counter_ non viene mai letto né scritto, collision_name non viene mai impostato
// né letto, prev_contact_name viene solo inizializzato ma mai usato, e WheelState::v viene calcolato alla riga 960 ma il suo valore non viene mai letto da 
// nessuna parte.  


using namespace gz_terramechanics;


class TerramechanicsSystem::TerramechanicsSystemPrivate

{
public:

  enum class PluginState {
    UNINITIALIZED,
    LOADING,
    INITIALIZED,
    RUNNING,
    SHUTTING_DOWN
  } plugin_state_ = PluginState::UNINITIALIZED;
  
  struct Options {
      std::string bulldozing_resistance = "neglect";
      bool use_compact_model;
      bool passive_plugin;  // true -> the plugin does not interact with gazebo (apply forces, modify frictions, ecc), but
                            // only publishes its results (confrontation purposes)
  };  

  struct WheelParams {
    std::string type;
    double r;   // wheel inner radius, [m]
    double h_g; // grousers height, [m]
    double r_s; // shearing radius, [m]
    double b;   // wheel width, [m]
    double mu;  // grousers area ratio, [-]
  };

  struct SoilParams {
    std::string name;
    double k_c;   // cohesive modulus, [N/m^(n+1)]
    double k_phi; // frictional modulus, [N/m^(n+2)]
    double k;     // sinkage modulus, could be expressed as (k_c/b+k_phi), [N/m^(n+2)]
    double c;     // cohesion, [Pa]
    double phi;   // friction angle, [rad]
    double K;     // shear modulus, [m]
    double rho;   // density, [N/m3]
    double n;     // sinkage exponent, could be expressed in terms of n0, n1, n2, [-]
    double X_c;   // destructive angle, [rad]
    double n0 = NAN, n1 = NAN, n2 = NAN;
  };

struct TunedParams {
    double a0 = 0, a1 = 0;
    double b0 = 0, b1 = 0;
    double d0 = 1, d1 = 0.5;
    double n0 = NAN, n1 = NAN, n2 = NAN;
    double n_effective = 0;
};

  struct WheelState {
    double v_x;     // wheel longitudinal velocity, [m/s]
    double v_y;     // wheel lateral velocity, [m/s]
    double v;       // wheel velocity, [m/s]
    double omega;   // wheel angular velocity, [rad/s]
    double s;       // slip/skid ratio, [-]
    double beta;    // slip angle, [rad]
  };

  struct TerramechanicsParams {
    double h_0;                 // wheel sinkage, [m]
    std::vector<double> h;      // point depht (referred to r_s), [m]
    double theta_f;             // entry angle, [rad]
    double theta_r;             // exit angle, [rad]
    double theta_m;             // maximum normal stress angle, [rad]
    std::vector<double> theta;  // point coordinate, [rad]
    std::vector<double> theta_e;// equivalent front angle for rear region points, [rad]
    double theta_0;             // shear transition angle in skid, [rad]
    std::vector<double> sigma;  // normal stress, [Pa]
    std::vector<double> tau;    // shear stress, [Pa]
    std::vector<double> tau_t;  // tangential shear stress, [Pa]
    std::vector<double> tau_l;  // lateral shear stress, [Pa]
    std::vector<double> j;      // shear displacement, [m]
    std::vector<double> j_t;    // tangential shear displacement, [m]
    std::vector<double> j_l;    // lateral shear displacement, [m]
    std::vector<double> v_jt;   // tangential shear velocity, [m/s]
    std::vector<double> v_jl;   // lateral shear velocity, [m/s]
  };

  struct Forces {
    double W;                 // wheel vertical load (constant), [N]
    std::vector<double> R_b;  // bulldozing resistance, [Pa*m]
    double F_x;               // force acting on the wheel along X axis, [N]
    double F_y;               // force acting on the wheel along Y axis, [N]
    double F_z;               // force acting on the wheel along Z axis, [N]
    double M_x;               // [N*m]
    double M_y;               // driving moment (around Y axis), [N*m]
    double M_z;               // [N*m]
  };

  struct WheelData {
    std::string name;
    gz::sim::Entity linkEntity{gz::sim::kNullEntity};
    gz::sim::Entity steerEntity{gz::sim::kNullEntity};
    gz::sim::Entity rayEntity{gz::sim::kNullEntity};
    std::vector<gz::sim::Entity> contactSensorsCollisionEntity{gz::sim::kNullEntity};

    // std::string collision_name;
    // std::string prev_contact_name = "";
    gz::math::Quaterniond contact_frame_rot;
    // double link_mass = 0.0;
    // double f_signs[2] = {1.0, 1.0};    

    //WheelParams wheelParam;
    const WheelParams* wheelParam = nullptr;
    //SoilParams soilParam;
    const SoilParams* soilParam = nullptr;
    TunedParams tunedParam;
    WheelState stateParam;
    TerramechanicsParams terraParam;
    Forces forces;

    int iters_without_contact = 10;
    bool in_contact = false;
    // std::vector<std::string> collision_names;
    std::string terrain_name;

    gz::transport::Node::Publisher forcesPub;
    gz::transport::Node::Publisher statePub;

    void initVectors(int rim_pts) {
        terraParam.h.resize(rim_pts, 0.0);
        terraParam.theta.resize(rim_pts, 0.0);
        terraParam.theta_e.resize(rim_pts, 0.0);
        terraParam.sigma.resize(rim_pts, 0.0);
        terraParam.tau.resize(rim_pts, 0.0);
        terraParam.tau_t.resize(rim_pts, 0.0);
        terraParam.tau_l.resize(rim_pts, 0.0);
        terraParam.j.resize(rim_pts, 0.0);
        terraParam.j_t.resize(rim_pts, 0.0);
        terraParam.j_l.resize(rim_pts, 0.0);
        terraParam.v_jt.resize(rim_pts, 0.0);
        terraParam.v_jl.resize(rim_pts, 0.0);
        forces.R_b.resize(rim_pts, 0.0);
    }
  };

  int counter_ = 0;
  gz::sim::Entity modelEntity_{gz::sim::kNullEntity};
  gz::sim::Model model_{gz::sim::kNullEntity};

  static constexpr int num_wheels = 4;
  int rim_pts = 100;

  std::array<std::string, num_wheels> wheel_names;
  std::array<std::string, num_wheels> joint_names;
  // true where the drive joint axis points to the wheel's inside, so its spin is negative
  // when the rover moves forward (wheel_params.yaml: reversed_axis)
  std::array<bool, num_wheels> reversed_axis{};

  std::array<WheelData, num_wheels> wheels;

  WheelParams globalWheelParams;
  Options options;
  std::unordered_map<std::string, SoilParams> soils;
  std::vector<std::pair<std::string, std::string>> terrainSoilMap;  // pattern -> soil
  std::unordered_map<gz::sim::Entity, std::string> collisionsToTerrainMap;
  std::string defaultSoil;


  // Settings
  bool debug = false;
  bool publish_results = false;
  bool publish_intermediate_values = false;

  // Transport
  gz::transport::Node node;

  // Contact surface customization (zero friction on wheels)
  gz::common::ConnectionPtr contactSurfaceConnection;


  /* // World data
  double world_gravity = 9.81;
  double rover_dimensions[2] = {0.0, 0.0};
  std::vector<std::vector<double>> other_masses; */


  // Helper methods
  void initializePluginParam(const std::shared_ptr<const sdf::Element> &_sdf, gz::sim::EntityComponentManager &_ecm);
  void initializeTransport();
  bool initializeWheel(int wheel_idx, gz::sim::EntityComponentManager &_ecm);
  void initializeWheels(gz::sim::EntityComponentManager &_ecm);
  bool findSinkage(int wheel_idx);
  void computeContactGeometry(int wheel_idx);
  void computeStresses(int wheel_idx);
  void computeForces(int wheel_idx, double* F_z_out);
  // void computeWheelLoad(gz::sim::EntityComponentManager &_ecm, int wheel_idx);
  void setWheelParams(int wheel_idx);
  void setWheelStateParams(int wheel_idx, gz::sim::EntityComponentManager &_ecm);
  void setSoilParams(int wheel_idx,  gz::sim::EntityComponentManager &_ecm);
  std::string getTerrainBelow(int wheel_idx, gz::sim::EntityComponentManager &_ecm);
  void setTunedParams(int wheel_idx);
  void applyForce(int wheel_idx, gz::sim::EntityComponentManager &_ecm);
  void publishResults(int wheel_idx);
  void onUpdateFull(gz::sim::EntityComponentManager &_ecm);
  void onUpdateCompact(gz::sim::EntityComponentManager &_ecm);
  void getContactData(int wheel_idx, const gz::sim::EntityComponentManager &_ecm);

};


TerramechanicsSystem::TerramechanicsSystem() : dataPtr(std::make_unique<TerramechanicsSystemPrivate>())
{

}


TerramechanicsSystem::~TerramechanicsSystem() = default;


void TerramechanicsSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
    // Set OpenMP threads to match number of wheels
    omp_set_num_threads(dataPtr->num_wheels);

    // Check OpenMP status
    int num_threads = 0;
    #pragma omp parallel
    {
        #pragma omp master
        {
            num_threads = omp_get_num_threads();
            gzmsg << "[TerramechanicsSystem] OpenMP is enabled with " << num_threads << " threads" << std::endl;
        }
    }

    // Load function - called when plugin is loaded
    dataPtr->plugin_state_ = TerramechanicsSystemPrivate::PluginState::LOADING;
    gzmsg << "[TerramechanicsSystem] Loading plugin..." <<  std::endl;

    // Store model
    dataPtr->modelEntity_ = _entity;
    dataPtr->model_ = gz::sim::Model(_entity);

    // Initialize
    dataPtr->initializePluginParam(_sdf, _ecm);
    dataPtr->initializeWheels(_ecm);

    if (dataPtr->publish_results) {
        dataPtr->initializeTransport();
    }

    // Set friction to zero on wheel collisions via contact surface customization
    // (gz-sim8 equivalent of Gazebo Classic's SetMuPrimary(0)/SetMuSecondary(0)/SetMuTorsion(0))
    if (!dataPtr->options.passive_plugin)
    {
        // Collect all wheel collision entities into a set for fast lookup
        std::set<gz::sim::Entity> wheelCollisions;
        for (int i = 0; i < dataPtr->num_wheels; i++)
            for (const auto &col : dataPtr->wheels[i].contactSensorsCollisionEntity)
                if (col != gz::sim::kNullEntity)
                    wheelCollisions.insert(col);

        // Enable contact surface customization on each wheel collision
        for (const auto &col : wheelCollisions)
            _ecm.CreateComponent(col,
                gz::sim::components::EnableContactSurfaceCustomization(true));

        // Register callback to zero out friction before each physics step
        dataPtr->contactSurfaceConnection =
            _eventMgr.Connect<gz::sim::events::CollectContactSurfaceProperties>(
                [wheelCollisions, collisionsToTerrainMap=dataPtr->collisionsToTerrainMap](
                    const gz::sim::Entity &_col1,
                    const gz::sim::Entity &_col2,
                    const gz::math::Vector3d &/*_point*/,
                    const std::optional<gz::math::Vector3d> /*_force*/,
                    const std::optional<gz::math::Vector3d> /*_normal*/,
                    const std::optional<double> /*_depth*/,
                    const size_t /*_numContacts*/,
                    gz::physics::SetContactPropertiesCallbackFeature::
                        ContactSurfaceParams<gz::physics::FeaturePolicy3d> &_params)
                {
                    if ((wheelCollisions.count(_col1) || wheelCollisions.count(_col2))
                      && (collisionsToTerrainMap.find(_col1) != collisionsToTerrainMap.end() 
                        || collisionsToTerrainMap.find(_col2) != collisionsToTerrainMap.end()))
                    {
                        _params.frictionCoeff = 0.0;
                        _params.secondaryFrictionCoeff = 0.0;
                        _params.torsionalFrictionCoeff = 0.0; // not supported by DART
                    }
                });

        gzmsg << "[TerramechanicsSystem] Contact surface friction set to 0 for "
              << wheelCollisions.size() << " wheel collision(s)" << std::endl;

        gzmsg << "[TerramechanicsSystem] Plugin successfully loaded"<< std::endl;
    } else {
        gzmsg << "[TerramechanicsSystem] Plugin successfully loaded in passive mode" << std::endl;
    }

    dataPtr->plugin_state_ = TerramechanicsSystemPrivate::PluginState::INITIALIZED;

    /* auto worldEntity = _ecm.EntityByComponents(gz::sim::components::World());
    _ecm.CreateComponent(worldEntity, gz::sim::components::PhysicsCollisionDetector("bullet"));
    for (int i = 0; i < dataPtr->num_wheels; i++) {
        dataPtr->wheels[i].rayEntity = _ecm.CreateEntity();
        _ecm.CreateComponent(dataPtr->wheels[i].rayEntity, gz::sim::components::RaycastData());
        _ecm.CreateComponent(dataPtr->wheels[i].rayEntity, gz::sim::components::Pose());

        // Set ray direction once — always straight down
        auto rayComp = _ecm.Component<gz::sim::components::RaycastData>(dataPtr->wheels[i].rayEntity);
        if (rayComp) {
            gz::sim::components::RayInfo ray;
            ray.start = gz::math::Vector3d::Zero;
            ray.end = gz::math::Vector3d(0, 0, -100); //for infinite downward
            rayComp->Data().rays.push_back(ray);
        }
    } */

    bool all_wheels_read = true;
    for (int i=0; i < dataPtr-> num_wheels; i++)
    {
      if (dataPtr->wheels[i].linkEntity == gz::sim::kNullEntity)
      {
        all_wheels_read = false;
        gzerr << "Wheel not found: " << dataPtr->wheel_names[i] << std::endl;
      }
    }

    if (all_wheels_read)
    {
      dataPtr->plugin_state_ = TerramechanicsSystemPrivate::PluginState::RUNNING;
      gzmsg << "[TerramechanicsSystem] Plugin ready" << std::endl;
    }
    else
    {
      gzerr << "Not all wheels initialized" << std::endl;
    }

}


void TerramechanicsSystem::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  // Return if simulation is paused
  if (_info.paused)
    return;

  if (dataPtr->plugin_state_ != TerramechanicsSystemPrivate::PluginState::RUNNING)
    return;

  if (dataPtr->options.use_compact_model)
    dataPtr->onUpdateCompact(_ecm);
  else
    dataPtr->onUpdateFull(_ecm);
}

void TerramechanicsSystem::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  // Return if simulation is paused
  if (_info.paused)
    return;

  // NOTE: Contact data are read (and corrected) with one step delay, can't read them otherwise.
  //   This might be moved in preupdate, contact data will be delayed anyway but maybe not 
  //   the steer frame since we also use that one to create the contact frame
  for (int i = 0; i < dataPtr->num_wheels; i++)
  {
    dataPtr->getContactData(i, _ecm);
  }
}

void TerramechanicsSystem::Reset(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  for (int i=0; i < dataPtr->num_wheels; i++)
  {
    dataPtr->wheels[i].forces.W = 0;
    dataPtr->wheels[i].forces.F_x = 0;
    dataPtr->wheels[i].forces.F_y = 0;
    dataPtr->wheels[i].forces.F_z = 0;
    dataPtr->wheels[i].forces.M_x = 0;
    dataPtr->wheels[i].forces.M_y = 0;
    dataPtr->wheels[i].forces.M_z = 0;
    std::fill(dataPtr->wheels[i].forces.R_b.begin(), 
              dataPtr->wheels[i].forces.R_b.end(), 0.0);
    dataPtr->wheels[i].terraParam.h_0 = 0;
  }
  gzmsg << "[TerramechanicsSystem] Plugin reset" << std::endl;
}


void TerramechanicsSystem::TerramechanicsSystemPrivate::initializePluginParam(
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm)
{
  // Get config path
  std::string configPath = _sdf->Get<std::string>("config_path", "config").first;

  try
    {
    // Load options
    YAML::Node optConfig = YAML::LoadFile(configPath + "/plugin_options.yaml");
    options.use_compact_model = optConfig["options"]["use_compact_model"].as<bool>();
    options.passive_plugin = optConfig["options"]["passive_plugin"].as<bool>();
    options.bulldozing_resistance = optConfig["options"]["bulldozing_resistance"].as<std::string>("neglect");
    debug = optConfig["options"]["debug"].as<bool>();
    publish_results = optConfig["options"]["publish_results"].as<bool>();
    publish_intermediate_values = optConfig["options"]["publish_intermediate_values"].as<bool>(false);
    rim_pts = optConfig["options"]["rim_pts"].as<int>(100);
    gzmsg << "[TerramechanicsSystem] Plugin options loaded" << std::endl;

    // Load wheel parameters
    YAML::Node wheelConfig = YAML::LoadFile(configPath + "/wheel_params.yaml");
    globalWheelParams.type = wheelConfig["type"].as<std::string>();
    globalWheelParams.r    = wheelConfig["r"].as<double>(0.0);
    globalWheelParams.h_g  = wheelConfig["h_g"].as<double>(0.0);
    globalWheelParams.b    = wheelConfig["b"].as<double>(0.0);
    globalWheelParams.mu   = wheelConfig["mu"].as<double>(0.0);
    globalWheelParams.r_s  = globalWheelParams.r + globalWheelParams.h_g;

    // Load wheel link and joint names from config
    if (wheelConfig["wheel_links"] && wheelConfig["wheel_links"].size() == num_wheels) {
      for (int i = 0; i < num_wheels; ++i) {
        wheel_names[i] = wheelConfig["wheel_links"][i].as<std::string>();
      }
      gzmsg << "[TerramechanicsSystem] Loaded wheel link names from config" << std::endl;
    } else {
      gzerr << "wheel_links not found or invalid in wheel_params.yaml (need " << num_wheels << " entries)" << std::endl;
    }

    if (wheelConfig["drive_joints"] && wheelConfig["drive_joints"].size() == num_wheels) {
      for (int i = 0; i < num_wheels; ++i) {
        joint_names[i] = wheelConfig["drive_joints"][i].as<std::string>();
      }
      gzmsg << "[TerramechanicsSystem] Loaded drive joint names from config" << std::endl;
    } else {
      gzerr << "drive_joints not found or invalid in wheel_params.yaml (need " << num_wheels << " entries)" << std::endl;
    }

    // Wheels whose drive axis is reversed. Without the list, fall back to the original
    // Archimede naming (right wheels contain "br"/"fr"), which misfires on names like
    // "front_left_wheel".
    if (wheelConfig["reversed_axis"] && wheelConfig["reversed_axis"].size() == num_wheels) {
      for (int i = 0; i < num_wheels; ++i) {
        reversed_axis[i] = wheelConfig["reversed_axis"][i].as<bool>();
      }
    } else {
      for (int i = 0; i < num_wheels; ++i) {
        reversed_axis[i] = wheel_names[i].find("br") != std::string::npos ||
                           wheel_names[i].find("fr") != std::string::npos;
      }
    }

    // Load soil parameters
    YAML::Node soilConfig = YAML::LoadFile(configPath + "/soil_params.yaml");
    for (const auto& soilNode : soilConfig) {
        std::string name = soilNode.first.as<std::string>();
        SoilParams s;
        s.name  = name;
        s.k     = soilNode.second["k"].as<double>(0.0);
        s.k_c   = soilNode.second["k_c"].as<double>(0.0);
        s.k_phi = soilNode.second["k_phi"].as<double>(0.0);
        s.c     = soilNode.second["c"].as<double>(0.0);
        s.phi   = soilNode.second["phi"].as<double>(0.0) * M_PI / 180.0;
        s.K     = soilNode.second["K"].as<double>(0.0);
        s.rho   = soilNode.second["rho"].as<double>(0.0);
        s.n     = soilNode.second["n"].as<double>(0.0);
        s.X_c   = M_PI / 4.0 - s.phi / 2.0;

        // Compute k from Bekker if not directly specified
        if (s.k == 0.0 && s.k_phi != 0.0)
            s.k = s.k_c / globalWheelParams.b + s.k_phi;

        // Load tuned n coefficients (NaN means use soil's fixed n)
        s.n0 = soilNode.second["n0"].as<double>(NAN);
        s.n1 = soilNode.second["n1"].as<double>(NAN);
        s.n2 = soilNode.second["n2"].as<double>(NAN);

        soils[name] = s;
    }
    gzmsg << "[TerramechanicsSystem] Loaded " << soils.size() << " soil types" << std::endl;

    // Load terrain mapping and find terrain collisions
    YAML::Node mapConfig = YAML::LoadFile(configPath + "/terrain_mapping.yaml");
    this->defaultSoil = mapConfig["default_soil"].as<std::string>("Sand_marsSim");
    for (const auto& entry : mapConfig["terrain_mapping"])
    {
      std::string terrainName = entry["pattern"].as<std::string>();

      gz::sim::Entity terrainEntity = _ecm.EntityByName(terrainName).value_or(gz::sim::kNullEntity);
      gz::sim::Model terrainModel(terrainEntity);
      if (!terrainModel.Valid(_ecm))
      {
        gzwarn << "No model found for terrain " << terrainName << std::endl;
        continue;
      }

      for (auto linkEntity : terrainModel.Links(_ecm))
      {
          gz::sim::Link link(linkEntity);
          if (!link.Valid(_ecm)) continue;

          for (auto coll : link.Collisions(_ecm))
          {
              if (coll == gz::sim::kNullEntity) continue;

              collisionsToTerrainMap[coll] = terrainName;

              gzmsg << "[TerramechanicsSystem] Collision entity [" << coll << "] found for terrain model [" << terrainName << "]" << std::endl;
          }
      }

      terrainSoilMap.emplace_back(
          terrainName,
          entry["soil"].as<std::string>()
      );
    }
  }
  catch(const std::exception &e)
  {
    gzerr << "Failed to load config: " << e.what() << std::endl;
  }

  // SDF overrides YAML
  if (_sdf->HasElement("use_compact_model"))
    options.use_compact_model = _sdf->Get<bool>("use_compact_model");
}


void TerramechanicsSystem::TerramechanicsSystemPrivate::initializeWheels(gz::sim::EntityComponentManager &_ecm)
{
  //Initialize all wheels
  bool all_wheels_initialized = true;
  for (int i =0; i <num_wheels; i++)
  {
    wheels[i].name = wheel_names[i];
    wheels[i].initVectors(rim_pts);
    all_wheels_initialized &= initializeWheel(i, _ecm);
  }

  if (!all_wheels_initialized)
    gzerr << "Failed to initiliaze one or more wheels" << std::endl;
}


bool TerramechanicsSystem::TerramechanicsSystemPrivate::initializeWheel(int wheel_idx, gz::sim::EntityComponentManager &_ecm)
{
  WheelData& wheel = wheels[wheel_idx];

  // Get wheel link entity
  wheel.linkEntity = model_.LinkByName(_ecm, wheel.name);
  if (wheel.linkEntity == gz::sim::kNullEntity)
  {
    gzerr << "Link not found: " << wheel.name << std::endl;
    return false;
  }

  // Get joint entity by name
  auto jointEntity = model_.JointByName(_ecm, joint_names[wheel_idx]);
  if (jointEntity == gz::sim::kNullEntity)
  {
    gzerr << "Joint not found: " << joint_names[wheel_idx] << std::endl;
    return false;
  }

  // Get parent link from joint (the steer link)
  gz::sim::Joint joint(jointEntity);
  auto parentLinkName = joint.ParentLinkName(_ecm);
  if (!parentLinkName.has_value())
  {
    gzerr << "Parent link name not found for joint [" << joint_names[wheel_idx] << "]" << std::endl;
    return false;
  }
  wheel.steerEntity = model_.LinkByName(_ecm, parentLinkName.value());
  if (wheel.steerEntity == gz::sim::kNullEntity)
  {
    gzerr << "Steer link [" << parentLinkName.value() << "] not found" << std::endl;
    return false;
  }

  // Discover wheel's contact sensors and attached collisions
  gz::sim::Link wheelLink(wheel.linkEntity);

  bool has_contact_sensor = false;
  for (auto sensorEntity : wheelLink.Sensors(_ecm))
  {
    auto sensorComp = _ecm.Component<gz::sim::components::ContactSensor>(sensorEntity);
    if (!sensorComp) continue;

    // Get sdf <sensor><contact> element of the contact sensor
    auto contactElem = sensorComp->Data()->GetElement("contact");
    if (!contactElem) continue;

    // Iterate through sensor collisions (there might be more than one)
    for (auto collElem = contactElem->GetElement("collision");
        collElem != nullptr;
        collElem = collElem->GetNextElement("collision"))
    {
      std::string collisionName = collElem->Get<std::string>();

      bool collision_found = false;

      for (auto c : wheelLink.Collisions(_ecm))
      {
        auto nameComp = _ecm.Component<gz::sim::components::Name>(c);

        if (nameComp && nameComp->Data() == collisionName)
        {
          wheel.contactSensorsCollisionEntity.push_back(c);
          has_contact_sensor = true;
          collision_found = true;
          gzmsg << "[TerramechanicsSystem] Collision [" << collisionName << "] found for contact sensor [" 
            << _ecm.Component<gz::sim::components::Name>(sensorEntity)->Data() << "] on link [" << wheel.name << "]" << std::endl;
          break;
        }
      }

      if (!collision_found)
        gzwarn << "Collision [" << collisionName << "] not found for contact sensor [" 
          << _ecm.Component<gz::sim::components::Name>(sensorEntity)->Data() << "] on link [" << wheel.name << "]" << std::endl;
    }
  }
  if (!has_contact_sensor)
    gzerr << "No contact sensor found for link [" << wheel.name << "]" << std::endl;

  // Enable velocity checks for wheel and steer links
  wheelLink.EnableVelocityChecks(_ecm, true);

  gz::sim::Link steerLink(wheel.steerEntity);
  steerLink.EnableVelocityChecks(_ecm, true);

  // Set wheel parameters
  setWheelParams(wheel_idx);

  /* // Wheel load computation setup
  // Get gravity
  auto worldEntity = _ecm.EntityByComponents(gz::sim::components::World());
  auto gravityComp = _ecm.Component<gz::sim::components::Gravity>(worldEntity);
  if (gravityComp)
    world_gravity = std::abs(gravityComp->Data().Z());

  // Request WorldPose component for the model (needed for computeWheelLoad)
  if (wheel_idx == 0 && !_ecm.Component<gz::sim::components::WorldPose>(modelEntity_)) {
    _ecm.CreateComponent(modelEntity_, gz::sim::components::WorldPose());
  }

  // Get wheel position
  auto poseComp = _ecm.Component<gz::sim::components::Pose>(wheel.linkEntity);
  gz::math::Vector3d wheel_pos_in_model;
  if (poseComp)
    wheel_pos_in_model = poseComp->Data().Pos();

  // Get wheel + steer mass
  auto wheelInertial = _ecm.Component<gz::sim::components::Inertial>(wheel.linkEntity);
  auto steerInertial = _ecm.Component<gz::sim::components::Inertial>(wheel.steerEntity);
  if (wheelInertial && steerInertial)
    wheel.link_mass = wheelInertial->Data().MassMatrix().Mass() +
                      steerInertial->Data().MassMatrix().Mass();

  // Set rover dimensions once
  if (wheel_idx == 0)
  {
    rover_dimensions[0] = 2 * fabs(wheel_pos_in_model.X());
    rover_dimensions[1] = 2 * fabs(wheel_pos_in_model.Y());

    auto model_links = model_.Links(_ecm);
    other_masses.clear();

    for (const auto& linkEntity : model_links)
    {
      auto nameComp = _ecm.Component<gz::sim::components::Name>(linkEntity);
      if (!nameComp) continue;

      std::string linkName = nameComp->Data();
      if (linkName.find("wheel") != std::string::npos ||
          linkName.find("steer") != std::string::npos)
        continue;

      auto inertialComp = _ecm.Component<gz::sim::components::Inertial>(linkEntity);
      if (!inertialComp) continue;

      double mass = inertialComp->Data().MassMatrix().Mass();
      if (mass == 0) continue;

      auto linkPoseComp = _ecm.Component<gz::sim::components::Pose>(linkEntity);
      if (!linkPoseComp) continue;

      gz::math::Vector3d wheel_to_link = linkPoseComp->Data().Pos() - wheel_pos_in_model;
      other_masses.push_back({mass, fabs(wheel_to_link.X()), fabs(wheel_to_link.Y()), fabs(wheel_to_link.Z())});
    }
  }

  // Set force signs based on wheel position
  if (wheel.name.find("br") != std::string::npos) {
    wheel.f_signs[0] = -1; wheel.f_signs[1] = -1;
  } else if (wheel.name.find("fr") != std::string::npos) {
    wheel.f_signs[0] = 1; wheel.f_signs[1] = -1;
  } else if (wheel.name.find("bl") != std::string::npos) {
    wheel.f_signs[0] = -1; wheel.f_signs[1] = 1;
  } else if (wheel.name.find("fl") != std::string::npos) {
    wheel.f_signs[0] = 1; wheel.f_signs[1] = 1;
  } else {
    gzerr << "unrecognized wheel name: " << wheel.name << std::endl;
  } */

  return true;
}


void TerramechanicsSystem::TerramechanicsSystemPrivate::initializeTransport()
{
  for (int i = 0; i < num_wheels; i++)
  {
    std::string prefix = "/terra/" + wheel_names[i];
    // wheels[i].forcesPub = node.Advertise<gz::msgs::Double_V>(prefix + "/forces");
    // wheels[i].statePub = node.Advertise<gz::msgs::Double_V>(prefix + "/state");
    wheels[i].forcesPub = node.Advertise<gz::msgs::Float_V>(prefix + "/forces");
    wheels[i].statePub = node.Advertise<gz::msgs::Float_V>(prefix + "/state");
  }
  gzmsg << "[TerramechanicsSystem] Transport initialized" << std::endl;
}


bool TerramechanicsSystem::TerramechanicsSystemPrivate::findSinkage(int wheel_idx)
{
    WheelData& wheel = wheels[wheel_idx];

    // find sinkage iteratively equalizing computed F_z with wheel load
    double h_min = 0;
    double h_max = 1.5 * wheel.wheelParam->r_s;
    double max_err = wheel.forces.W * pow(10, -3);
    double F_z = 0;

    // binary search algorithm
    while (fabs(F_z - wheel.forces.W) > fabs(max_err)) {
        wheel.terraParam.h_0 = (h_max + h_min) / 2;

        computeContactGeometry(wheel_idx);
        computeStresses(wheel_idx);
        computeForces(wheel_idx, &F_z);

        if (F_z < wheel.forces.W) {
            h_min = wheel.terraParam.h_0;
        } else {
            h_max = wheel.terraParam.h_0;
        }

        if ((h_max - h_min) < pow(10, -6)) {
            /* gzwarn << "Sinkage not found for wheel [" << wheel.name << 
                            "]: expected Fz = " << wheel.forces.W << 
                            ", computed Fz = " << F_z << 
                            ", h_min_max = [" << h_min << " - " << h_max << "]" << std::endl; */
            return false;
        }
    }

    if (this->debug) {
        gzmsg << "[TerramechanicsSystem] DB [" << wheel.name << "]: Sinkage found: " << 
                        wheel.terraParam.h_0 << std::endl;
    }

    return true;
}


void TerramechanicsSystem::TerramechanicsSystemPrivate::computeContactGeometry(int wheel_idx)
{
  WheelData& wheel = wheels[wheel_idx];

  // unpack params
  double h_0 = wheel.terraParam.h_0;
  double r_s = wheel.wheelParam->r_s;
  // int rim_pts = rim_pts;
  double theta_f = acos(1 - h_0/r_s);

  /* // complex geometry
  // for s>1, theta_m>theta_f breaks things
  if (s >= 0)
  {
    theta_m = theta_f * (a0 + a1*s);
    theta_r = theta_f * (b0 + b1*s);
  } else{
    theta_m = theta_f * a0;
    theta_r = theta_f * b0;
  } */

  // simplified geometry
  double theta_r = 0;
  double theta_m = (theta_f + theta_r) / 2;

  double d_theta = (theta_f - theta_r) / (rim_pts - 1);
  for (int i = 0; i < rim_pts; i++) {
      wheel.terraParam.theta[i] = theta_r + i * d_theta;
      wheel.terraParam.h[i] = r_s * (cos(wheel.terraParam.theta[i]) - cos(theta_f));
  }

  // update params
  wheel.terraParam.theta_f = theta_f;
  wheel.terraParam.theta_r = theta_r;
  wheel.terraParam.theta_m = theta_m;
}


void TerramechanicsSystem::TerramechanicsSystemPrivate::computeStresses(int wheel_idx)
{
  WheelData& wheel = wheels[wheel_idx];
  // unpack params
  // int rim_pts = this->rim_pts;
  double k = wheel.soilParam->k;
  double c = wheel.soilParam->c;
  double phi = wheel.soilParam->phi;
  double K = wheel.soilParam->K;
  double n = wheel.tunedParam.n_effective;
  double r = wheel.wheelParam->r;
  double r_s = wheel.wheelParam->r_s;
  double mu = wheel.wheelParam->mu;
  double d0 = wheel.tunedParam.d0;
  double d1 = wheel.tunedParam.d1;
  double v_y = wheel.stateParam.v_y;
  double omega = wheel.stateParam.omega;
  double s = wheel.stateParam.s;
  double theta_f = wheel.terraParam.theta_f;
  double theta_r = wheel.terraParam.theta_r;
  double theta_m = wheel.terraParam.theta_m;

  double theta_0 = theta_f * (d0 + d1 * s);
  wheel.terraParam.theta_0 = theta_0;

  std::ostringstream out_info; out_info << "DB [" << wheel.name << "]:";
  for (size_t i = 0; i < rim_pts; i++)
  {
    double theta_i = wheel.terraParam.theta[i];
    // normal stress
    if (theta_i >= theta_m)
    {wheel.terraParam.theta_e[i] = theta_i;}
    else {
      wheel.terraParam.theta_e[i] = theta_f - 
          (theta_i - theta_r) * (theta_f - theta_m) / (theta_m - theta_r);
    }

    double sigma_r, sigma_rs;
    if (fabs(theta_i - theta_f) < pow(10,-6))
    {
      // if (cos(theta_e[i])-cos(theta_f)) is slightly negative (when we should be in theta_f), pow gives nan values
      sigma_r = 0;
      sigma_rs = 0;
    } else {
        sigma_r = k * pow(r, n) * 
            pow((cos(wheel.terraParam.theta_e[i]) - cos(theta_f)), n);
        sigma_rs = k * pow(r_s, n) * 
            pow((cos(wheel.terraParam.theta_e[i]) - cos(theta_f)), n);
      }

    // if(this->debug){out_info << "\n\ttheta_i = "<< theta[i]<<"\n\tsigma_rs: " << sigma_rs << " = " << k * pow(r_s,n) << " * pow(" << (cos(theta_e[i])-cos(theta_f))<<","<<n<<")";}

    wheel.terraParam.sigma[i] = mu * sigma_rs + (1 - mu) * sigma_r;

    // shear stress
    wheel.terraParam.v_jl[i] = v_y;
    // a stopped wheel would give 0/0 here; a tiny omega still saturates the lateral shear
    double omega_safe = std::max(fabs(omega), 1e-3);
    wheel.terraParam.j_l[i] = (theta_f - theta_i) * v_y / omega_safe;
    // if(this->debug){out_info <<"\n\tv_jl = " << v_jl[i] << ", j_l = " << j_l[i];}

    if (s >= 0) {
        wheel.terraParam.v_jt[i] = omega * r_s * (1 - (1 - s) * cos(theta_i));
        wheel.terraParam.j_t[i] = r_s * ((theta_f - theta_i) - 
            (1 - s) * (sin(theta_f) - sin(theta_i)));
    } else if (theta_i >= theta_0) {
        wheel.terraParam.v_jt[i] = omega * r_s / (1 + s) * 
            ((sin(theta_f) - sin(theta_0)) / (theta_f - theta_0) - cos(theta_i));
        wheel.terraParam.j_t[i] = r_s / (1 + s) * 
            ((sin(theta_f) - sin(theta_0)) * (theta_f - theta_i) / (theta_f - theta_0) - 
              (sin(theta_f) - sin(theta_i)));
    } else {
        wheel.terraParam.v_jt[i] = omega * r_s * (1 - cos(theta_i) / (1 + s));
        wheel.terraParam.j_t[i] = r_s * ((theta_0 - theta_i) - 
            (sin(theta_0) - sin(theta_i)) / (1 + s));
    }

    wheel.terraParam.j[i] = sqrt(pow(wheel.terraParam.j_t[i], 2) + 
                                          pow(wheel.terraParam.j_l[i], 2));

    wheel.terraParam.tau[i] = (c + wheel.terraParam.sigma[i] * tan(phi)) * 
                                        (1 - exp(-wheel.terraParam.j[i] / K));
        double denominator = sqrt(pow(wheel.terraParam.v_jt[i], 2) + 
                                  pow(wheel.terraParam.v_jl[i], 2));

    if (denominator < 1e-9) {
      // no shear velocity (stopped wheel), so no direction to apply the shear stress in
      wheel.terraParam.tau_t[i] = 0;
      wheel.terraParam.tau_l[i] = 0;
    } else {
      wheel.terraParam.tau_t[i] = wheel.terraParam.tau[i] * 
                                            wheel.terraParam.v_jt[i] / denominator;

      wheel.terraParam.tau_l[i] = wheel.terraParam.tau[i] * 
                                            wheel.terraParam.v_jl[i] / denominator;
    }

    // if(this->debug){out_info << "\n\tsigma = " << sigma[i] << ", tau = " << tau[i];}
  }
  // // if(this->debug){ROS_INFO_STREAM(out_info.str());}

  // // update params
  // std::memcpy(this -> terraParam.theta_e, theta_e, rim_pts * sizeof(double));
  // std::memcpy(this -> terraParam.sigma, sigma, rim_pts * sizeof(double));
  // std::memcpy(this -> terraParam.tau, tau, rim_pts * sizeof(double));
  // std::memcpy(this -> terraParam.tau_t, tau_t, rim_pts * sizeof(double));
  // std::memcpy(this -> terraParam.tau_l, tau_l, rim_pts * sizeof(double));
  // std::memcpy(this -> terraParam.j, j, rim_pts * sizeof(double));
  // std::memcpy(this -> terraParam.j_t, j_t, rim_pts * sizeof(double));
  // std::memcpy(this -> terraParam.j_l, j_l, rim_pts * sizeof(double));
  // std::memcpy(this -> terraParam.v_jt, v_jt, rim_pts * sizeof(double));
  // std::memcpy(this -> terraParam.v_jl, v_jl, rim_pts * sizeof(double));
  // this -> terraParam.theta_0 = theta_0;
}


void TerramechanicsSystem::TerramechanicsSystemPrivate::computeForces(int wheel_idx, double* F_z_out)
{
  // if F_z_out is passed as an argument this only computes F_z and assignes it to F_z_out,
  // otherwise it computes all forces/moments and updates the forces params
  WheelData& wheel = wheels[wheel_idx];
  // unpack params
  int rim_pts = this -> rim_pts;
  std::string bulldozing_resistance = this -> options.bulldozing_resistance;
  double X_c = wheel.soilParam->X_c;
  double c = wheel.soilParam->c;
  double rho = wheel.soilParam->rho;
  double phi = wheel.soilParam->phi;
  double theta_f = wheel.terraParam.theta_f;
  double theta_r = wheel.terraParam.theta_r;
  double theta_m = wheel.terraParam.theta_m;
  double beta = wheel.stateParam.beta;
  double r_s = wheel.wheelParam->r_s;
  double b = wheel.wheelParam->b;


  double d_theta = (theta_f-theta_r) / (rim_pts - 1);
  double F_z = 0;

  for (size_t i = 1; i < rim_pts; i++) {
      F_z += r_s * b * d_theta * (wheel.terraParam.tau_t[i] * 
                                sin(wheel.terraParam.theta[i]) + 
                                wheel.terraParam.sigma[i] * 
                                cos(wheel.terraParam.theta[i]));
  }
  if (F_z_out)
  {
    *F_z_out = F_z;
    return;
  }

  double F_x = 0, F_y = 0, M_x = 0, M_y = 0, M_z = 0;

  if (bulldozing_resistance == "Ishigami"){
    double D1 = 1/tan(X_c) + tan(X_c + phi);
    double D2 = 1/tan(X_c) + pow((1/tan(X_c)), 2) / (1/tan(phi));

    for (size_t i = 1; i < rim_pts; i++) {
        double hh = r_s * (cos(wheel.terraParam.theta[i]) - cos(theta_f));
        wheel.forces.R_b[i] = D1 * (c * hh + 0.5 * D2 * rho * pow(hh, 2));

        // negative sign so that F_y has opposite direction to wheel lateral velocity
        F_y -= d_theta * (r_s * b * wheel.terraParam.tau_l[i] + 
                        wheel.forces.R_b[i] * sin(beta) * 
                        (r_s - hh * cos(wheel.terraParam.theta[i])));
        
        F_x += r_s * b * d_theta * (wheel.terraParam.tau_t[i] * 
                                  cos(wheel.terraParam.theta[i]) - 
                                  wheel.terraParam.sigma[i] * 
                                  sin(wheel.terraParam.theta[i]));
                    M_y -= pow(r_s, 2) * b * d_theta * wheel.terraParam.tau_t[i];
    }   

  } else if (bulldozing_resistance == "Pavlov"){
      for (size_t i = 1; i < rim_pts; i++) {
          double h_b = r_s * ((sin(wheel.terraParam.theta[i]) - sin(theta_r)) * 
                            (cos(theta_r) - cos(theta_f)) / (sin(theta_f) - sin(theta_r)) - 
                            (cos(theta_r) - cos(wheel.terraParam.theta[i])));

          wheel.forces.R_b[i] = rho/2 * pow(h_b, 2) * pow((1/tan(X_c)), 2) * 
                              (1 + 0.5 / tan(X_c) * tan(phi)) + 
                              2 * h_b * c / tan(X_c);

      // negative sign so that F_y has opposite direction to wheel lateral velocity
      F_y -= r_s * d_theta * (b * wheel.terraParam.tau_l[i] + 
                              wheel.forces.R_b[i] * sin(beta) * 
                              cos(wheel.terraParam.theta[i]));

      F_x += r_s * b * d_theta * (wheel.terraParam.tau_t[i] * 
                                cos(wheel.terraParam.theta[i]) - 
                                wheel.terraParam.sigma[i] * 
                                sin(wheel.terraParam.theta[i]));

      M_y -= pow(r_s, 2) * b * d_theta * wheel.terraParam.tau_t[i];
    }
  } else{
    if (bulldozing_resistance != "neglect"){
      gzmsg << "[TerramechanicsSystem] bulldozing_resistance option not recognised. Neglecting sidewall bulldozing force..." << std::endl;
    }
    for (size_t i = 1; i < rim_pts; i++)
    {
      // negative sign so that F_y has opposite direction to wheel lateral velocity
      F_y -= d_theta * (r_s * b * wheel.terraParam.tau_l[i]);

      F_x += r_s * b * d_theta * (wheel.terraParam.tau_t[i] * 
                                cos(wheel.terraParam.theta[i]) - 
                                wheel.terraParam.sigma[i] * 
                                sin(wheel.terraParam.theta[i]));

      M_y -= pow(r_s,2) * b * d_theta * wheel.terraParam.tau_t[i];
    }
  }

  M_x += F_y * r_s;
  M_z += F_y * r_s * sin(theta_m);

  // update params
  wheel.forces.F_x = F_x;
  wheel.forces.F_y = F_y;
  wheel.forces.F_z = F_z;
  wheel.forces.M_x = M_x;
  wheel.forces.M_y = M_y;
  wheel.forces.M_z = M_z;
}


/* void TerramechanicsSystem::TerramechanicsSystemPrivate::computeWheelLoad(gz::sim::EntityComponentManager &_ecm, int wheel_idx)
{
    WheelData& wheel = wheels[wheel_idx];
  // compute static load on the wheel and gravity component tangential to the ground according
  // to the rover orientation wrt to gravity direction
  // this does not account for dynamics effects (curves and acceleration/deceleration),
  // and always considers all 4 wheels in contact with the terrain

  auto poseComp = _ecm.Component<gz::sim::components::WorldPose>(modelEntity_);
  if (!poseComp) {
    gzerr << "WorldPose component not available for model" << std::endl;
    return;
  }
  gz::math::Quaterniond rover_orient = poseComp->Data().Rot();

  // rover (and assumed terrain) inclinations around lateral & longitudinal axis
  double theta = -rover_orient.Pitch(); // positive -> forward is uphill
  double alpha = rover_orient.Roll(); // positive -> left is uphill

  double wheel_load = wheel.link_mass;

  for (std::vector<double> link_i : this->other_masses)
  {
    wheel_load += link_i[0] * (1 - (link_i[1] + wheel.f_signs[0]*link_i[3]*tan(theta))/this->rover_dimensions[0]) * (1 - (link_i[2] + wheel.f_signs[1]*link_i[3]*tan(alpha))/this->rover_dimensions[1]);
  }

  // wheel load along gravity direction (world negative z-axis)
  wheel_load *= this -> world_gravity;

  // load components in contact frame
  gz::math::Vector3d wheel_load_contact = wheel.contact_frame_rot.RotateVectorReverse(gz::math::Vector3d(0,0,-wheel_load));

  if (debug)
  {
    gzmsg << "[TerramechanicsSystem] DB [" << wheel.name << "]: Load on wheel xyz:"
          << wheel_load_contact.X() << ", "
          << wheel_load_contact.Y() << ", "
          << wheel_load_contact.Z() << std::endl;
  }
  wheel.forces.W = fabs(wheel_load_contact.Z());
} */


void TerramechanicsSystem::TerramechanicsSystemPrivate::setWheelParams(int wheel_idx)
{
  WheelData& wheel = wheels[wheel_idx];

  // Copy from global config
  wheel.wheelParam = &globalWheelParams;
}


void TerramechanicsSystem::TerramechanicsSystemPrivate::setWheelStateParams(int wheel_idx, gz::sim::EntityComponentManager &_ecm)
{
  WheelData& wheel = wheels[wheel_idx];

  // Wheel angular drive velocity
  // auto angularVelComp = _ecm.Component<gz::sim::components::AngularVelocity>(wheel.linkEntity);
  auto angularVelComp = _ecm.Component<gz::sim::components::AngularVelocity>(wheel.linkEntity);
  if(angularVelComp) {
      wheel.stateParam.omega = angularVelComp->Data().Y();
  }

  // Change sign where necessary so that omega > 0 for forward movement
  if (reversed_axis[wheel_idx])
    wheel.stateParam.omega *= -1;

  // If omega < 0 (backward movement), change its sign and rotate contact frame so it's the same as moving forward in the reverse direction
  if (wheel.stateParam.omega < 0) {
      wheel.stateParam.omega *= -1;
      wheel.contact_frame_rot = wheel.contact_frame_rot * gz::math::Quaterniond(0, 0, M_PI);
      wheel.contact_frame_rot.Normalize();
  }


  // Wheel velocity in contact frame
  auto velComp = _ecm.Component<gz::sim::components::WorldLinearVelocity>(wheel.steerEntity);
  gz::math::Vector3d link_relative_vel = 
      wheel.contact_frame_rot.RotateVectorReverse(velComp->Data());


  wheel.stateParam.v_x = link_relative_vel.X();
  wheel.stateParam.v_y = link_relative_vel.Y();

  wheel.stateParam.v = sqrt(pow(wheel.stateParam.v_x, 2) + 
                                    pow(wheel.stateParam.v_y, 2));

  wheel.stateParam.beta = (wheel.stateParam.v_x == 0 && wheel.stateParam.v_y == 0)
      ? 0.0 : atan(wheel.stateParam.v_y / wheel.stateParam.v_x);

  if (fabs(wheel.stateParam.omega * wheel.wheelParam->r_s) <= pow(10, -4) && 
      fabs(wheel.stateParam.v_x) <= pow(10, -4)) {
      // Case 0/0
      wheel.stateParam.s = 0;
  } else if (fabs(wheel.wheelParam->r_s * wheel.stateParam.omega) >= 
              fabs(wheel.stateParam.v_x)) {
      // Slip
      wheel.stateParam.s = 
          (wheel.wheelParam->r_s * wheel.stateParam.omega - wheel.stateParam.v_x) / 
          (wheel.wheelParam->r_s * wheel.stateParam.omega);
  } else {
      // Skid
      wheel.stateParam.s = 
          (wheel.wheelParam->r_s * wheel.stateParam.omega - wheel.stateParam.v_x) / 
          (wheel.stateParam.v_x);
  }     

};


void TerramechanicsSystem::TerramechanicsSystemPrivate::setSoilParams(int wheel_idx,  gz::sim::EntityComponentManager &_ecm)
{
  // Get terrain under the wheels
  std::string terrainName = getTerrainBelow(wheel_idx, _ecm); // TODO update
  // gzmsg << "[TerramechanicsSystem] " << terrainName << std::endl;

  // Lookup mapping
  std::string soilName = defaultSoil;
  for (const auto& [pattern, soil] : terrainSoilMap)
  {
    if (terrainName.find(pattern) != std::string::npos)
    {
      soilName = soil;
      break;
    }
  }

  // Get params
  wheels[wheel_idx].soilParam = &soils[soilName];
}


std::string TerramechanicsSystem::TerramechanicsSystemPrivate::getTerrainBelow(
    int wheel_idx, gz::sim::EntityComponentManager &_ecm)
{
    /* auto rayComp = _ecm.Component<gz::sim::components::RaycastData>(wheels[wheel_idx].rayEntity);
    if (rayComp && !rayComp->Data().results.empty() && 
        rayComp->Data().results[0].fraction > 0)
        return "terrain";
    return ""; */

    return wheels[wheel_idx].terrain_name;
}


void TerramechanicsSystem::TerramechanicsSystemPrivate::setTunedParams(int wheel_idx)
{
    WheelData& wheel = wheels[wheel_idx];

    // Initialize tunedParam n-coefficients from soil YAML values
    wheel.tunedParam.n0 = wheel.soilParam->n0;
    wheel.tunedParam.n1 = wheel.soilParam->n1;
    wheel.tunedParam.n2 = wheel.soilParam->n2;
    
    double beta = fabs(wheel.stateParam.beta);


    if (wheel.wheelParam->type == "smooth" && wheel.soilParam->name == "Soil_Direct_90_sand") {
    // coeff interpolated from Pavlov data (wheels won't be the same), not used in simplified geometry
        wheel.tunedParam.a0 = -0.0539*pow(beta, 3) - 0.0227*pow(beta, 2) + 0.6294*beta + 0.1674;
        wheel.tunedParam.a1 = 0.0156*pow(beta, 3) + 0.3414*pow(beta, 2) - 1.0039*beta + 0.7632;
        wheel.tunedParam.b0 = -0.4386*pow(beta, 4) + 0.8719*pow(beta, 3) - 0.5565*pow(beta, 2) + 0.0905*beta - 0.4753;
        wheel.tunedParam.b1 = 0.4178*pow(beta, 3) - 0.8113*pow(beta, 2) + 0.2433*beta - 0.0044;
        
        wheel.tunedParam.n0 = 1.46;
        wheel.tunedParam.n1 = 0.01;
        wheel.tunedParam.n2 = 0.55;
    } else if (wheel.wheelParam->type == "grousers" && wheel.soilParam->name == "Soil_Direct_90_sand") {
        wheel.tunedParam.a0 = 0.7450*pow(beta, 4) - 2.4800*pow(beta, 3) + 2.6033*pow(beta, 2) - 0.4332*beta + 0.2716;
        wheel.tunedParam.a1 = -0.6876*pow(beta, 4) + 2.3687*pow(beta, 3) - 2.5096*pow(beta, 2) + 0.3618*beta + 0.6761;
        wheel.tunedParam.b0 = -0.3198*pow(beta, 3) + 0.1680*pow(beta, 2) + 0.3219*beta - 0.6372;
        wheel.tunedParam.b1 = 0.1387*pow(beta, 3) + 0.3971*pow(beta, 2) - 0.8644*beta - 0.1921;
        
        wheel.tunedParam.n0 = 1.46;
        wheel.tunedParam.n1 = 0.01;
        wheel.tunedParam.n2 = 0.74;
    }

    wheel.tunedParam.d0 = 1;
    wheel.tunedParam.d1 = 0.5;

    // case n is already set for the soil
    if (std::isnan(wheel.soilParam->n0)) {
        wheel.tunedParam.n_effective = wheel.soilParam->n;
        return;
    }
    if (wheel.stateParam.s >= 0) {
        wheel.tunedParam.n_effective = wheel.tunedParam.n0 + wheel.tunedParam.n1 * wheel.stateParam.s;
    } else {
        wheel.tunedParam.n_effective = wheel.tunedParam.n0 - wheel.tunedParam.n2 * wheel.stateParam.s;
    }
}


void TerramechanicsSystem::TerramechanicsSystemPrivate::applyForce(int wheel_idx, gz::sim::EntityComponentManager &_ecm)
{
  WheelData& wheel = wheels[wheel_idx];

  // case forces are in contact frame
  // contact frame: force -> z=0(keep gazebo's), xy = model's
  gz::math::Vector3d force_to_add_contact(wheel.forces.F_x, wheel.forces.F_y, 0);
  // M_y = M_z = 0, they are drive and steer joints resistent moments, but do not influence the rover movement
  // gz::math::Vector3d torque_to_add_contact(wheel.forces.M_x, 0, 0);
  gz::math::Vector3d torque_to_add_contact(wheel.forces.M_x, wheel.forces.M_y, wheel.forces.M_z);

  // Transform to world frame
  gz::math::Vector3d force_to_add = wheel.contact_frame_rot.RotateVector(force_to_add_contact);
  gz::math::Vector3d torque_to_add = wheel.contact_frame_rot.RotateVector(torque_to_add_contact);

  // a NaN wrench would wreck the whole physics step, so drop it instead
  if (!std::isfinite(force_to_add.X()) || !std::isfinite(force_to_add.Y()) ||
      !std::isfinite(force_to_add.Z()) || !std::isfinite(torque_to_add.X()) ||
      !std::isfinite(torque_to_add.Y()) || !std::isfinite(torque_to_add.Z()))
  {
    gzwarn << "[TerramechanicsSystem] non-finite wrench on [" << wheel.name
           << "], skipped this step" << std::endl;
    return;
  }

  // apply force/torque in world frame to the wheel
  if (!options.passive_plugin)
  {
    gz::sim::Link link(wheel.linkEntity);
    link.AddWorldWrench(_ecm, force_to_add, torque_to_add);
  }

  if (this->debug) {
      gzmsg << "[TerramechanicsSystem] DB [" << wheel.name << "]: applied terramechanic forces: " << 
                      force_to_add.X() << " - " << force_to_add.Y() << " - " << 
                      force_to_add.Z() << "\n\t\t\t\t\t\t\t\t\t\ttorques: " << 
                      torque_to_add.X() << " - " << torque_to_add.Y() << " - " << 
                      torque_to_add.Z() << std::endl;
  }
}


void TerramechanicsSystem::TerramechanicsSystemPrivate::publishResults(int wheel_idx)
{
  WheelData& wheel = wheels[wheel_idx];

  gz::math::Vector3d force_contact(wheel.forces.F_x, wheel.forces.F_y, wheel.forces.F_z);
  gz::math::Vector3d torque_contact(wheel.forces.M_x, wheel.forces.M_y, wheel.forces.M_z);
  gz::math::Vector3d force_world = wheel.contact_frame_rot.RotateVector(force_contact);
  gz::math::Vector3d torque_world = wheel.contact_frame_rot.RotateVector(torque_contact);

  // gz::msgs::Double_V forceMsg;
  gz::msgs::Float_V forceMsg; // already supported in ros_gz_bridge
  forceMsg.add_data(force_contact.X());
  forceMsg.add_data(force_contact.Y());
  forceMsg.add_data(force_contact.Z());
  forceMsg.add_data(torque_contact.X());
  forceMsg.add_data(torque_contact.Y());
  forceMsg.add_data(torque_contact.Z());
  forceMsg.add_data(force_world.X());
  forceMsg.add_data(force_world.Y());
  forceMsg.add_data(force_world.Z());
  forceMsg.add_data(torque_world.X());
  forceMsg.add_data(torque_world.Y());
  forceMsg.add_data(torque_world.Z());

  wheel.forcesPub.Publish(forceMsg);

  if (publish_intermediate_values)
  {
    // gz::msgs::Double_V stateMsg;
    gz::msgs::Float_V stateMsg; // already supported in ros_gz_bridge
    stateMsg.add_data(wheel.stateParam.omega);
    stateMsg.add_data(wheel.stateParam.v_x);
    stateMsg.add_data(wheel.stateParam.v_y);
    stateMsg.add_data(wheel.stateParam.s);
    stateMsg.add_data(wheel.stateParam.beta * 180.0 / M_PI);
    stateMsg.add_data(wheel.forces.W);
    stateMsg.add_data(wheel.terraParam.h_0);

    wheel.statePub.Publish(stateMsg);
  }
}


// Main update method with three-phase parallelism
void TerramechanicsSystem::TerramechanicsSystemPrivate::onUpdateFull(gz::sim::EntityComponentManager &_ecm)
{
    // Skip if not running
    if (plugin_state_ != PluginState::RUNNING) {
        return;
    }

    // // Rate limiter for performance
    // update_step_counter++;
    // if (options.skip_update_steps > 0) {
    //     if (update_step_counter % (options.skip_update_steps + 1) != 0) {
    //         // On skipped steps, just apply previous forces to maintain behavior
    //         for (int i = 0; i < num_wheels; i++) {
    //             if (fabs(wheels[i].stateParam.omega * wheels[i].wheelParam->r_s) >= 0.02) {
    //                 applyForce(i);
    //             }
    //         }
    //         return;
    //     }
    // }

    // 1. First collect all wheel states (serial)
    for (int i = 0; i < num_wheels; i++)
    {
      /* auto steerPose = _ecm.Component<gz::sim::components::WorldPose>(wheels[i].steerEntity);
      auto wheelPose = _ecm.Component<gz::sim::components::WorldPose>(wheels[i].linkEntity);
      if (wheelPose)
          _ecm.SetComponentData<gz::sim::components::Pose>(
              wheels[i].rayEntity, gz::math::Pose3d(wheelPose->Data().Pos(), gz::math::Quaterniond::Identity));

      if (steerPose)
        wheels[i].contact_frame_rot = steerPose->Data().Rot();

      if (wheels[i].name.find("br") != std::string::npos ||
          wheels[i].name.find("fr") != std::string::npos)
      {
        wheels[i].contact_frame_rot =
            wheels[i].contact_frame_rot * gz::math::Quaterniond(0, 0, M_PI);
      }
      wheels[i].contact_frame_rot.Normalize(); */

      // getContactData(i, _ecm);

      if (! wheels[i].in_contact)
        continue;

      setSoilParams(i, _ecm);
      setWheelStateParams(i, _ecm);

      // if (this->debug) {
      //   gzmsg << "[TerramechanicsSystem] " << wheels[i].name << ": soil=" << wheels[i].soilParam.name
      //   << " omega=" << wheels[i].stateParam.omega << std::endl;
      // }
    }

    // 2. Perform computations (parallel - safe)
    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < num_wheels; i++)
    {
      if (! wheels[i].in_contact)
        continue;

      // Skip static wheels
      /* if (fabs(wheels[i].stateParam.omega * wheels[i].wheelParam->r_s) < 0.02)
        continue; */

      // Computation-heavy tasks
      setTunedParams(i);
      // computeWheelLoad(_ecm, i); // Taken from contact sensors

      // Find sinkage and compute forces (computationally intensive)
      if (findSinkage(i))
          computeForces(i, nullptr);
      else
      {
        // keep friction? reset contact_counter?
        // wheels[i].in_contact = false;
        wheels[i].forces.F_x = 0;
        wheels[i].forces.F_y = 0;
        wheels[i].forces.F_z = 0;
        wheels[i].forces.M_x = 0;
        wheels[i].forces.M_y = 0;
        wheels[i].forces.M_z = 0;
      }
    }

    // 3. Apply forces (serial - safe)
    for (int i = 0; i < num_wheels; i++)
    {
      if (! wheels[i].in_contact)
        continue;

      /* if (fabs(wheels[i].stateParam.omega * wheels[i].wheelParam->r_s) < 0.02) {
          // Only zero velocities when wheel is in contact with terrain,
          // otherwise gravity free-fall is blocked
          if (!this->options.passive_plugin)  // NOT SURE IF THIS IS NECESSARY ANYMORE
          {
              gz::sim::Link link(wheels[i].linkEntity);
              link.SetLinearVelocity(_ecm, gz::math::Vector3d::Zero);
              link.SetAngularVelocity(_ecm, gz::math::Vector3d::Zero);
          }
          continue;
      } */

      // Interact with physics engine (not thread-safe)
      applyForce(i, _ecm);

      // Publish results
      if (this->publish_results) {
          publishResults(i);
      }
    }
}


void TerramechanicsSystem::TerramechanicsSystemPrivate::getContactData(int wheel_idx, const gz::sim::EntityComponentManager &_ecm)
{
  WheelData& wheel = wheels[wheel_idx];
  wheel.in_contact = false;
  gz::math::Vector3d normal, force;

  for (const auto &collisionEntity : wheel.contactSensorsCollisionEntity)
  {
    const auto *comp = _ecm.Component<gz::sim::components::ContactSensorData>(collisionEntity);
    if (comp)
    {
      // Loop contact pairs
      for (int c = 0; c < comp->Data().contact_size(); c++)
      {
        const auto &contact = comp->Data().contact(c);

        bool is_body_1;
        gz::sim::Entity coll1 = contact.collision1().id();
        gz::sim::Entity coll2 = contact.collision2().id();

        if (coll1 != collisionEntity && coll2 != collisionEntity)
        {
          gzwarn << "Wheel collision not detected by its own sensor? Something is wrong..." << std::endl;
          continue;
        }

        if (auto it = collisionsToTerrainMap.find(coll2); it != collisionsToTerrainMap.end())
        {
          // Collision1 is the wheel and collision2 is a terrain
          is_body_1 = true;
          wheel.terrain_name = it->second;
        } else if (auto it = collisionsToTerrainMap.find(coll1); it != collisionsToTerrainMap.end())
        {
          // Collision2 is the wheel and collision1 is a terrain
          is_body_1 = false;
          wheel.terrain_name = it->second;
        } else {
          // Collision with some other non-saved-terrain object. Neglected for this plugin
          // Maybe normal force should be included anyway within wheel load(?) maybe not. Friction we keep
          continue;
        }

        wheel.in_contact = true;

        // Loop contact points
        for (int ii = 0; ii < contact.position_size(); ii++)
        {
          if (is_body_1)
          {
            // normal points from collision2 to collision1, change sign to have it ground to wheel
            normal += gz::msgs::Convert(contact.normal(ii));
            force += gz::msgs::Convert(contact.wrench(ii).body_1_wrench().force());
          }
          else
          {
            normal -= gz::msgs::Convert(contact.normal(ii));
            force += gz::msgs::Convert(contact.wrench(ii).body_2_wrench().force());
          }
        }
      }
    }
  }

  if (wheel.in_contact)
  {
    // NOT SURE ABOUT THIS 'aggregation' LOGIC IF WHEEL HAS MULTIPLE CONTACT SENSORS on different collisions,
    // they might 'jump' one at the time due to sim artifact and we would keep only part of the load,
    // probably better to treat them individually in that case and aggregate later

    wheel.iters_without_contact = 0;

    // Find normal and tangential force components
    normal.Normalize();

    // Set wheel load
    wheel.forces.W = fabs(force.Dot(normal));

    // Contact frame
    auto steerPose = _ecm.Component<gz::sim::components::WorldPose>(wheel.steerEntity);
    if (steerPose)
    {
      gz::math::Vector3d steer_x = steerPose->Data().Rot().XAxis();

      gz::math::Vector3d contact_x = (steer_x - steer_x.Dot(normal) * normal).Normalize();
      gz::math::Vector3d contact_y = normal.Cross(contact_x).Normalize();

      gz::math::Matrix3d contact_frame;
      contact_frame.SetAxes(contact_x, contact_y, normal);
      wheel.contact_frame_rot.SetFromMatrix(contact_frame);

      if (reversed_axis[wheel_idx])
      {
        wheel.contact_frame_rot =
            wheel.contact_frame_rot * gz::math::Quaterniond(0, 0, M_PI);
      }
      wheel.contact_frame_rot.Normalize();
    }
  } else if(wheel.iters_without_contact < 3){
    // still considered in contact for 3 more steps since contacts might "jump"
    // Last wheel load and terrain type are kept
    wheel.in_contact = true;
    wheel.iters_without_contact++;
  }
}


GZ_ADD_PLUGIN(
    gz_terramechanics::TerramechanicsSystem,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate,
    gz::sim::ISystemPostUpdate,
    gz::sim::ISystemReset)


