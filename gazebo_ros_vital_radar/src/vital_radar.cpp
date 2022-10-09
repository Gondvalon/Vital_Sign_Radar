#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <cmath>

#include "vital_radar.hh"

using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(VitalRadar)

VitalRadar::VitalRadar() {}

VitalRadar::~VitalRadar() {
    this->newLaserScansConnection.reset();

    this->sensor->SetActive(false);

    node_handle_->shutdown();
    delete node_handle_;
}

void VitalRadar::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
    this->sensor = std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);

    if(!this->sensor)
        gzthrow("RayPlugin requires Ray Sensor as its parent.\n");

    this->world = physics::get_world(this->sensor->WorldName());

    //default Parameters
    this->namespace_.clear();
    this->topic_ = "vitalRadar";
    this->frame_id_ = "/vital_radar_link";
    this->penetrableObjects = 1000;
    this->radarPower = 100000.0;
    this->gain = 1.0;
    this->receivableSignalArea = 0.01;
    this->defaultDamping = 1.0;
    this->minDetectablePower = 0.0;

    if (_sdf->HasElement("robotNamespace")) {
        this->namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
    }
    if (_sdf->HasElement("topic_")) {
        this->topic_ = _sdf->GetElement("topic_")->GetValue()->GetAsString();
    }
    if (_sdf->HasElement("topicName")) {
        topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();
    }

    if (_sdf->HasElement("penetrableObjects")) {
        this->penetrableObjects = std::stoi(_sdf->GetElement("penetrableObjects")->GetValue()->GetAsString());
    }
    if (_sdf->HasElement("radarPower")) {
        this->radarPower = std::stod(_sdf->GetElement("radarPower")->GetValue()->GetAsString());
    }
    if (_sdf->HasElement("gain")) {
        this->gain = std::stod(_sdf->GetElement("gain")->GetValue()->GetAsString());
    }
    if (_sdf->HasElement("receivableSignalArea")) {
        this->receivableSignalArea = std::stod(_sdf->GetElement("receivableSignalArea")->GetValue()->GetAsString());
    }
    if (_sdf->HasElement("defaultDamping")) {
        this->defaultDamping = std::stod(_sdf->GetElement("defaultDamping")->GetValue()->GetAsString());
    }
    if (_sdf->HasElement("minDetectablePower")) {
        this->minDetectablePower = std::stod(_sdf->GetElement("minDetectablePower")->GetValue()->GetAsString());
    }

    sensor_model_.Load(_sdf);

    this->multipleVitalSignsMsg.header.frame_id = frame_id_;

    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
        return;
    }

    node_handle_ = new ros::NodeHandle(namespace_);
    publisher_ = node_handle_->advertise<vital_sign_msgs::VitalSigns>(topic_, 100);

    Reset();

    this->newLaserScansConnection =
            this->sensor->LaserShape()->ConnectNewLaserScans(std::bind(&VitalRadar::OnNewLaserScans, this));

    this->sensor->SetActive(true);
};

void VitalRadar::OnNewLaserScans() {
    //update MultiRayShape collision box position
    physics::CollisionPtr coll = boost::dynamic_pointer_cast<physics::Collision>(this->sensor->LaserShape()->GetParent());
    coll->SetWorldPoseDirty();

    std::string nameOfHitModel;

    std::vector<double> ranges;
    this->sensor->Ranges(ranges);

    if (ranges.size() <= 0) {
        return;
    }

    struct wall {
        double thickness;
        double dampingCoeff;
    };
    std::vector<wall> currentPath;
    struct human {
        std::string name;
        double distance;
        double heartRate;
        double respiratoryRate;
        double receivedPower;
        double rayReflectingArea;
        ignition::math::Vector3d position;
    };
    std::vector<human> humanObjects;

    physics::ModelPtr modelHitByRay;
    physics::RayShapePtr rayShape;
    ignition::math::Vector3d rayStart;
    ignition::math::Vector3d rayEnd;
    ignition::math::Vector3d rayGradient;
    for(int i = 0; i < ranges.size(); i++) {
        //update RayShape collision box position
        physics::CollisionPtr rayColl = boost::dynamic_pointer_cast<physics::Collision>(this->sensor->LaserShape()->Ray(i)->GetParent());
        rayColl->SetWorldPoseDirty();

        if (ranges.at(i) > this->sensor->RangeMax()) {
            continue;
        }

        //setup for each ray
        double rayTravelDist = this->sensor->RangeMin();
        int penetratedWalls = 0;
        std::string oldNameOfHitModel = "";
        double raySignalStrength = 0.0;
        currentPath.clear();
        while (rayTravelDist <= this->sensor->RangeMax()) {
            this->wallDamping = this->defaultDamping;
            rayShape = this->sensor->LaserShape()->Ray(i);

            double sectionTilHit = 0.0;
            rayShape->GetIntersection(sectionTilHit, nameOfHitModel);

            rayShape->RelativePoints(rayStart, rayEnd);
            rayGradient = rayEnd - rayStart;
            rayGradient = rayGradient.Normalize();

            //if no object hit then sectionTilHit will be 1000
            if (sectionTilHit > this->sensor->RangeMax()) {
                break;
            }

            std::size_t seperator;
            seperator = nameOfHitModel.find("::");
            nameOfHitModel = nameOfHitModel.substr(0, seperator);
            modelHitByRay = this->world->ModelByName(nameOfHitModel);

            currentPath.push_back(wall());
            currentPath[currentPath.size()-1].thickness = sectionTilHit;

            if (nameOfHitModel == oldNameOfHitModel) {
                penetratedWalls++;
                if (modelHitByRay->GetSDF()->HasElement("vitalRadar:damping")) {
                    this->wallDamping = std::stod(modelHitByRay->GetSDF()->GetElement("vitalRadar:damping")->GetValue()->GetAsString());
                    currentPath[currentPath.size()-1].dampingCoeff = this->wallDamping;
                }
            } else {
                currentPath[currentPath.size()-1].dampingCoeff = this->defaultDamping;
            }

            if (penetratedWalls > this->penetrableObjects) {
                break;
            }

            //finding and calculating found vital signs
            if (!(nameOfHitModel == oldNameOfHitModel) && (modelHitByRay->GetSDF()->HasElement("human:heartRate") &&
                    modelHitByRay->GetSDF()->HasElement("human:respiratoryRate"))) {

                //calculate surface area for ray
                double horizontalAngle = (this->sensor->AngleMax().Radian() - this->sensor->AngleMin().Radian()) /
                        (this->sensor->RayCount() - 1);
                double verticalAngle = (this->sensor->VerticalAngleMax().Radian() - this->sensor->AngleMin().Radian()) /
                        (this->sensor->VerticalRayCount() -1);
                double raySurfaceCoverage = sin(horizontalAngle) * (rayTravelDist + sectionTilHit) *
                        sin(verticalAngle) * (rayTravelDist + sectionTilHit);

                //calculate received power without area
                double medianDampingCoeff = 0.0;
                double sectionPercentage;
                double rayLength = rayTravelDist + sectionTilHit - this->sensor->RangeMin();
                for (int i = 0; i < currentPath.size(); i++) {
                    sectionPercentage = currentPath[i].thickness / (rayLength);
                    medianDampingCoeff = medianDampingCoeff + sectionPercentage * currentPath[i].dampingCoeff;
                }
                raySignalStrength = this->radarPower * this->gain * this->receivableSignalArea /
                        (medianDampingCoeff * pow(rayLength, 4.0) * pow((4 * M_PI), 2.0));

                double heartRate = std::stod(modelHitByRay->GetSDF()->GetElement("human:heartRate")->GetValue()->GetAsString());
                double respiratoryRate = std::stod(modelHitByRay->GetSDF()->GetElement("human:respiratoryRate")->GetValue()->GetAsString());

                if (raySignalStrength > this->minDetectablePower ) {
                    humanObjects.push_back(human());
                    humanObjects[humanObjects.size()-1].name = nameOfHitModel;
                    humanObjects[humanObjects.size()-1].heartRate = heartRate;
                    humanObjects[humanObjects.size()-1].respiratoryRate = respiratoryRate;
                    humanObjects[humanObjects.size()-1].distance = rayLength;
                    humanObjects[humanObjects.size()-1].receivedPower = raySignalStrength;
                    humanObjects[humanObjects.size()-1].rayReflectingArea = raySurfaceCoverage;
                    humanObjects[humanObjects.size()-1].position = rayEnd;
                }
            }
            rayTravelDist = rayTravelDist + sectionTilHit + 0.0001;
            oldNameOfHitModel = nameOfHitModel;
            rayShape->SetPoints(rayGradient * rayTravelDist, rayGradient * this->sensor->RangeMax());
        }
        //resets ray
        rayShape->SetPoints(rayGradient * this->sensor->RangeMin(), rayGradient * this->sensor->RangeMax());
    }

    //handling of found human objects
    std::sort(humanObjects.begin(), humanObjects.end(), [](human a, human b) {
        return a.name < b.name;
    });
    double reflectingArea;
    for (int i = 0; i < humanObjects.size(); i++) {
        reflectingArea = humanObjects[i].rayReflectingArea;
        while (humanObjects[i].name == humanObjects[i+1].name) {
            if (humanObjects.size() <2 || (i+1) >= humanObjects.size()) {
                break;
            }
            reflectingArea = reflectingArea + humanObjects[i+1].rayReflectingArea;
            if (humanObjects[i].receivedPower > humanObjects[i+1].receivedPower) {
                humanObjects.erase(humanObjects.begin()+i+1);
            } else if (humanObjects[i].name == humanObjects[i+1].name) {
                humanObjects.erase(humanObjects.begin()+i);
            }
        }
        humanObjects[i].receivedPower = humanObjects[i].receivedPower * reflectingArea;
    }

    //calculate the vital signs with noise and send the ros message
    double deviationPercentage;
    vital_sign_msgs::VitalSignMeasurement vitalSignMsg;
    for (int i = 0; i<humanObjects.size(); i++) {
        deviationPercentage = 0.2 + pow(((humanObjects[i].receivedPower - this->minDetectablePower) /
                (this->radarPower - this->minDetectablePower)), 1.0/4.0) * - 0.2;
        double heartDeviation = humanObjects[i].heartRate * deviationPercentage;
        double respiratoryDeviation = humanObjects[i].respiratoryRate * deviationPercentage;
        std::normal_distribution<> distHeart(humanObjects[i].heartRate, heartDeviation);
        std::normal_distribution<> distRespiratory(humanObjects[i].respiratoryRate, respiratoryDeviation);

        std::random_device seed{};
        std::mt19937 generator{seed()};
        double noisyHeartRate = distHeart(generator);
        double noisyRespiratoryRate = distRespiratory(generator);

        vitalSignMsg.location.x = humanObjects[i].position.X();
        vitalSignMsg.location.y = humanObjects[i].position.Y();
        vitalSignMsg.location.z = humanObjects[i].position.Z();
        vitalSignMsg.heart_rate = noisyHeartRate;
        vitalSignMsg.breathing_rate = noisyRespiratoryRate;
        vitalSignMsg.heart_rate_variance = heartDeviation;
        vitalSignMsg.breathing_rate_variance = respiratoryDeviation;

        this->multipleVitalSignsMsg.measurements.push_back(vitalSignMsg);
    }
    publisher_.publish(this->multipleVitalSignsMsg)

    printf("\n");
}
