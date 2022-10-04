#ifndef GAZEBO_PLUGINS_VITALRADAR_HH_
#define GAZEBO_PLUGINS_VITALRADAR_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/RayShape.hh>
#include <gazebo/util/system.hh>
#include <ignition/math/Vector3.hh>
#include <list>

namespace gazebo {
    class GZ_PLUGIN_VISIBLE VitalRadar : public SensorPlugin {
    public:
        VitalRadar();

        virtual ~VitalRadar();

        virtual void OnNewLaserScans();

        void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

        
    private:
        sensors::RaySensorPtr sensor;
        physics::WorldPtr world;

        std::string namespace_;
        std::string topic_;
        std::string frame_id_;

        event::ConnectionPtr newLaserScansConnection;

        std::vector<double> ranges;

        physics::RayShapePtr rayShape;

        //calculating new Rays
        ignition::math::Vector3d rayStart;
        ignition::math::Vector3d rayEnd;
        ignition::math::Vector3d rayGradient;

        double heartRate;
        double respiratoryRate;

        //parameters changable in sdf sensor plugin
        int penetrableObjects;
        double radarPower;
        double receivableSignalArea;
        double gain;
        //damping should be >=1
        double defaultDamping;
        double wallDamping;
        double minDetectablePower;
    };
}
#endif