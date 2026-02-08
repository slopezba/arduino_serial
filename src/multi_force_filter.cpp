#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <dynamic_reconfigure/server.h>
#include <vector>
#include <arduino_serial/ForceFilterConfig.h>

class MultiForceFilterNode
{
public:
    MultiForceFilterNode() {
        sub_ = nh_.subscribe("/girona500/payload/external/arduino/adc",
                             1,
                             &MultiForceFilterNode::adcCallback,
                             this);
        
        sensor_names_ = {
            "right/front",
            "left/front",
            "left/lateral",
            "right/lateral"
        };

        for(int i=0;i<4;i++)
        {
            pubs_.push_back(
                nh_.advertise<geometry_msgs::WrenchStamped>(
                    "/girona500/bravo/gripper/" + sensor_names_[i] + "/wrench", 1)
            );
        }
        filtered_.assign(4, 0.0);

        dynamic_reconfigure::Server<arduino_serial::ForceFilterConfig>::CallbackType f;
        f = boost::bind(&MultiForceFilterNode::configCallback, this, _1, _2);
        server_.setCallback(f);
        // ===== tabla calibración sensor 111N =====
        ref_mass_ = {0,25,65,90,100,125,165,190,215,255,280,355,380,480,570,670,760,860,950,1150};
        ref_read_ = {70,75,110,130,160,175,185,210,260,270,350,360,530,620,650,675,710,730,780,810};
    }
    private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::vector<ros::Publisher> pubs_;
    std::vector<double> filtered_;
    std::vector<double> ref_mass_;
    std::vector<double> ref_read_;
    std::vector<std::string> sensor_names_;

    dynamic_reconfigure::Server<arduino_serial::ForceFilterConfig> server_;
    double alpha_ = 0.5;
    const double GRAVITY = 9.81;

    void configCallback(arduino_serial::ForceFilterConfig &config, uint32_t level)
    {
        alpha_ = config.alpha;
    }

    // interpolación lineal
    double interpolateForce(double reading)
    {
        if(reading <= ref_read_.front())
            return (ref_mass_.front()/1000.0)*GRAVITY;

        if(reading >= ref_read_.back())
            return (ref_mass_.back()/1000.0)*GRAVITY;

        for(size_t i=0;i<ref_read_.size()-1;i++)
        {
            if(reading >= ref_read_[i] && reading <= ref_read_[i+1])
            {
                double t = (reading - ref_read_[i]) /
                           (ref_read_[i+1] - ref_read_[i]);

                double mass = ref_mass_[i] +
                              t*(ref_mass_[i+1]-ref_mass_[i]);

                return (mass/1000.0)*GRAVITY;
            }
        }
        return 0.0;
    }

    void adcCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
    {
        if(msg->data.size()<4)
            return;

        for(int i=0;i<4;i++)
        {
            double raw = msg->data[i];

            // filtro exponencial
            filtered_[i] = alpha_*raw + (1.0-alpha_)*filtered_[i];

            // conversión ADC → Fuerza
            double force = interpolateForce(filtered_[i]);

            geometry_msgs::WrenchStamped wrench;
            wrench.header.stamp = ros::Time::now();
            wrench.header.frame_id ="girona500/bravo/gripper/" + sensor_names_[i];

            wrench.wrench.force.x = 0.0;
            wrench.wrench.force.y = 0.0;
            wrench.wrench.force.z = force;

            pubs_[i].publish(wrench);
        }
    }
};

int main(int argc,char** argv)
{
    ros::init(argc,argv,"multi_force_filter");
    MultiForceFilterNode node;
    ros::spin();
    return 0;
}
