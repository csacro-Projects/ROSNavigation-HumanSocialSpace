#ifndef SOCIALSPACE_LAYER_H_
#define SOCIALSPACE_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>
#include <socialspace_navigation_layer/SocialspaceLayerConfig.h>
#include <humans/HumansWithInteractions.h>

namespace socialspace_layer {
    class SocialspaceLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D {
    public:
        SocialspaceLayer();

        virtual void onInitialize();

        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                                  double *min_x, double *min_y, double *max_x, double *max_y);

        virtual void updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);

    private:
        bool first_time_;
        double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

        boost::recursive_mutex lock_;
        humans::HumansWithInteractions humansWithInteractions_;
        std::vector<humans::HumanWithInteractions> transformed_humansWithInteractions_;

        ros::ServiceClient client;
		ros::Subscriber humansWithInteractions_sub;
        void humansWithInteractionsCallback(const humans::HumansWithInteractions &humansWithInteractions);

        void configure(socialspace_navigation_layer::SocialspaceLayerConfig &config, uint32_t level);
        dynamic_reconfigure::Server<socialspace_navigation_layer::SocialspaceLayerConfig> *dsrv_;
        double cutoff_, amplitude_, robot_bounds_;
        double personal_sigma_h_factor_;
        bool passing_right_hand_traffic_;
        double passing_sigma_h_, passing_sigma_s_, passing_sigma_r_;
        double activity_object_sigma_, activity_human_sigma_;
    };
}
#endif