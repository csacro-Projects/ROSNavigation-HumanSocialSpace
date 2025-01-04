#include<socialspace_navigation_layer/SocialspaceFunction.h>
#include<socialspace_navigation_layer/socialspace_layer.h>
#include<pluginlib/class_list_macros.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/QuaternionStamped.h>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2/LinearMath/Matrix3x3.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(socialspace_layer::SocialspaceLayer, costmap_2d::Layer
)

namespace socialspace_layer {
    SocialspaceLayer::SocialspaceLayer() {
        layered_costmap_ = NULL;
    }

    void SocialspaceLayer::onInitialize() {
        ros::NodeHandle nh("~/" + name_);
        current_ = true;
        first_time_ = true;

        // setup configuration of the costmap layer
        dsrv_ = new dynamic_reconfigure::Server<socialspace_navigation_layer::SocialspaceLayerConfig>(nh);
        dynamic_reconfigure::Server<socialspace_navigation_layer::SocialspaceLayerConfig>::CallbackType cb = boost::bind(
                &SocialspaceLayer::configure, this, _1, _2);
        dsrv_->setCallback(cb);

        // connect to services and subscribe to messages
        ros::NodeHandle n;
        client = n.serviceClient<socialspace_navigation_layer::SocialspaceFunction>("socialspace_function");
        humansWithInteractions_sub = nh.subscribe("/humans", 1, &SocialspaceLayer::humansWithInteractionsCallback,
                                                  this);
    }

    void
    SocialspaceLayer::humansWithInteractionsCallback(const humans::HumansWithInteractions &humansWithInteractions) {
        boost::recursive_mutex::scoped_lock lock(lock_);
        humansWithInteractions_ = humansWithInteractions;
    }

    void SocialspaceLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                        double *max_x, double *max_y) {
        if (!enabled_)
            return;
        boost::recursive_mutex::scoped_lock lock(lock_);

        // transform HumansWithInteractions into the correct frame
        std::string costmap_global_frame = layered_costmap_->getGlobalFrameID();
        transformed_humansWithInteractions_.clear();

        for (unsigned int i = 0; i < humansWithInteractions_.humans_with_interactions.size(); i++) {
            humans::HumanWithInteractions &humanWithInteractions = humansWithInteractions_.humans_with_interactions[i];
            humans::HumanWithInteractions t_humanWithInteractions;
            geometry_msgs::PointStamped pt, t_pt;
            tf2::Quaternion quaternion, t_quaternion;
            geometry_msgs::QuaternionStamped quat, t_quat;

            try {
                // position
                pt.header.frame_id = humansWithInteractions_.header.frame_id;
                pt.header.stamp = humansWithInteractions_.header.stamp;

                pt.point.x = humanWithInteractions.human.x;
                pt.point.y = humanWithInteractions.human.y;
                tf_->transform(pt, t_pt, costmap_global_frame);
                t_humanWithInteractions.human.x = t_pt.point.x;
                t_humanWithInteractions.human.y = t_pt.point.y;

                // (keep track of min and max in x and y direction)
                *min_x = std::min(*min_x, t_humanWithInteractions.human.x);
                *min_y = std::min(*min_y, t_humanWithInteractions.human.y);
                *max_x = std::max(*max_x, t_humanWithInteractions.human.x);
                *max_y = std::max(*max_y, t_humanWithInteractions.human.y);

                // orientation
                quaternion.setRPY(0, 0, humanWithInteractions.human.theta);
                quat.header.frame_id = humansWithInteractions_.header.frame_id;
                quat.header.stamp = humansWithInteractions_.header.stamp;
                quat.quaternion = tf2::toMsg(quaternion);
                tf_->transform(quat, t_quat, costmap_global_frame);
                tf2::fromMsg(t_quat.quaternion, t_quaternion);
                tf2::Matrix3x3 m(t_quaternion);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                t_humanWithInteractions.human.theta = yaw;

                t_humanWithInteractions.human.velocity = humanWithInteractions.human.velocity;
                t_humanWithInteractions.human.confidence = humanWithInteractions.human.confidence;

                // interaction with objects
                for (unsigned int j = 0; j < humanWithInteractions.object_interactions.size(); j++) {
                    humans::ObjectInteraction &obj_interaction = humanWithInteractions.object_interactions[j];
                    humans::ObjectInteraction t_obj_interaction;

                    // position start
                    pt.point.x = obj_interaction.item.x_start;
                    pt.point.y = obj_interaction.item.y_start;
                    tf_->transform(pt, t_pt, costmap_global_frame);
                    t_obj_interaction.item.x_start = t_pt.point.x;
                    t_obj_interaction.item.y_start = t_pt.point.y;

                    // position end
                    pt.point.x = obj_interaction.item.x_end;
                    pt.point.y = obj_interaction.item.y_end;
                    tf_->transform(pt, t_pt, costmap_global_frame);
                    t_obj_interaction.item.x_end = t_pt.point.x;
                    t_obj_interaction.item.y_end = t_pt.point.y;

                    // (keep track of min and max in x and y direction)
                    *min_x = std::min({*min_x, t_obj_interaction.item.x_start, t_obj_interaction.item.x_end});
                    *min_y = std::min({*min_y, t_obj_interaction.item.y_start, t_obj_interaction.item.y_end});
                    *max_x = std::max({*max_x, t_obj_interaction.item.x_start, t_obj_interaction.item.x_end});
                    *max_y = std::max({*max_y, t_obj_interaction.item.y_start, t_obj_interaction.item.y_end});

                    t_obj_interaction.confidence = obj_interaction.confidence;

                    t_humanWithInteractions.object_interactions.push_back(t_obj_interaction);
                }

                // interaction with humans
                for (unsigned int j = 0; j < humanWithInteractions.human_interactions.size(); j++) {
                    humans::HumanInteraction &human_interaction = humanWithInteractions.human_interactions[j];
                    humans::HumanInteraction t_human_interaction;

                    // traverse list of humans that are interacted with
                    for (unsigned int k = 0; k < human_interaction.item.size(); k++) {
                        humans::Human &human_interaction_human = human_interaction.item[k];
                        humans::Human t_human_interaction_human;

                        // position of human
                        pt.point.x = human_interaction_human.x;
                        pt.point.y = human_interaction_human.y;
                        tf_->transform(pt, t_pt, costmap_global_frame);
                        t_human_interaction_human.x = t_pt.point.x;
                        t_human_interaction_human.y = t_pt.point.y;

                        // (keep track of min and max in x and y direction)
                        *min_x = std::min(*min_x, t_human_interaction_human.x);
                        *min_y = std::min(*min_y, t_human_interaction_human.y);
                        *max_x = std::max(*max_x, t_human_interaction_human.x);
                        *max_y = std::max(*max_y, t_human_interaction_human.y);

                        // the following fields of human are not relevant for this kind of interaction,
                        // but we transform them anyways
                        quaternion.setRPY(0, 0, human_interaction_human.theta);
                        quat.header.frame_id = humansWithInteractions_.header.frame_id;
                        quat.header.stamp = humansWithInteractions_.header.stamp;
                        quat.quaternion = tf2::toMsg(quaternion);
                        tf_->transform(quat, t_quat, costmap_global_frame);
                        tf2::fromMsg(t_quat.quaternion, t_quaternion);
                        tf2::Matrix3x3 m(t_quaternion);
                        double roll, pitch, yaw;
                        m.getRPY(roll, pitch, yaw);
                        t_human_interaction_human.theta = yaw;

                        t_human_interaction_human.velocity = t_human_interaction_human.velocity;
                        t_human_interaction_human.confidence = t_human_interaction_human.confidence;

                        t_human_interaction.item.push_back(t_human_interaction_human);
                    }

                    // position of O-Space center
                    pt.point.x = human_interaction.center_x;
                    pt.point.y = human_interaction.center_y;
                    tf_->transform(pt, t_pt, costmap_global_frame);
                    t_human_interaction.center_x = t_pt.point.x;
                    t_human_interaction.center_y = t_pt.point.y;

                    // (keep track of min and max in x and y direction)
                    *min_x = std::min(*min_x, t_human_interaction.center_x);
                    *min_y = std::min(*min_y, t_human_interaction.center_y);
                    *max_x = std::max(*max_x, t_human_interaction.center_x);
                    *max_y = std::max(*max_y, t_human_interaction.center_y);

                    t_human_interaction.confidence = human_interaction.confidence;

                    t_humanWithInteractions.human_interactions.push_back(t_human_interaction);
                }

                transformed_humansWithInteractions_.push_back(t_humanWithInteractions);
            }
            catch (tf2::LookupException &ex) {
                ROS_ERROR("No Transform available Error: %s\n", ex.what());
                continue;
            }
            catch (tf2::ConnectivityException &ex) {
                ROS_ERROR("Connectivity Error: %s\n", ex.what());
                continue;
            }
            catch (tf2::ExtrapolationException &ex) {
                ROS_ERROR("Extrapolation Error: %s\n", ex.what());
                continue;
            }
        }

        // include minimal area around robot for updating bounds (keep track of min and max in x and y direction)
        *min_x = std::min(*min_x, robot_x - robot_bounds_);
        *min_y = std::min(*min_y, robot_y - robot_bounds_);
        *max_x = std::max(*max_x, robot_x + robot_bounds_);
        *max_y = std::max(*max_y, robot_y + robot_bounds_);

        // consider the previous min and max in x and y direction (as this area should be "cleaned")
        if (first_time_) {
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;
            first_time_ = false;
        } else {
            double a = *min_x, b = *min_y, c = *max_x, d = *max_y;
            *min_x = std::min(last_min_x_, *min_x);
            *min_y = std::min(last_min_y_, *min_y);
            *max_x = std::max(last_max_x_, *max_x);
            *max_y = std::max(last_max_y_, *max_y);
            last_min_x_ = a;
            last_min_y_ = b;
            last_max_x_ = c;
            last_max_y_ = d;
        }
    }

    void SocialspaceLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {
        if (!enabled_)
            return;
        boost::recursive_mutex::scoped_lock lock(lock_);

        // prepare call to service by setting all request parameters
        socialspace_navigation_layer::SocialspaceFunction srv;
        srv.request.humans_with_interactions = transformed_humansWithInteractions_;

        // (area to be updated)
        double min_x, min_y, max_x, max_y;
        master_grid.mapToWorld(min_i, min_j, min_x, min_y);
        master_grid.mapToWorld(max_i, max_j, max_x, max_y);
        srv.request.min_x = min_x;
        srv.request.min_y = min_y;
        srv.request.max_x = max_x;
        srv.request.max_y = max_y;
        srv.request.resolution = master_grid.getResolution();

        // ("hyper-parameters")
        srv.request.personal_sigma_h_factor = personal_sigma_h_factor_;
        srv.request.passing_right_hand_traffic = passing_right_hand_traffic_;
        srv.request.passing_sigma_h = passing_sigma_h_;
        srv.request.passing_sigma_s = passing_sigma_s_;
        srv.request.passing_sigma_r = passing_sigma_r_;
        srv.request.activity_object_sigma = activity_object_sigma_;
        srv.request.activity_human_sigma = activity_human_sigma_;

        if (client.call(srv)) {
            // call to service did succeed
            std::vector<double> costmap_values = srv.response.costmap_values;

            // update the costmap by setting the costs from costmap_values
            for (int j = min_j; j < max_j; j++) {
                for (int i = min_i; i < max_i; i++) {
                    // check the cost set there by the previous layer
                    unsigned char prev_layer_cost = master_grid.getCost(i, j);
                    if (prev_layer_cost == costmap_2d::NO_INFORMATION)
                        continue;

                    // check the cost to be set there by our layer
                    double cost_value = amplitude_ * costmap_values[(i - min_i) + (max_i - min_i) * (j - min_j)];
                    if (cost_value < cutoff_)
                        continue;

                    // set the new cost by applying the maximum operation between the previous cost and our cost
                    unsigned char our_layer_cost = (unsigned char) cost_value;
                    master_grid.setCost(i, j, std::max(our_layer_cost, prev_layer_cost));
                }
            }
        } else {
            // call to service did not succeed
            ROS_ERROR("error calling service socialspace_function\n");
        }
    }

    void SocialspaceLayer::configure(socialspace_navigation_layer::SocialspaceLayerConfig &config, uint32_t level) {
        enabled_ = config.enabled;
        cutoff_ = config.cutoff;
        amplitude_ = config.amplitude;
        robot_bounds_ = config.robot_bounds;

        personal_sigma_h_factor_ = config.personal_sigma_h_factor;

        passing_right_hand_traffic_ = config.passing_right_hand_traffic;
        passing_sigma_h_ = config.passing_sigma_h;
        passing_sigma_s_ = config.passing_sigma_s;
        passing_sigma_r_ = config.passing_sigma_r;

        activity_object_sigma_ = config.activity_object_sigma;
        activity_human_sigma_ = config.activity_human_sigma;
    }
}
