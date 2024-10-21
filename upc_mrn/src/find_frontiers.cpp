#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "upc_mrn/msg/frontiers.hpp"
#include <boost/pending/disjoint_sets.hpp>
#include <numeric>
#include <algorithm>

class FindFrontiers : public rclcpp::Node
{
private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_frontiers_pub_, map_free_pub_, map_filtered_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<upc_mrn::msg::Frontiers>::SharedPtr frontiers_pub_;

    visualization_msgs::msg::MarkerArray markers_;
    int min_frontier_size_;
    double cell_size_;

public:
    FindFrontiers();

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid &msg);
    std::vector<int> twoPassLabeling(const nav_msgs::msg::OccupancyGrid &map_frontiers, std::map<int, int> &labels_sizes, bool diagonal) const;
    bool isFrontier(const int &cell, const nav_msgs::msg::OccupancyGrid &map) const;
    void publishMarkers(const upc_mrn::msg::Frontiers &frontiers_msg);

    // UTILS
    int point2cell(const geometry_msgs::msg::Point &point, const nav_msgs::msg::OccupancyGrid &map_) const;
    geometry_msgs::msg::Point cell2point(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;

    std::vector<int> getStraightPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    std::vector<int> getAdjacentPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;

    int rightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int uprightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int upCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int upleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int leftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int downleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int downCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int downrightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int floor0(const float &value) const;
};

FindFrontiers::FindFrontiers() : Node("find_frontiers"), cell_size_(-1)
{
    // Declare & load parameter with default values
    get_parameter_or("min_frontier_size", min_frontier_size_, 5); // default 5 cells

    // publisher and subscribers
    frontiers_pub_ = this->create_publisher<upc_mrn::msg::Frontiers>("/frontiers", 1);
    map_frontiers_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_frontiers", 1);
    map_free_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_free", 1);
    map_filtered_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_filtered", 1);
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("frontiers_markers", 1);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 1, std::bind(&FindFrontiers::mapCallback, this, std::placeholders::_1));
}

void FindFrontiers::mapCallback(const nav_msgs::msg::OccupancyGrid &msg)
{
    RCLCPP_DEBUG(this->get_logger(), "map received!");

    nav_msgs::msg::OccupancyGrid map_occupancy = msg;
    nav_msgs::msg::OccupancyGrid map_frontiers = msg;
    nav_msgs::msg::OccupancyGrid map_free = msg;

    // init cell_size
    if (cell_size_ < 0)
        cell_size_ = msg.info.resolution;

    // reset maps
    map_frontiers.data.assign(map_frontiers.data.size(), 0);
    map_free.data.assign(map_free.data.size(), 0);

    // FILTER UNCONNECTED FREE CELLS
    // create map free (assign each cell if it is free)
    for (unsigned int i = 0; i < map_free.data.size(); ++i)
    {
        if (map_occupancy.data[i] == 0)
            map_free.data[i] = 100;
    }
    // Label free (connected cells)
    std::map<int, int> labels_free_sizes;
    std::vector<int> labels_free = twoPassLabeling(map_free, labels_free_sizes, false);
    // Classify as unknown all free groups except for the biggest one
    int remaining_label = std::max_element(labels_free_sizes.begin(),
                                           labels_free_sizes.end(),
                                           [](const std::pair<int, int> &p1, const std::pair<int, int> &p2)
                                           { return p1.second < p2.second; })
                              ->first;
    for (unsigned int i = 0; i < map_occupancy.data.size(); ++i)
    {
        if (map_occupancy.data[i] == 0 and labels_free[i] != remaining_label)
        {
            map_occupancy.data[i] = -1;
            map_free.data[i] = 0;
        }
    }
    // publish map_free
    map_free_pub_->publish(map_free);
    RCLCPP_DEBUG(this->get_logger(), "map free published!");

    // publish map_free
    map_filtered_pub_->publish(map_occupancy);
    RCLCPP_DEBUG(this->get_logger(), "map filtered published!");

    // FIND FRONTIERS
    // create map frontiers (assign each cell if it is frontier)
    for (unsigned int i = 0; i < map_frontiers.data.size(); ++i)
    {
        if (isFrontier(i, map_occupancy))
            map_frontiers.data[i] = 100;
    }

    // publish map_frontiers
    map_frontiers_pub_->publish(map_frontiers);
    RCLCPP_DEBUG(this->get_logger(), "map frontiers published!");

    // Label frontiers (connected cells)
    std::map<int, int> labels_sizes;
    std::vector<int> labels = twoPassLabeling(map_frontiers, labels_sizes, true);

    // Create and fill frontiers message
    std::vector<int> frontiers_labels;
    upc_mrn::msg::Frontiers frontiers_msg;
    frontiers_msg.header = msg.header;
    frontiers_msg.frontiers.clear();
    for (unsigned int i = 0; i < labels.size(); ++i)
    {
        if (labels[i] != 0) // labeled cell
        {
            // if too small continue
            if (labels_sizes[labels[i]] < min_frontier_size_)
                continue;

            // search existing frontier
            bool new_label = true;
            for (unsigned int j = 0; j < frontiers_msg.frontiers.size(); j++)
            {
                // found
                if (frontiers_labels[j] == labels[i])
                {
                    frontiers_msg.frontiers[j].cells.push_back(i);
                    frontiers_msg.frontiers[j].cells_points.push_back(cell2point(i, map_frontiers));
                    new_label = false;
                    break;
                }
            }
            // not found: create new frontier
            if (new_label)
            {
                upc_mrn::msg::Frontier new_frontier;
                new_frontier.size = labels_sizes[labels[i]];
                new_frontier.cells.push_back(i);
                new_frontier.cells_points.push_back(cell2point(i, map_frontiers));
                frontiers_msg.frontiers.push_back(new_frontier);
                frontiers_labels.push_back(labels[i]);
            }
        }
    }

    // Compute center cell
    for (unsigned int i = 0; i < frontiers_msg.frontiers.size(); ++i)
    {
        int label = frontiers_labels[i];

        // order the frontier cells
        std::deque<int> ordered_cells(0);
        ordered_cells.push_back(frontiers_msg.frontiers[i].cells.front());
        while (ordered_cells.size() < frontiers_msg.frontiers[i].size)
        {
            auto initial_size = ordered_cells.size();

            // connect cells to first cell
            std::vector<int> frontAdjacentPoints = getAdjacentPoints(ordered_cells.front(), map_frontiers);
            for (unsigned int k = 0; k < frontAdjacentPoints.size(); k++)
                if (frontAdjacentPoints[k] != -1 && labels[frontAdjacentPoints[k]] == label && std::find(ordered_cells.begin(), ordered_cells.end(), frontAdjacentPoints[k]) == ordered_cells.end())
                {
                    ordered_cells.push_front(frontAdjacentPoints[k]);
                    break;
                }

            // connect cells to last cell
            std::vector<int> backAdjacentPoints = getAdjacentPoints(ordered_cells.back(), map_frontiers);
            for (unsigned int k = 0; k < backAdjacentPoints.size(); k++)
                if (backAdjacentPoints[k] != -1 && labels[backAdjacentPoints[k]] == label && std::find(ordered_cells.begin(), ordered_cells.end(), backAdjacentPoints[k]) == ordered_cells.end())
                {
                    ordered_cells.push_back(backAdjacentPoints[k]);
                    break;
                }

            if (initial_size == ordered_cells.size() && ordered_cells.size() < frontiers_msg.frontiers[i].size)
                break;
        }
        // center cell
        frontiers_msg.frontiers[i].center_cell = ordered_cells[ordered_cells.size() / 2];
        frontiers_msg.frontiers[i].center_point = cell2point(frontiers_msg.frontiers[i].center_cell, map_frontiers);
    }

    // Publish
    frontiers_pub_->publish(frontiers_msg);
    RCLCPP_DEBUG(this->get_logger(), "frontiers published!");
    publishMarkers(frontiers_msg);
    RCLCPP_DEBUG(this->get_logger(), "marker array published!");
}

// Check if a cell is frontier (free close to unknown)
bool FindFrontiers::isFrontier(const int &cell, const nav_msgs::msg::OccupancyGrid &map) const
{
    if (map.data[cell] == 0) // check if it is free
    {
        auto straightPoints = getStraightPoints(cell, map);
        for (unsigned int i = 0; i < straightPoints.size(); ++i)
            if (straightPoints[i] != -1 && map.data[straightPoints[i]] == -1) // check if any neigbor is unknown
                return true;
    }
    // If it is obstacle or unknown, it can not be frontier
    return false;
}

// Two pass labeling to label frontiers [http://en.wikipedia.org/wiki/Connected-component_labeling]
std::vector<int> FindFrontiers::twoPassLabeling(const nav_msgs::msg::OccupancyGrid &map_labelr, std::map<int, int> &labels_sizes, bool diagonal) const
{
    labels_sizes.clear();
    std::vector<int> labels(map_labelr.data.size());
    labels.assign(map_labelr.data.begin(), map_labelr.data.end());

    std::vector<int> neigh_labels;
    std::vector<int> rank(1000);
    std::vector<int> parent(1000);
    boost::disjoint_sets<int *, int *> dj_set(&rank[0], &parent[0]);
    int current_label_ = 1;

    // 1ST PASS: Assign temporary labels to frontiers and establish relationships
    for (unsigned int i = 0; i < map_labelr.data.size(); i++)
    {
        if (map_labelr.data[i] != 0)
        {
            neigh_labels.clear();
            // Find )8 or 4) connectivity neighbours already labeled
            if (diagonal && upleftCell(i, map_labelr) != -1 && labels[upleftCell(i, map_labelr)] != 0)
                neigh_labels.push_back(labels[upleftCell(i, map_labelr)]);
            if (upCell(i, map_labelr) != -1 && labels[upCell(i, map_labelr)] != 0)
                neigh_labels.push_back(labels[upCell(i, map_labelr)]);
            if (diagonal && uprightCell(i, map_labelr) != -1 && labels[uprightCell(i, map_labelr)] != 0)
                neigh_labels.push_back(labels[uprightCell(i, map_labelr)]);
            if (leftCell(i, map_labelr) != -1 && labels[leftCell(i, map_labelr)] != 0)
                neigh_labels.push_back(labels[leftCell(i, map_labelr)]);

            if (neigh_labels.empty()) // case: No neighbours
            {
                dj_set.make_set(current_label_); //   create new set of labels
                labels[i] = current_label_;      //   update cell's label
                current_label_++;                //   update label
            }
            else // case: With neighbours
            {
                labels[i] = *std::min_element(neigh_labels.begin(), neigh_labels.end()); //   choose minimum label of the neighbours
                for (unsigned int j = 0; j < neigh_labels.size(); ++j)                   //   update neighbours sets
                    dj_set.union_set(labels[i], neigh_labels[j]);                        //   unite sets minimum label with the others
            }
        }
    }

    // 2ND PASS: Assign final label
    dj_set.compress_sets(labels.begin(), labels.end());
    // compress sets for efficiency
    for (unsigned int i = 0; i < map_labelr.data.size(); i++)
        if (labels[i] != 0)
        {
            // relabel each element with the lowest equivalent label
            labels[i] = dj_set.find_set(labels[i]);
            // increment the size of the label
            if (labels_sizes.count(labels[i]) == 0)
                labels_sizes[labels[i]] = 1;
            else
                labels_sizes[labels[i]]++;
        }

    return labels;
}

void FindFrontiers::publishMarkers(const upc_mrn::msg::Frontiers &frontiers_msg)
{
    // resize
    markers_.markers.resize(frontiers_msg.frontiers.size() * 3 + 1); // each frontier: cells, center_point and text

    // deleteall
    markers_.markers[0].action = visualization_msgs::msg::Marker::DELETEALL;

    // omplir
    for (unsigned int i = 0; i < frontiers_msg.frontiers.size(); i++)
    {
        std_msgs::msg::ColorRGBA c;
        double x = (double)i / (double)frontiers_msg.frontiers.size();
        c.a = 1.0;
        c.r = x < 0.33 ? 1 - 0.33 * x : (x < 0.66 ? 0 : 3 * (x - 0.66));
        c.g = x < 0.33 ? 3 * x : (x < 0.66 ? 1 - 0.33 * (x - 0.33) : 0);
        c.b = x < 0.33 ? 0 : (x < 0.66 ? 3 * (x - 0.33) : 1 - 0.33 * (x - 0.66));

        // RCLCPP_INFO(this->get_logger(), "Colors: i = %u\nx = %f\nr = %f\ng = %f\nb = %f", i, x, c.r, c.g, c.b);

        // point center
        visualization_msgs::msg::Marker marker_center;
        marker_center.header.frame_id = frontiers_msg.header.frame_id;
        marker_center.header.stamp = this->now();
        marker_center.lifetime = rclcpp::Duration(0, 0);
        marker_center.type = visualization_msgs::msg::Marker::CYLINDER;
        marker_center.scale.x = marker_center.scale.y = 0.1;
        marker_center.scale.z = 0.5;
        marker_center.color = c;
        marker_center.ns = "centers";
        marker_center.id = 3 * i;
        marker_center.pose.position.x = frontiers_msg.frontiers[i].center_point.x;
        marker_center.pose.position.y = frontiers_msg.frontiers[i].center_point.y;
        marker_center.pose.position.z = marker_center.scale.z / 2;
        marker_center.pose.orientation.x = marker_center.pose.orientation.y = marker_center.pose.orientation.z = 0;
        marker_center.pose.orientation.w = 1;
        markers_.markers[3 * i + 1] = marker_center;

        // cells
        visualization_msgs::msg::Marker marker_points;
        marker_points.header.frame_id = frontiers_msg.header.frame_id;
        marker_points.header.stamp = this->now();
        marker_points.lifetime = rclcpp::Duration(0, 0);
        marker_points.type = visualization_msgs::msg::Marker::POINTS;
        marker_points.pose.orientation.x = marker_points.pose.orientation.y = marker_points.pose.orientation.z = 0;
        marker_points.pose.orientation.w = 1;
        marker_points.points = frontiers_msg.frontiers[i].cells_points;
        marker_points.scale.x = marker_points.scale.y = marker_points.scale.z = cell_size_;
        marker_points.colors = std::vector<std_msgs::msg::ColorRGBA>(marker_points.points.size(), c);
        marker_points.ns = "cells";
        marker_points.id = 3 * i + 1;
        markers_.markers[3 * i + 2] = marker_points;

        // text
        visualization_msgs::msg::Marker marker_id;
        marker_id.header.frame_id = frontiers_msg.header.frame_id;
        marker_id.header.stamp = this->now();
        marker_id.lifetime = rclcpp::Duration(0, 0);
        marker_id.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker_id.scale.x = marker_id.scale.y = 1;
        marker_id.scale.z = 0.5;
        marker_id.text = std::to_string(i);
        marker_id.color = c;
        marker_id.pose = marker_center.pose;
        marker_id.pose.position.z = marker_center.scale.z + 0.2;
        marker_id.ns = "id";
        marker_id.id = 3 * i + 2;
        markers_.markers[3 * i + 3] = marker_id;
    }
    // publish
    markers_pub_->publish(markers_);
}

// UTILS ////////////////////////////////////////////////////////////////////////////////////////////////////
int FindFrontiers::point2cell(const geometry_msgs::msg::Point &point, const nav_msgs::msg::OccupancyGrid &map_) const
{
    if (point.x <= map_.info.origin.position.x || point.x >= map_.info.width * map_.info.resolution + map_.info.origin.position.x ||
        point.y <= map_.info.origin.position.y || point.y >= map_.info.height * map_.info.resolution + map_.info.origin.position.y)
    {
        return -1;
    }

    int x_cell = floor0((point.x - map_.info.origin.position.x) / map_.info.resolution);
    int y_cell = floor0((point.y - map_.info.origin.position.y) / map_.info.resolution);
    int cell = x_cell + (y_cell)*map_.info.width;
    return cell;
}

geometry_msgs::msg::Point FindFrontiers::cell2point(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    geometry_msgs::msg::Point point;
    point.x = (cell % map_.info.width) * map_.info.resolution + map_.info.origin.position.x + map_.info.resolution / 2;
    point.y = floor(cell / map_.info.width) * map_.info.resolution + map_.info.origin.position.y + map_.info.resolution / 2;
    return point;
}

std::vector<int> FindFrontiers::getStraightPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    return std::vector<int>({leftCell(cell, map_),
                             upCell(cell, map_),
                             rightCell(cell, map_),
                             downCell(cell, map_)});
}
std::vector<int> FindFrontiers::getAdjacentPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    return std::vector<int>({leftCell(cell, map_),
                             upCell(cell, map_),
                             rightCell(cell, map_),
                             downCell(cell, map_),
                             upleftCell(cell, map_),
                             uprightCell(cell, map_),
                             downrightCell(cell, map_),
                             downleftCell(cell, map_)});
}
int FindFrontiers::rightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    // only go left if no index error and if current cell is not already on the left boundary
    if ((cell % map_.info.width != 0))
        return cell + 1;

    return -1;
}
int FindFrontiers::uprightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    if ((cell % map_.info.width != 0) && (cell >= (int)map_.info.width))
        return cell - map_.info.width + 1;

    return -1;
}
int FindFrontiers::upCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    if (cell >= (int)map_.info.width)
        return cell - map_.info.width;

    return -1;
}
int FindFrontiers::upleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    if ((cell >= (int)map_.info.width) && ((cell + 1) % (int)map_.info.width != 0))
        return cell - map_.info.width - 1;

    return -1;
}
int FindFrontiers::leftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    if ((cell + 1) % map_.info.width != 0)
        return cell - 1;

    return -1;
}
int FindFrontiers::downleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    if (((cell + 1) % map_.info.width != 0) && ((cell / map_.info.width) < (map_.info.height - 1)))
        return cell + map_.info.width - 1;

    return -1;
}
int FindFrontiers::downCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    if ((cell / map_.info.width) < (map_.info.height - 1))
        return cell + map_.info.width;

    return -1;
}
int FindFrontiers::downrightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    if (((cell / map_.info.width) < (map_.info.height - 1)) && (cell % map_.info.width != 0))
        return cell + map_.info.width + 1;

    return -1;
}
int FindFrontiers::floor0(const float &value) const
{
    if (value < 0.0)
        return ceil(value);
    else
        return floor(value);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FindFrontiers>());
    rclcpp::shutdown();
    return 0;
}
